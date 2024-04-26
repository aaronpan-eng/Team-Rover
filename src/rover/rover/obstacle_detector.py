from argparse import ArgumentParser
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import List

import numpy as np
from sklearn import cluster


@dataclass
class Obstacle:
    forward: float
    left: float
    radius: float

    to_dict = asdict


@dataclass
class Scan:
    forward_range: np.ndarray
    """forward distance to scan point [m]"""
    left_range: np.ndarray
    """left distance to scan point [m]"""

    @classmethod
    def from_ranges(cls, angle_min, angle_max, angle_increment,
                    time_increment, scan_time,
                    range_min, range_max, ranges,
                    intensities) -> "Scan":
        """Expects lidar is placed with Z axis up and X axis forward so that angles are counterclockwise around Z.

        Ranges less than range_min or greater than range_max are discarded.

        Doesn't account for scan time or beam width currently.

        Angle_increment is the angular distance *between beams* which have nonzero width. So it's less than the angular
        distance between points.
        """

        if ranges[-1] == '...':
            ranges = ranges[:-1]
            intensities = intensities[:-1]
        ranges = np.array(ranges)
        angles = np.linspace(angle_min, angle_max, len(ranges))
        intensities = np.array(intensities)

        filtered_points = (range_min <= ranges) & (ranges <= range_max) & np.isfinite(intensities)
        ranges = ranges[filtered_points]
        angles = angles[filtered_points]

        forward = ranges * np.cos(angles)
        left = ranges * np.sin(angles)

        return cls(forward, left)

    def cluster(self, plot=None):
        X = np.array([
            self.forward_range,
            self.left_range,
        ]).T

        clustering = cluster.DBSCAN().fit(X)

        labels = clustering.labels_
        unique_labels = set(labels)
        if -1 in unique_labels:
            unique_labels.remove(-1)  # ignore the noise points
        core_samples_mask = np.zeros_like(labels, dtype=bool)
        core_samples_mask[clustering.core_sample_indices_] = True

        obstacles = {}
        for i in unique_labels:
            pts = X[(labels == i) & core_samples_mask]
            center = np.mean(pts, axis=0)
            radii = np.linalg.norm(pts - center, axis=1)
            ob = Obstacle(float(center[0]), float(center[1]), float(max(radii)))
            obstacles[i] = ob

        if plot:
            plot_clusters(X, clustering, obstacles, plot)

        return list(obstacles.values())


def detect_obstacles(msg, plot=None) -> List[Obstacle]:
    scan = Scan.from_ranges(msg['angle_min'], msg['angle_max'], msg['angle_increment'],
                            msg['time_increment'], msg['scan_time'],
                            msg['range_min'], msg['range_max'], msg['ranges'],
                            msg['intensities'])

    obstacles = scan.cluster(plot)
    return obstacles


def plot_clusters(X, clustering, obstacles: dict = None, save_path: Path = None):
    """Adapted from https://scikit-learn.org/stable/auto_examples/cluster/plot_dbscan.html#sphx-glr-auto-examples-cluster-plot-dbscan-py"""
    import matplotlib.pyplot as plt
    plt.close()

    labels = clustering.labels_

    unique_labels = set(labels)
    core_samples_mask = np.zeros_like(labels, dtype=bool)
    core_samples_mask[clustering.core_sample_indices_] = True

    colors = [plt.colormaps['Spectral'](each) for each in np.linspace(0, 1, len(unique_labels))]
    for k, col in zip(unique_labels, colors):
        if k == -1:
            # Black used for noise.
            col = [0, 0, 0, 1]

        class_member_mask = labels == k

        plt.plot(
            0, 0,
            "s",
            markerfacecolor="cyan",
            markeredgecolor="k",
            markersize=14,
        )

        xy = X[class_member_mask & core_samples_mask]
        plt.plot(
            xy[:, 0],
            xy[:, 1],
            "x",
            markerfacecolor=tuple(col),
            markeredgecolor="k",
            markersize=14,
        )

        xy = X[class_member_mask & ~core_samples_mask]
        plt.plot(
            xy[:, 0],
            xy[:, 1],
            "x",
            markerfacecolor=tuple(col),
            markeredgecolor="k",
            markersize=6,
        )

    n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
    n_noise_ = list(labels).count(-1)

    print("Estimated number of clusters: %d" % n_clusters_)
    print("Estimated number of noise points: %d" % n_noise_)

    plt.title(f"Laser scans and detected obstacles\nEstimated number of obstacles: {n_clusters_}")
    plt.xlabel('Forward axis (m)')
    plt.ylabel('Lateral axis (m)')

    if obstacles is not None:
        fig = plt.gcf()
        ax = fig.gca()

        for k, col in zip(unique_labels, colors):
            if k == -1:
                continue
            o = obstacles[k]

            circle = plt.Circle(
                (o.forward, o.left),
                o.radius,
                color=col,
            )
            ax.add_patch(circle)

    if save_path:
        plt.savefig(save_path)
    else:
        plt.show()


def process_file(scan_file: Path, output_file: Path, plot_folder: Path = None):
    import yaml

    scans = yaml.safe_load_all(scan_file.open('r'))

    obstacle_msgs = []
    for i, msg in enumerate(scans):
        if msg is None:
            continue
        if plot_folder:
            plot_filename = plot_folder / f'fig_{i}.png'
        else:
            plot_filename = None
        obs = detect_obstacles(msg, plot_filename)
        obstacle_msgs.append([ob.to_dict() for ob in obs])

    yaml.safe_dump_all(obstacle_msgs, output_file.open('w'))


if __name__ == '__main__':
    parser = ArgumentParser(description='Convert a file of laser scans into a file of collections of obstacles')
    parser.add_argument('scan_file', type=Path)
    parser.add_argument('output_file', type=Path)
    parser.add_argument('plot_folder', default=None, type=Path)
    args = parser.parse_args()

    process_file(args.scan_file, args.output_file, args.plot_folder)
