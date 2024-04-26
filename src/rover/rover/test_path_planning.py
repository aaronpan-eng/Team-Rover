import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline

# Define the start, midpoint, and end points
start_easting, start_northing = 327433.15, 4686130.26
mid_easting, mid_northing = 327389.57, 4686096.43  # Desired midpoint
end_easting, end_northing = 327346.80, 4685992.94

# Define the number of points to generate
num_points = 500

# Generate random noise for the path
noise_scale = 0.5  # Adjust this parameter to control the amount of noise
easting_noise = np.random.normal(0, noise_scale, num_points)
northing_noise = np.random.normal(0, noise_scale, num_points)

# Interpolate between start, midpoint, and end points and add noise
easting_values = np.linspace(
    start_easting, mid_easting, num_points // 2) + easting_noise[:num_points // 2]
easting_values = np.concatenate((easting_values, np.linspace(
    mid_easting, end_easting, num_points // 2) + easting_noise[num_points // 2:]))
northing_values = np.linspace(
    start_northing, mid_northing, num_points // 2) + northing_noise[:num_points // 2]
northing_values = np.concatenate((northing_values, np.linspace(
    mid_northing, end_northing, num_points // 2) + northing_noise[num_points // 2:]))

# Apply a low-pass filter to smooth out the path


def low_pass_filter(data, alpha=0.1):
    filtered_data = [data[0]]
    for i in range(1, len(data)):
        filtered_data.append(
            alpha * data[i] + (1 - alpha) * filtered_data[i - 1])
    return np.array(filtered_data)


easting_values_filtered = low_pass_filter(easting_values)
northing_values_filtered = low_pass_filter(northing_values)

# Apply polynomial interpolation to create higher-order curves
t = np.linspace(0, 1, num_points)
cubic_spline_easting = CubicSpline(t, easting_values_filtered)
cubic_spline_northing = CubicSpline(t, northing_values_filtered)

# Generate new interpolated points
t_new = np.linspace(0, 1, 5 * num_points)
easting_values_smoothed = cubic_spline_easting(t_new)
northing_values_smoothed = cubic_spline_northing(t_new)

# Calculate tangent angle at each point
dy = np.gradient(northing_values_smoothed)
dx = np.gradient(easting_values_smoothed)
yaw_values = np.arctan2(dy, dx)

# Apply a low-pass filter to smooth out the yaw values
yaw_values_smoothed = low_pass_filter(yaw_values, alpha=0.9)

# Unwrap the yaw values from -π to π
yaw_values_unwrapped = np.unwrap(yaw_values_smoothed)

# Plot the path and smoothed yaw values
fig, axs = plt.subplots(2, 1, figsize=(10, 8))

# Plot the path
axs[0].plot(easting_values_smoothed, northing_values_smoothed, label='Path')
axs[0].scatter([start_easting, mid_easting, end_easting],
               [start_northing, mid_northing, end_northing],
               color='red', label='Start/Midpoint/End')
axs[0].set_xlabel('UTM Easting')
axs[0].set_ylabel('UTM Northing')
axs[0].set_title('Path with Midpoint between Start and End Points (Smoothed)')
axs[0].legend()
axs[0].grid(True)

# Plot the smoothed and unwrapped yaw values
axs[1].plot(yaw_values_unwrapped, label='Yaw (Smoothed and Unwrapped)')
axs[1].set_xlabel('Index')
axs[1].set_ylabel('Yaw (radians)')
axs[1].set_title('Smoothed and Unwrapped Yaw Values along the Path')
axs[1].legend()
axs[1].grid(True)

plt.tight_layout()
plt.show()
