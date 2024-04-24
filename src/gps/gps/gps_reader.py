from dataclasses import dataclass
from datetime import datetime
from typing import Optional, TextIO, BinaryIO, Union, List, Type
import time

import utm

def UTCtoUTCEpoch(UTC):
    UTC = float(UTC)
    UTCinSecs = (int(UTC/10000)*3600)+(int((UTC%10000)/100)*60)+(UTC%100) #Replace with a line that converts the UTC float in hhmmss.ss to seconds as a float
    TimeSinceEpoch = time.mktime(time.localtime()) #Replace with a 1-line method to get time since epoch
    TimeSinceEpochBOD = TimeSinceEpoch - (TimeSinceEpoch % 86400) #Use the time since epoch to get the time since epoch *at the beginning of the day*
    CurrentTime = TimeSinceEpochBOD + UTCinSecs
    CurrentTimeSec = int(CurrentTime) #Replace with a 1-line calculation to get total seconds as an integer
    CurrentTimeNsec = int((CurrentTime-CurrentTimeSec)*1e9) #Replace with a 1-line calculation to get remaining nanoseconds as an integer (between CurrentTime and CurrentTimeSec )
    print(CurrentTime)
    return [CurrentTimeSec, CurrentTimeNsec]

@dataclass
class UTM:
    easting: float
    northing: float
    zone: int
    letter: str


@dataclass
class GpsMsg:
    raw: str
    timestamp: tuple
    latitude: float
    longitude: float
    altitude: float
    undulation: float
    quality: int
    n_sats: int
    hdop: float
    age: Optional[int]
    stn_id: str
    header: str = '$GPGGA'

    @classmethod
    def from_str(cls, raw: str) -> "GpsMsg":
        """
Referenced while writing
* https://docs.novatel.com/OEM7/Content/Logs/GPGGA.htm
* https://en.wikipedia.org/wiki/Decimal_degrees
        """
        (header,
         time,
         lat, lat_dir,
         lon, lon_dir,
         quality,
         n_sats,
         hdop,
         alt,
         a_units,
         undulation,
         u_units,
         age,
         stn_id_and_checksum) = raw.strip().split(',')

        if header != cls.header:
            raise ValueError(f'not a {cls.header} message, header was {header}')

        # raise NotImplementedError('need to fix gps time conversion')
        sec, nsec = UTCtoUTCEpoch(time)
        timestamp = (sec, nsec)

        try:
            deg = float(lat[:2])
            mins = float(lat[2:])
        except ValueError:
            raise ValueError(f'invalid lat {lat}')
        latitude = deg + mins / 60
        if lat_dir == 'N':
            pass
        elif lat_dir == 'S':
            latitude *= -1
        else:
            raise ValueError(f'invalid lat_dir {lat_dir}')

        try:
            deg = float(lon[:3])
            mins = float(lon[3:])
        except ValueError:
            raise ValueError(f'invalid lon {lon}')
        longitude = deg + mins / 60
        if lon_dir == 'E':
            pass
        elif lon_dir == 'W':
            longitude *= -1
        else:
            raise ValueError(f'invalid lon_dir {lon_dir}')

        try:
            altitude = float(alt)
        except ValueError:
            raise ValueError(f'invalid altitude {alt}')
        if a_units != 'M':
            raise ValueError(f'invalid a_units {a_units}')

        try:
            undulation = float(undulation)
        except ValueError:
            raise ValueError(f'invalid undulation {undulation}')
        if u_units != 'M':
            raise ValueError(f'invalid u_units {u_units}')

        try:
            quality = int(quality)
        except ValueError:
            raise ValueError(f'invalid quality {quality}')

        try:
            hdop = float(hdop)
        except ValueError:
            raise ValueError(f'invalid hdop {hdop}')

        if age == '':
            age = None
        else:
            try:
                age = float(age)
            except ValueError:
                raise ValueError(f'invalid age {age}')

        try:
            n_sats = int(n_sats)
        except ValueError:
            raise ValueError(f'invalid n_sats {n_sats}')

        (stn_id, checksum) = stn_id_and_checksum.split('*')
        stn_id = stn_id
        try:
            checksum = int(checksum)
        except ValueError:
            raise ValueError(f'invalid checksum {checksum}')

        return cls(
            raw=raw,
            timestamp=timestamp,
            latitude=latitude,
            longitude=longitude,
            altitude=altitude,
            undulation=undulation,
            quality=quality,
            hdop=hdop,
            age=age,
            n_sats=n_sats,
            stn_id=stn_id,
        )

    def to_utm(self) -> UTM:
        return UTM(
            *utm.from_latlon(self.latitude, self.longitude)
        )


@dataclass
class GpsRtkMsg(GpsMsg):
    header = '$GNGGA'

    """
    Referenced
    * https://openrtk.readthedocs.io/en/latest/communication_port/nmea.html
    
    The message format seems to be the same as GPGGA but with a different header and more precision in lat and lon.
    """


class GpsReader:
    stream: Union[TextIO, BinaryIO]
    supported_msg_types: List[Type[GpsMsg]]

    def __init__(self, stream: Union[TextIO, BinaryIO], use_basic=True, use_rtk=False):
        self.stream = stream
        self.supported_msg_types = []
        if use_basic:
            self.supported_msg_types.append(GpsMsg)
        if use_rtk:
            self.supported_msg_types.append(GpsRtkMsg)

    def __next__(self) -> GpsMsg:
        while b := self.stream.readline():
            if isinstance(b, bytes):
                line = b.decode('ascii')
            else:
                line = b

            for MsgType in self.supported_msg_types:
                try:
                    return MsgType.from_str(line)
                except ValueError:
                    pass
        raise StopIteration

    def __bool__(self):
        return self.stream.readable()

    def __iter__(self):
        return self

    def close(self):
        self.stream.close()


# Just a quick example/test
if __name__ == '__main__':
    import sys
    path = sys.argv[1]

    f = open(path)
    for msg in GpsReader(f):
        print(msg, msg.to_utm())
