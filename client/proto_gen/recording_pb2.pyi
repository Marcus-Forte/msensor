import lidar_pb2 as _lidar_pb2
import imu_pb2 as _imu_pb2
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from collections.abc import Mapping as _Mapping
from typing import ClassVar as _ClassVar, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class RecordingEntry(_message.Message):
    __slots__ = ("scan", "imu")
    SCAN_FIELD_NUMBER: _ClassVar[int]
    IMU_FIELD_NUMBER: _ClassVar[int]
    scan: _lidar_pb2.PointCloud3
    imu: _imu_pb2.IMUData
    def __init__(self, scan: _Optional[_Union[_lidar_pb2.PointCloud3, _Mapping]] = ..., imu: _Optional[_Union[_imu_pb2.IMUData, _Mapping]] = ...) -> None: ...

class saveFileRequest(_message.Message):
    __slots__ = ("filename",)
    FILENAME_FIELD_NUMBER: _ClassVar[int]
    filename: str
    def __init__(self, filename: _Optional[str] = ...) -> None: ...
