from google.protobuf.internal import containers as _containers
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from collections.abc import Iterable as _Iterable, Mapping as _Mapping
from typing import ClassVar as _ClassVar, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class Point3(_message.Message):
    __slots__ = ("x", "y", "z", "r", "g", "b", "intensity")
    X_FIELD_NUMBER: _ClassVar[int]
    Y_FIELD_NUMBER: _ClassVar[int]
    Z_FIELD_NUMBER: _ClassVar[int]
    R_FIELD_NUMBER: _ClassVar[int]
    G_FIELD_NUMBER: _ClassVar[int]
    B_FIELD_NUMBER: _ClassVar[int]
    INTENSITY_FIELD_NUMBER: _ClassVar[int]
    x: float
    y: float
    z: float
    r: float
    g: float
    b: float
    intensity: int
    def __init__(self, x: _Optional[float] = ..., y: _Optional[float] = ..., z: _Optional[float] = ..., r: _Optional[float] = ..., g: _Optional[float] = ..., b: _Optional[float] = ..., intensity: _Optional[int] = ...) -> None: ...

class PointCloud3(_message.Message):
    __slots__ = ("points", "timestamp")
    POINTS_FIELD_NUMBER: _ClassVar[int]
    TIMESTAMP_FIELD_NUMBER: _ClassVar[int]
    points: _containers.RepeatedCompositeFieldContainer[Point3]
    timestamp: int
    def __init__(self, points: _Optional[_Iterable[_Union[Point3, _Mapping]]] = ..., timestamp: _Optional[int] = ...) -> None: ...

class LidarStreamRequest(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class SubSampledLidarStreamRequest(_message.Message):
    __slots__ = ("voxel_size",)
    VOXEL_SIZE_FIELD_NUMBER: _ClassVar[int]
    voxel_size: float
    def __init__(self, voxel_size: _Optional[float] = ...) -> None: ...
