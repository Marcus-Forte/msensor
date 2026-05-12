from google.protobuf.internal import containers as _containers
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from collections.abc import Iterable as _Iterable
from typing import ClassVar as _ClassVar, Optional as _Optional

DESCRIPTOR: _descriptor.FileDescriptor

class PointCloud3(_message.Message):
    __slots__ = ("timestamp", "x", "y", "z", "intensity", "r", "g", "b")
    TIMESTAMP_FIELD_NUMBER: _ClassVar[int]
    X_FIELD_NUMBER: _ClassVar[int]
    Y_FIELD_NUMBER: _ClassVar[int]
    Z_FIELD_NUMBER: _ClassVar[int]
    INTENSITY_FIELD_NUMBER: _ClassVar[int]
    R_FIELD_NUMBER: _ClassVar[int]
    G_FIELD_NUMBER: _ClassVar[int]
    B_FIELD_NUMBER: _ClassVar[int]
    timestamp: int
    x: _containers.RepeatedScalarFieldContainer[float]
    y: _containers.RepeatedScalarFieldContainer[float]
    z: _containers.RepeatedScalarFieldContainer[float]
    intensity: _containers.RepeatedScalarFieldContainer[int]
    r: _containers.RepeatedScalarFieldContainer[float]
    g: _containers.RepeatedScalarFieldContainer[float]
    b: _containers.RepeatedScalarFieldContainer[float]
    def __init__(self, timestamp: _Optional[int] = ..., x: _Optional[_Iterable[float]] = ..., y: _Optional[_Iterable[float]] = ..., z: _Optional[_Iterable[float]] = ..., intensity: _Optional[_Iterable[int]] = ..., r: _Optional[_Iterable[float]] = ..., g: _Optional[_Iterable[float]] = ..., b: _Optional[_Iterable[float]] = ...) -> None: ...

class LidarStreamRequest(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class SubSampledLidarStreamRequest(_message.Message):
    __slots__ = ("voxel_size",)
    VOXEL_SIZE_FIELD_NUMBER: _ClassVar[int]
    voxel_size: float
    def __init__(self, voxel_size: _Optional[float] = ...) -> None: ...
