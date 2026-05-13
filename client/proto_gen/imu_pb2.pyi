import header_pb2 as _header_pb2
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from collections.abc import Mapping as _Mapping
from typing import ClassVar as _ClassVar, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class IMUData(_message.Message):
    __slots__ = ("header", "ax", "ay", "az", "gx", "gy", "gz")
    HEADER_FIELD_NUMBER: _ClassVar[int]
    AX_FIELD_NUMBER: _ClassVar[int]
    AY_FIELD_NUMBER: _ClassVar[int]
    AZ_FIELD_NUMBER: _ClassVar[int]
    GX_FIELD_NUMBER: _ClassVar[int]
    GY_FIELD_NUMBER: _ClassVar[int]
    GZ_FIELD_NUMBER: _ClassVar[int]
    header: _header_pb2.Header
    ax: float
    ay: float
    az: float
    gx: float
    gy: float
    gz: float
    def __init__(self, header: _Optional[_Union[_header_pb2.Header, _Mapping]] = ..., ax: _Optional[float] = ..., ay: _Optional[float] = ..., az: _Optional[float] = ..., gx: _Optional[float] = ..., gy: _Optional[float] = ..., gz: _Optional[float] = ...) -> None: ...

class ImuStreamRequest(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...
