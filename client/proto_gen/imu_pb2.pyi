from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Optional as _Optional

DESCRIPTOR: _descriptor.FileDescriptor

class IMUData(_message.Message):
    __slots__ = ("ax", "ay", "az", "gx", "gy", "gz", "timestamp")
    AX_FIELD_NUMBER: _ClassVar[int]
    AY_FIELD_NUMBER: _ClassVar[int]
    AZ_FIELD_NUMBER: _ClassVar[int]
    GX_FIELD_NUMBER: _ClassVar[int]
    GY_FIELD_NUMBER: _ClassVar[int]
    GZ_FIELD_NUMBER: _ClassVar[int]
    TIMESTAMP_FIELD_NUMBER: _ClassVar[int]
    ax: float
    ay: float
    az: float
    gx: float
    gy: float
    gz: float
    timestamp: int
    def __init__(self, ax: _Optional[float] = ..., ay: _Optional[float] = ..., az: _Optional[float] = ..., gx: _Optional[float] = ..., gy: _Optional[float] = ..., gz: _Optional[float] = ..., timestamp: _Optional[int] = ...) -> None: ...

class ImuStreamRequest(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...
