from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Optional as _Optional

DESCRIPTOR: _descriptor.FileDescriptor

class AdcDataRequest(_message.Message):
    __slots__ = ("channel",)
    CHANNEL_FIELD_NUMBER: _ClassVar[int]
    channel: int
    def __init__(self, channel: _Optional[int] = ...) -> None: ...

class AdcData(_message.Message):
    __slots__ = ("sample", "timestamp")
    SAMPLE_FIELD_NUMBER: _ClassVar[int]
    TIMESTAMP_FIELD_NUMBER: _ClassVar[int]
    sample: float
    timestamp: int
    def __init__(self, sample: _Optional[float] = ..., timestamp: _Optional[int] = ...) -> None: ...
