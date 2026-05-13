import header_pb2 as _header_pb2
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from collections.abc import Mapping as _Mapping
from typing import ClassVar as _ClassVar, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class AdcDataRequest(_message.Message):
    __slots__ = ("channel",)
    CHANNEL_FIELD_NUMBER: _ClassVar[int]
    channel: int
    def __init__(self, channel: _Optional[int] = ...) -> None: ...

class AdcData(_message.Message):
    __slots__ = ("header", "sample")
    HEADER_FIELD_NUMBER: _ClassVar[int]
    SAMPLE_FIELD_NUMBER: _ClassVar[int]
    header: _header_pb2.Header
    sample: float
    def __init__(self, header: _Optional[_Union[_header_pb2.Header, _Mapping]] = ..., sample: _Optional[float] = ...) -> None: ...
