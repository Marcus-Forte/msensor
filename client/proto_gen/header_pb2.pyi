from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Optional as _Optional

DESCRIPTOR: _descriptor.FileDescriptor

class Header(_message.Message):
    __slots__ = ("timestamp", "sequence_number")
    TIMESTAMP_FIELD_NUMBER: _ClassVar[int]
    SEQUENCE_NUMBER_FIELD_NUMBER: _ClassVar[int]
    timestamp: int
    sequence_number: int
    def __init__(self, timestamp: _Optional[int] = ..., sequence_number: _Optional[int] = ...) -> None: ...
