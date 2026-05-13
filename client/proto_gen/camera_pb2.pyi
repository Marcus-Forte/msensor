import header_pb2 as _header_pb2
from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from collections.abc import Mapping as _Mapping
from typing import ClassVar as _ClassVar, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class CameraEncoding(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    UNKNOWN: _ClassVar[CameraEncoding]
    RGB8: _ClassVar[CameraEncoding]
    BGR8: _ClassVar[CameraEncoding]
    GRAY8: _ClassVar[CameraEncoding]
    MJPEG: _ClassVar[CameraEncoding]
UNKNOWN: CameraEncoding
RGB8: CameraEncoding
BGR8: CameraEncoding
GRAY8: CameraEncoding
MJPEG: CameraEncoding

class CameraStreamRequest(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class CameraStreamReply(_message.Message):
    __slots__ = ("header", "width", "height", "encoding", "image_data")
    HEADER_FIELD_NUMBER: _ClassVar[int]
    WIDTH_FIELD_NUMBER: _ClassVar[int]
    HEIGHT_FIELD_NUMBER: _ClassVar[int]
    ENCODING_FIELD_NUMBER: _ClassVar[int]
    IMAGE_DATA_FIELD_NUMBER: _ClassVar[int]
    header: _header_pb2.Header
    width: int
    height: int
    encoding: CameraEncoding
    image_data: bytes
    def __init__(self, header: _Optional[_Union[_header_pb2.Header, _Mapping]] = ..., width: _Optional[int] = ..., height: _Optional[int] = ..., encoding: _Optional[_Union[CameraEncoding, str]] = ..., image_data: _Optional[bytes] = ...) -> None: ...
