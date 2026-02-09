from google.protobuf import empty_pb2 as _empty_pb2
from google.protobuf.internal import containers as _containers
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Iterable as _Iterable, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class AddAxisRequest(_message.Message):
    __slots__ = ("axis_id", "number_of_samples", "plot_title", "x_axis_title", "y_axis_title")
    AXIS_ID_FIELD_NUMBER: _ClassVar[int]
    NUMBER_OF_SAMPLES_FIELD_NUMBER: _ClassVar[int]
    PLOT_TITLE_FIELD_NUMBER: _ClassVar[int]
    X_AXIS_TITLE_FIELD_NUMBER: _ClassVar[int]
    Y_AXIS_TITLE_FIELD_NUMBER: _ClassVar[int]
    axis_id: int
    number_of_samples: int
    plot_title: str
    x_axis_title: str
    y_axis_title: str
    def __init__(self, axis_id: _Optional[int] = ..., number_of_samples: _Optional[int] = ..., plot_title: _Optional[str] = ..., x_axis_title: _Optional[str] = ..., y_axis_title: _Optional[str] = ...) -> None: ...

class RemoveAxisRequest(_message.Message):
    __slots__ = ("axis_id",)
    AXIS_ID_FIELD_NUMBER: _ClassVar[int]
    axis_id: int
    def __init__(self, axis_id: _Optional[int] = ...) -> None: ...

class AddSignalRequest(_message.Message):
    __slots__ = ("axis_id", "signal_id", "signal_name", "signal_color")
    AXIS_ID_FIELD_NUMBER: _ClassVar[int]
    SIGNAL_ID_FIELD_NUMBER: _ClassVar[int]
    SIGNAL_NAME_FIELD_NUMBER: _ClassVar[int]
    SIGNAL_COLOR_FIELD_NUMBER: _ClassVar[int]
    axis_id: int
    signal_id: int
    signal_name: str
    signal_color: str
    def __init__(self, axis_id: _Optional[int] = ..., signal_id: _Optional[int] = ..., signal_name: _Optional[str] = ..., signal_color: _Optional[str] = ...) -> None: ...

class RemoveSignalRequest(_message.Message):
    __slots__ = ("signal_id",)
    SIGNAL_ID_FIELD_NUMBER: _ClassVar[int]
    signal_id: int
    def __init__(self, signal_id: _Optional[int] = ...) -> None: ...

class streamPoint(_message.Message):
    __slots__ = ("signal_id", "value")
    SIGNAL_ID_FIELD_NUMBER: _ClassVar[int]
    VALUE_FIELD_NUMBER: _ClassVar[int]
    signal_id: int
    value: float
    def __init__(self, signal_id: _Optional[int] = ..., value: _Optional[float] = ...) -> None: ...

class streamPointRequest(_message.Message):
    __slots__ = ("points",)
    POINTS_FIELD_NUMBER: _ClassVar[int]
    points: _containers.RepeatedCompositeFieldContainer[streamPoint]
    def __init__(self, points: _Optional[_Iterable[_Union[streamPoint, _Mapping]]] = ...) -> None: ...
