"""Connection LED: whether the client is connected to the sensor server.

"Connected" is the gRPC channel state READY: an HTTP/2 connection to the server's address is up. The monitor has
its own channel (no RPC on it) with a short reconnect backoff, so the LED turns green within about 2 s of the
server coming back, and red as soon as the server process goes away (its socket closes). A server that stops
answering without closing the connection (cable pulled, host frozen) is not detected: that would need keepalive
pings, which the C++ server rejects by default when no call is active. Green also does not say which services the
server implements.

States, from the channel's connectivity:

* green, connected: READY.
* amber, connecting: IDLE or CONNECTING, before the first failure (start-up, or right after the server dropped).
* red, unreachable: TRANSIENT_FAILURE or SHUTDOWN; the retries in between (CONNECTING) stay red, so the LED does
  not flicker while the server is down.
"""

import html
import logging
import threading
from collections.abc import Callable
from enum import Enum

import grpc

logger = logging.getLogger(__name__)

RECONNECT_BACKOFF_MS = 1000
"""Retry the connection about every second while the server is unreachable (gRPC's default backs off to 120 s)."""

MONITOR_CHANNEL_OPTIONS = (
    ("grpc.initial_reconnect_backoff_ms", RECONNECT_BACKOFF_MS),
    ("grpc.min_reconnect_backoff_ms", RECONNECT_BACKOFF_MS),
    ("grpc.max_reconnect_backoff_ms", 2 * RECONNECT_BACKOFF_MS),
)

KICK_INTERVAL_SEC = 1.0


class ConnectionState(Enum):
    CONNECTING = ("connecting", "#f59f00")
    CONNECTED = ("connected", "#2fb344")
    DISCONNECTED = ("unreachable", "#e03131")

    @property
    def label(self) -> str:
        return self.value[0]

    @property
    def color(self) -> str:
        return self.value[1]


def next_state(current: ConnectionState, connectivity: grpc.ChannelConnectivity) -> ConnectionState:
    if connectivity is grpc.ChannelConnectivity.READY:
        return ConnectionState.CONNECTED
    if connectivity in (grpc.ChannelConnectivity.TRANSIENT_FAILURE, grpc.ChannelConnectivity.SHUTDOWN):
        return ConnectionState.DISCONNECTED
    # IDLE / CONNECTING: a retry while unreachable stays red.
    return ConnectionState.DISCONNECTED if current is ConnectionState.DISCONNECTED else ConnectionState.CONNECTING


def led_html(state: ConnectionState, address: str, name: str = "Sensor server") -> str:
    color = state.color
    return (
        '<div style="display:flex;align-items:center;gap:0.6em;padding:0.1em 0.75em;font-size:0.875em">'
        f'<span style="flex:none;width:0.85em;height:0.85em;border-radius:50%;background:{color};'
        f'box-shadow:0 0 0.4em {color}"></span>'
        f"<span>{html.escape(name)} <code>{html.escape(address)}</code>: {state.label}</span>"
        "</div>"
    )


class ConnectionMonitor:
    """Tracks the connection to `address` and calls `on_change(state)` on every change (from gRPC's or its own
    thread), and once at start."""

    def __init__(self, address: str, on_change: Callable[[ConnectionState], None]) -> None:
        self._address = address
        self._on_change = on_change
        self._lock = threading.Lock()
        self._state = ConnectionState.CONNECTING
        self._connectivity = grpc.ChannelConnectivity.IDLE
        self._channel = grpc.insecure_channel(address, options=MONITOR_CHANNEL_OPTIONS)
        self._thread: threading.Thread | None = None

    @property
    def state(self) -> ConnectionState:
        with self._lock:
            return self._state

    def start(self, should_stop: Callable[[], bool], wait: Callable[[float], bool]) -> None:
        """Start watching; `wait(seconds)` sleeps and returns True when the viewer shuts down."""
        self._on_change(self._state)
        self._channel.subscribe(self._on_connectivity, try_to_connect=True)
        self._thread = threading.Thread(target=self._kick_loop, args=(should_stop, wait), daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._channel.unsubscribe(self._on_connectivity)
        self._channel.close()

    def _on_connectivity(self, connectivity: grpc.ChannelConnectivity) -> None:
        with self._lock:
            self._connectivity = connectivity
            previous = self._state
            self._state = next_state(previous, connectivity)
            state = self._state
        if state is not previous:
            log = logger.info if state is ConnectionState.CONNECTED else logger.warning
            log("Sensor server %s: %s", self._address, state.label)
            self._on_change(state)

    def _kick_loop(self, should_stop: Callable[[], bool], wait: Callable[[float], bool]) -> None:
        """An IDLE channel (after the server dropped the connection) does not reconnect by itself: ask it to."""
        while not should_stop():
            with self._lock:
                idle = self._connectivity is grpc.ChannelConnectivity.IDLE
            if idle:
                future = grpc.channel_ready_future(self._channel)
                try:
                    future.result(timeout=KICK_INTERVAL_SEC)
                except grpc.FutureTimeoutError:
                    future.cancel()
            if wait(KICK_INTERVAL_SEC):
                return
