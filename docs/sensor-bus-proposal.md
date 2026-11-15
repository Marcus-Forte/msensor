# Change proposal: in-process sensor bus

Status: proposed 2026-11-15. Base commit: `8e426cf` (master).

## Summary

Put a small in-process publish/subscribe layer (the "sensor bus") between the
drivers and their consumers. Today each driver has exactly one consumer, the
gRPC stream of the one connected client, and that consumer's work runs on the
driver's thread. With the bus, a driver hands each sample to a topic, and any
number of subscribers (gRPC clients, a recorder on the sensor host) each read
it from their own bounded queue.

This covers three items of `todo`: "Sensor Bus", "Support more subscribers at
once" and "separate thread for sensor recording?".

## Problems in the current design

1. One client per stream. `makeStreamReactor` (`grpc/sensor_stream.hh`) answers
   a second client with `RESOURCE_EXHAUSTED`. Recording while a SLAM or a viewer
   is connected is impossible.
2. Protobuf conversion runs on the driver's thread. `SensorStreamReactor::onSample`
   converts the sample and starts the write when no write is in flight. For the
   Mid360 this is the Livox SDK callback thread, and `CallbackSlot::emit` holds
   its lock for the whole callback.
3. A complete recording depends on the network. `remote_recorder` is the only
   recorder, and it sits behind a bounded server queue and a network link.
4. Loss is invisible while it happens. The server logs its drop total once,
   when the stream ends, on the sensor host. `remote_recorder` reports only how
   many entries it saved.

## Goals

- Several clients can read the same sensor stream at once.
- A driver's thread only hands a sample over. No serialization, network or disk
  work happens on it.
- A recording can be made on the sensor host, with no network in the path.
- Loss is reported when it happens, on the side that can act on it.

## Not in scope

- Any change to the `.proto` files, the wire format or the `.pbscan` format.
  This excludes two ideas from the design discussion: a drop counter carried in
  the stream messages, and batching IMU samples into one message. Both need a
  wire change and the second needs a profile first.
- Any change to the driver classes or to the driver interfaces.
- The Python client.

## Frozen interface (what mslam uses)

mslam builds msensor as a submodule (`add_subdirectory(third_party/msensor
EXCLUDE_FROM_ALL)`, compiled with `MSENSOR_NO_LEGACY_NAMES`). None of the
following may change in name, signature, semantics or location:

| Kind | Items |
|---|---|
| Headers | `msensor/interface/IImu.hh`, `msensor/interface/ILidar.hh`, `msensor/config/config.hh`, `msensor/lidar/mid360.hh`, `msensor/recorder/recording_driver.hh`, `msensor/recorder/scan_player.hh`, `msensor/conversions/lidar_imu_conversions.hh`, `sensors_remote_client.hh` |
| Types and functions | `IMUData`, `Scan3DI`, `Header`, `ILidar`, `IImu` (including the single-consumer contract of `setScanCallback` / `setImuCallback`), `Config::fromFile`, `Config::defaultConfigPath` and the existing `Config` members, `Mid360` (constructor, `init`, `setMode`, `setScanPattern`), `SensorsRemoteClient` and `RemoteClientOptions{scan_queue_size, imu_queue_size}`, `RecordingSensorDriver`, `ScanPlayer`, `fromProtobuf` |
| Protobuf | every existing message and service in `proto/` (mslam's `slam.proto` imports them and `bench_slam` reads `sensors::RecordingEntry`) |
| CMake | targets `msensor::msensor`, `msensor::server`, `msensor`, `mid360`, `config`, `scan_recorder`, `sensors_proto`, `playback_publisher`, and the option `MSENSOR_INSTALL_PLAYBACK_PUBLISHER` |
| Wire | `getLidarScan` / `getImuData` with `max_queued_samples`, including the defaults and caps in `sensor_stream.hh` |

Additions are allowed: new headers, new targets, new members with defaults,
new optional config keys, new methods on existing classes. A client built
against the old headers must compile and behave as before.

Consequence: the drivers keep their single `setXCallback` consumer. The bus is
that consumer and fans out behind it.

## Design

```
driver (Mid360, ICM20948, RPLidar, camera, playback, sim)
   │  setXCallback: the topic is the driver's single consumer
   ▼
Topic<T>::publish(T)          wraps the sample in shared_ptr<const T>,
   │                          pushes it to every subscription, returns
   ├── Subscription A  ── gRPC stream, client 1 (depth from the request)
   ├── Subscription B  ── gRPC stream, client 2
   └── Subscription C  ── BusRecorder (deep queue, own writer thread, disk)
```

### `Topic<T>` and `Subscription<T>` (new, `include/msensor/bus/`)

- `publish(T sample)` runs on the driver's thread. It wraps the sample in a
  `std::shared_ptr<const T>` once, pushes the pointer into each subscription's
  queue and wakes the consumers. It does nothing else: no conversion, no I/O,
  no call into subscriber code that can block.
- Each subscription has its own bounded FIFO with the existing policy: when it
  is full the oldest entry is dropped and counted. Capacity is fixed at
  subscribe time and is at least 1. One subscriber's backlog never affects
  another's.
- A subscription exposes: a non-blocking pop, a stop-aware blocking pop, the
  running drop total, and a wake-up hook for consumers that do not own a thread.
- Wake-up hooks do not run on the publisher's thread. Recommended mechanism:
  one dispatcher thread per topic, woken by `publish`, that calls the hooks.
  The number of threads then depends on the number of topics, not of clients.
  Another mechanism is acceptable if the invariants below hold.
- Destroying a subscription detaches it. When the destructor returns, its hook
  is not running and will not run again (the guarantee `CallbackSlot` gives).
- Lazy attachment: the topic installs itself as the driver's callback when its
  first subscriber arrives and clears it when the last one leaves. Drivers
  behave differently with and without a consumer (the Mid360 discards packets
  and restarts scan accumulation), and that must be preserved.

### `SensorBus` (new)

Built from a `Sensors` struct. Owns one topic per available streaming sensor
and keeps the driver pointers alive. `SensorsServer` and the recorder share one
bus through a `std::shared_ptr<SensorBus>`.

### gRPC streams

- `SensorStreamReactor` becomes a bus subscriber. Its queue is the
  subscription's queue, sized by `resolveQueueSize()` exactly as today.
- The single-client check (`in_use`) goes away. `ServerOptions` gets
  `max_clients_per_stream` (default 8); a client beyond it gets
  `RESOURCE_EXHAUSTED`.
- Conversion to protobuf and `StartWrite` happen on the dispatcher thread or on
  a gRPC thread (`OnWriteDone`), never on the driver's thread. Conversion stays
  lazy: a dropped sample is never converted.
- The gRPC callback rules stay as they are: one write in flight, no write after
  cancel or a failed write, `Finish` once.
- Drops are logged when they happen, at most once per second per stream and
  client, and the total is still logged at stream end.
- `SensorsServer(Sensors, ServerOptions)` and the legacy positional constructor
  keep working and build a private bus. A new constructor,
  `SensorsServer(ServerOptions, std::shared_ptr<SensorBus>)`, takes a shared
  bus. Options come first so that the existing call
  `SensorsServer({}, {.listen_address = ...})` stays unambiguous.

### Recording on the sensor host

- `BusRecorder` (new, `src/recorder/`): subscribes to the LiDAR and IMU topics
  with deep queues and writes through the existing `ScanRecorder` on its own
  thread, so the file format is unchanged and `ScanPlayer` reads the result.
- At each wake-up the writer drains pending IMU samples before pending scans,
  so a scan's IMU samples precede it in the file as they do in arrival order.
- Queue depths are not limited by the gRPC caps. Defaults: 256 scans and 8192
  IMU samples; both configurable. A drop is logged as a warning at once (rate
  limited) because a recorder that loses data should say so.
- On stop it drains what is queued, closes the file and logs the entries
  written and dropped per stream.
- `sensor_publisher` starts it from an optional `recording` object in the
  config (`enable`, `file`, `scan_queue`, `imu_queue`; all optional, disabled
  by default) or from `--record <file>` on the command line. The positional
  config argument keeps working.
- `remote_recorder` stays.

### Loss visibility on the client

- `SensorsRemoteClient` counts gaps in `header.sequence_number` per stream. A
  forward jump of `n > 1` counts `n - 1` lost samples (modulo 2^32). A backward
  jump is a restart (playback rewind, server restart), not a loss, and the
  first number of a stream carries no meaning.
- New const getters return the totals. A gap is logged as a warning, at most
  once per second per stream.
- `remote_recorder` prints the lost counts next to "Saved entries" on exit.

## Invariants (acceptance criteria)

1. Nothing in the frozen interface changes. mslam builds without a source
   change and `bench_slam` prints the same final pose as before.
2. `Topic::publish` never converts, never does I/O and never waits on a
   subscriber. A test asserts that conversion does not run on the publishing
   thread.
3. Two clients on the same stream both receive every sample when their queues
   are deep enough, in order.
4. A slow or stalled subscriber loses only its own samples; another subscriber
   on the same topic loses none.
5. With no subscriber, the driver has no callback installed.
6. A subscription can be destroyed while samples are being published, with no
   use after free. The suite passes under the `tsan` and `asan` presets.
7. A local recording of a known sample sequence is read back complete and in
   order by `ScanPlayer`.
8. Every test that exists today still passes, except tests of the removed
   single-client refusal, which are replaced by tests of the client limit.

## Work packages

Sequential, one engineer at a time (they share the working tree and the build
directory).

1. Bus and streams: `Topic`, `Subscription`, `SensorBus`, the reactor and the
   four services, `SensorsServer`, the drop logging, their tests, and the docs
   that describe streams (`README.md`, `AGENTS.md`, `docs/arch.puml`,
   `docs/sampling-threads.md`).
2. Recording and loss visibility: `BusRecorder`, the config keys,
   `sensor_publisher`, the gap counters in `SensorsRemoteClient`,
   `remote_recorder`, their tests, the docs for both, and `todo`.

## Verification

- msensor: `cmake --preset gcc && cmake --build --preset build-gcc &&
  ctest --preset test-gcc`, then the `asan` and `tsan` presets.
- mslam: build `bench_slam` against the changed msensor and run it on
  `test/data/living_room_garden.pbscan`. The final pose must equal the
  reference in mslam's `AGENTS.md`
  (`0.043671476 -0.000098382 0.001114079 -0.003311060 0.002207124 0.487497048`).

## Risks

- Shutdown order. A reactor, the dispatcher and the driver can all be stopping
  at once. Invariant 6 and the sanitizer runs cover this.
- One allocation per sample for the shared pointer. At 200 Hz of IMU and 21 Hz
  of scans this is negligible; `bench_msensor`'s gRPC rows can confirm.
- Memory: 8 clients at the LiDAR cap of 64 scans share the same scan objects,
  so the worst case is bounded by the deepest queue, not by the client count.
- No hardware is available in the dev container. The Mid360 path is verified
  only through the simulated and playback drivers.
