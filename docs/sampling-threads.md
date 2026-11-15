# Sensor sampling threads

Sensor acquisition is delivered to services through callbacks. The thread
performing acquisition depends on the driver:

| Driver | Acquisition thread |
| --- | --- |
| `RPLidar` | Own `SamplingThread` (`std::jthread`), started by `startSampling()`; repeatedly reads scans and emits its callback. |
| `ICM20948` | Own `SamplingThread` (`std::jthread`), started by `startSampling()`; repeatedly reads samples and emits its callback. |
| `OpenCvCamera` | Own `SamplingThread` (`std::jthread`), started by `startSampling()`; repeatedly captures frames and emits its callback. |
| `SimLidar`, `SimImu`, `SimCamera` | Each simulator starts its own `SamplingThread` (`std::jthread`) in `startSampling()` and emits generated data from it. |
| `Mid360` | Starts the Livox SDK with `LivoxLidarSdkStart()` and receives data through SDK callbacks; msensor does not create a sampling `std::jthread` for it. |
| `ADS1115` | Pull-based: `readSingleEnded()` performs a read when called; the driver does not start a background sampling thread. |
| `RecordingSensorDriver` | Own `std::jthread`, started by `startSampling()`; parses the recording, paces entries by their timestamps and emits them. |
| `BusRecorder` | One `std::jthread` (the writer), started by `start()`: it sleeps until a topic's wake-up hook (on the topic's dispatcher thread) notifies it, then writes the queued IMU samples and scans through `ScanRecorder`. `stop()` joins it after a last drain. |
| `SensorsRemoteClient` | Two `std::jthread`s started by `start()`, one per gRPC stream (LiDAR, IMU); each converts received messages, counts gaps in their sequence numbers and emits them. Queue depths to request come from `RemoteClientOptions`. |

`RPLidar`, `ICM20948`, `OpenCvCamera`, `SimLidar`, `SimImu` and `SimCamera`
do not manage their thread themselves: each owns a `msensor::SamplingThread`
(`include/msensor/interface/SamplingThread.hh`), declared as its last data
member so it is joined before the members its loop uses are destroyed. It has
two modes:

- Periodic (`startPeriodic(period, tick)`; the simulators and `ICM20948`): `tick`
  runs on a fixed cadence, tick n due at `start + n * period` (the first one
  immediately), so the work time does not stretch the period. If a tick overruns
  its slot, the missed slots are skipped and the cadence continues on the
  original grid; it does not fire a burst to catch up. Periods: `SimLidar`
  25 ms (40 Hz), `SimImu` 20 ms (50 Hz), `SimCamera` 33 ms, `ICM20948` 10 ms.
- Free-running (`startFreeRunning(step)`; `RPLidar`, `OpenCvCamera`): the work is
  itself a blocking read, so `step` is called back to back and returns how long
  to back off before the next call (zero for none). `RPLidar` backs off 10 ms
  when no scan was grabbed, `OpenCvCamera` 100 ms when capture or grab fails.

All waits are stop-aware, so `stopSampling()` returns promptly even with a long
period or back-off. Starting while running and stopping while not running are
no-ops, and a stopped sampler can be started again. Start and stop are meant to
be called from one controlling thread.

For the drivers that own a `std::jthread`, `stopSampling()` requests stop and
joins the thread. This keeps their continuous polling or capture loops off the
thread that starts the service. Other driver operations, such as initialization
or configuration, are not automatically moved to a worker thread and may run
synchronously on their caller.

The sensor interfaces allow one callback consumer per stream. `CallbackSlot`
invokes that callback directly on the producer's thread and moves the sample
into it. Clearing the callback (`setXCallback({})`) waits for an invocation
that is still running, so the consumer may be destroyed once it returns. That
one consumer is the sensor's bus topic (`msensor::Topic<T>`, see
`include/msensor/bus/topic.hh`), which fans the sample out to any number of
subscribers:

- The driver's thread runs `Topic::publish()`: it wraps the sample in a
  `shared_ptr<const T>`, pushes it into each subscription's bounded FIFO and
  signals the topic's dispatcher thread. No protobuf conversion, I/O or wait
  on a subscriber happens there. The only locks it takes are the topic's
  subscriber list and each FIFO's, all held for a few instructions.
- Each topic has one dispatcher thread, started with its first subscriber and
  joined when the last one leaves, so the thread count follows the number of
  topics in use, not the number of clients. It calls the subscriptions'
  wake-up hooks one after the other.
- The gRPC `SensorStreamReactor` is a subscriber; it does not dedicate a
  blocked thread to each client. Its hook (dispatcher thread) and
  `OnWriteDone` (a gRPC thread) take the oldest queued sample, convert it to
  protobuf and call `StartWrite`, serialised by the reactor's mutex, with at
  most one write in flight.
- The topic is the driver's callback only while it has subscribers, so a driver
  with no client keeps its no-consumer behaviour (below). Attaching and
  detaching call `setXCallback` on the thread of the first subscribe / last
  unsubscribe, never while holding the lock `publish()` needs.
- Destroying a subscription waits for a hook that is running, so after it
  returns the hook never runs again; a hook therefore must not destroy a
  subscription of its own topic and should return quickly (a stuck hook delays
  the others of that topic, but never the driver's thread).

Each client's FIFO is its own: its depth is chosen with `max_queued_samples`
in the stream request, and when it is full the oldest queued sample is dropped,
so slow clients miss samples rather than block acquisition or slow another
client. The samples themselves are shared between the FIFOs. Queued samples are
converted to protobuf only when they are written, oldest first, so a dropped
sample is never converted. Once the client cancels or a write fails, its queue
is discarded and the reactor starts no further writes. Up to
`ServerOptions::max_clients_per_stream` clients (default 8) are served per
stream.

| Stream | Default (request 0) | Cap |
| --- | --- | --- |
| IMU | 200 (about 1 s at 200 Hz) | 4000 |
| LiDAR | 1 (latest wins) | 64 |
| Camera | 1 (latest wins) | 8 |

A larger request is lowered to the cap and logged once at info level. Depth 1 is
the old latest-wins behaviour, which suits scans and frames; IMU consumers that
integrate every sample ask for a deeper queue. Drops are counted per client and
logged as a warning while they happen, at most once per second per stream and
client, and their total is logged when the stream ends. A client detects a loss as a jump in
`header.sequence_number` between consecutive messages of one stream. The first
number received means nothing: some drivers, such as the Mid360 IMU, keep
counting while no client is connected. The constants live in
`grpc/sensor_stream.hh`.

`Mid360::stopSampling()` uninitialises the Livox SDK, which stops its
callbacks; sampling again requires `init()` first. Its SDK callback lifecycle
therefore differs from the explicitly joined `std::jthread` drivers.

## No consumer, no work

Sequence numbers count produced samples, so a driver that skips work while its
callback slot is empty does not advance them; a consumer that connects later
gets a whole, fresh sample with the next number. `OpenCvCamera` calls
`grab()` instead of `read()` (the pipeline keeps draining, nothing is decoded
or copied). `SimCamera`, `SimLidar` and `ICM20948` produce nothing and skip the
frame render, point generation or I2C read. `Mid360` drops point-cloud packets
and resets its accumulation state, so the first scan after a consumer connects
is built only from packets that arrived after it connected (its scan sequence
number is assigned when a scan completes); its IMU callback is unchanged.
`RPLidar` is unchanged and keeps scanning.
