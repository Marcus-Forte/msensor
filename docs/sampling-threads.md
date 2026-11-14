# Sensor sampling threads

Sensor acquisition is delivered to services through callbacks. The thread
performing acquisition depends on the driver:

| Driver | Acquisition thread |
| --- | --- |
| `RPLidar` | Own `std::jthread`, started by `startSampling()`; repeatedly reads scans and emits its callback. |
| `ICM20948` | Own `std::jthread`, started by `startSampling()`; repeatedly reads samples and emits its callback. |
| `OpenCvCamera` | Own `std::jthread`, started by `startSampling()`; repeatedly captures frames and emits its callback. |
| `SimLidar`, `SimImu`, `SimCamera` | Each simulator starts its own `std::jthread` in `startSampling()` and emits generated data from it. |
| `Mid360` | Starts the Livox SDK with `LivoxLidarSdkStart()` and receives data through SDK callbacks; msensor does not create a sampling `std::jthread` for it. |
| `ADS1115` | Pull-based: `readSingleEnded()` performs a read when called; the driver does not start a background sampling thread. |
| `RecordingSensorDriver` | Own `std::jthread`, started by `startSampling()`; parses the recording, paces entries by their timestamps and emits them. |
| `SensorsRemoteClient` | Two `std::jthread`s started by `start()`, one per gRPC stream (LiDAR, IMU); each converts received messages and emits them. |

For the drivers that own a `std::jthread`, `stopSampling()` requests stop and
joins the thread. This keeps their continuous polling or capture loops off the
thread that starts the service. Other driver operations, such as initialization
or configuration, are not automatically moved to a worker thread and may run
synchronously on their caller.

The sensor interfaces allow one callback consumer per stream. `CallbackSlot`
invokes that callback directly on the producer's thread and moves the sample
into it. Clearing the callback (`setXCallback({})`) waits for an invocation
that is still running, so the consumer may be destroyed once it returns. The
gRPC `SensorStreamReactor` registers such a callback and starts asynchronous
writes; it does not dedicate a blocked thread to each client. If a write is
already in flight, the reactor retains only the latest pending sample, so slow
clients can miss samples rather than block acquisition. Once the client
cancels or a write fails, the reactor starts no further writes.

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
