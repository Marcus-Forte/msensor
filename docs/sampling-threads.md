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

For the drivers that own a `std::jthread`, `stopSampling()` requests stop and
joins the thread. This keeps their continuous polling or capture loops off the
thread that starts the service. Other driver operations, such as initialization
or configuration, are not automatically moved to a worker thread and may run
synchronously on their caller.

The sensor interfaces allow one callback consumer per stream. `CallbackSlot`
invokes that callback directly on the producer's thread. The gRPC
`SensorStreamReactor` registers such a callback and starts asynchronous writes;
it does not dedicate a blocked thread to each client. If a write is already in
flight, the reactor retains only the latest pending sample, so slow clients can
miss samples rather than block acquisition.

The Mid360 driver currently leaves `stopSampling()` unimplemented. Its SDK
callback lifecycle therefore differs from the explicitly joined `std::jthread`
drivers.
