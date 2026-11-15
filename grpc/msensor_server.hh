#pragma once

#include <grpcpp/grpcpp.h>
#include <grpcpp/server_builder.h>

#include <cstddef>
#include <memory>
#include <string>

#include "adc_service.hh"
#include "camera_service.hh"
#include "imu_service.hh"
#include "lidar_service.hh"
#include "msensor/bus/sensor_bus.hh"
#include "msensor/interface/Sensors.hh"

namespace msensor {

struct ServerOptions {
  std::string listen_address = "0.0.0.0:50051";
  int camera_jpeg_quality = 85; // 0..100, used when raw frames are encoded
  /// Clients each streaming RPC (LiDAR, IMU, camera) serves at once; one more
  /// gets RESOURCE_EXHAUSTED.
  size_t max_clients_per_stream = 8;
};

/**
 * @brief This class manages the gRPC server and provides methods to publish
 * data.
 *
 */
class SensorsServer {
public:
  /// Builds a private bus over `sensors`. Throws std::invalid_argument if
  /// options.camera_jpeg_quality is outside [0, 100].
  explicit SensorsServer(Sensors sensors, ServerOptions options = {});

  /// Serves the topics of a bus that other consumers (a recorder) may share.
  /// The server keeps it alive. Options first, so a call such as
  /// `SensorsServer({}, {.listen_address = ...})` stays unambiguous. A null
  /// bus is treated as an empty one. Same exceptions as above.
  SensorsServer(ServerOptions options, std::shared_ptr<SensorBus> bus);

  /// Legacy positional form; equivalent to the constructor above.
  SensorsServer(std::shared_ptr<IAdc> adc = nullptr,
                std::shared_ptr<ICamera> camera = nullptr,
                std::shared_ptr<IImu> imu = nullptr,
                std::shared_ptr<ILidar> lidar = nullptr,
                std::string listen_address = "0.0.0.0:50051");

  /// Build the server and start listening. Throws std::runtime_error if the
  /// server cannot be started (e.g. the address cannot be bound).
  void start();
  /// Block the calling thread until the server is shut down by stop().
  void wait();
  /// Shut down, giving in-flight streams up to one second to finish.
  void stop();

private:
  std::shared_ptr<SensorBus> bus_; // before the services that use it
  LidarServiceImpl lidar_service_;
  ImuServiceImpl imu_service_;
  CameraServiceImpl camera_service_;
  AdcServiceImpl adc_service_;
  std::string listen_address_;
  std::unique_ptr<grpc::Server> server_;
};

} // namespace msensor

#ifndef MSENSOR_NO_LEGACY_NAMES
// Legacy global name, for downstream code. Define MSENSOR_NO_LEGACY_NAMES to
// drop it.
using msensor::SensorsServer;
#endif
