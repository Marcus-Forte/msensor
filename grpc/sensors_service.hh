#pragma once

#include <boost/lockfree/spsc_queue.hpp>

#include "interface/IAdc.hh"
#include "interface/ICamera.hh"
#include "interface/IImu.hh"
#include "interface/ILidar.hh"
#include "sensors.grpc.pb.h"

using namespace msensor;

/**
 * @brief Call the methods of this service to publish data via gRPC.
 *
 */
class ScanService : public sensors::SensorService::Service {
public:
  /**
   * @brief Construct a new Scan Service object
   *
   * @param adc
   * @param camera
   * @param imu
   * @param lidar
   */
  ScanService(std::shared_ptr<IAdc> adc, std::shared_ptr<ICamera> camera,
              std::shared_ptr<IImu> imu, std::shared_ptr<ILidar> lidar);

  /**
   * @brief Stream LiDAR scans to the requesting client.
   */
  ::grpc::Status
  getLidarScan(::grpc::ServerContext *context,
               const ::sensors::SensorStreamRequest *request,
               ::grpc::ServerWriter<sensors::PointCloud3> *writer) override;

  /**
   * @brief Stream IMU samples to the requesting client.
   */
  ::grpc::Status
  getImuData(::grpc::ServerContext *context,
             const ::sensors::SensorStreamRequest *request,
             ::grpc::ServerWriter<sensors::IMUData> *writer) override;

  /**
   * @brief Return the most recent ADC reading.
   */
  ::grpc::Status getAdcData(::grpc::ServerContext *context,
                            const ::sensors::AdcDataRequest *request,
                            ::sensors::AdcData *response) override;

  /**
   * @brief Stream camera frames to the requesting client.
   */
  ::grpc::Status getCameraFrame(
      ::grpc::ServerContext *context,
      const ::sensors::CameraStreamRequest *request,
      ::grpc::ServerWriter<::sensors::CameraStreamReply> *writer) override;

private:
  std::shared_ptr<IAdc> adc_;
  std::shared_ptr<ICamera> camera_;
  std::shared_ptr<IImu> imu_;
  std::shared_ptr<ILidar> lidar_;
};