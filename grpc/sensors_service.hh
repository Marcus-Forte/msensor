#pragma once

#include <boost/lockfree/spsc_queue.hpp>

#include "interface/IAdc.hh"
#include "interface/ICamera.hh"
#include "interface/IImu.hh"
#include "interface/ILidar.hh"
#include "sensors.grpc.pb.h"

/**
 * @brief Call the methods of this service to publish data via gRPC.
 *
 */
class ScanService : public sensors::SensorService::Service {
public:
  /**
   * @brief Construct a new Scan Service object
   *
   */
  ScanService();

  /**
   * @brief Stream LiDAR scans to the requesting client.
   */
  ::grpc::Status
  getScan(::grpc::ServerContext *context,
          const ::sensors::SensorStreamRequest *request,
          ::grpc::ServerWriter<sensors::PointCloud3> *writer) override;

  /**
   * @brief Stream IMU samples to the requesting client.
   */
  ::grpc::Status
  getImu(::grpc::ServerContext *context,
         const ::sensors::SensorStreamRequest *request,
         ::grpc::ServerWriter<sensors::IMUData> *writer) override;

  /**
   * @brief Return the most recent ADC reading.
   */
  ::grpc::Status getAdc(::grpc::ServerContext *context,
                        const ::sensors::AdcDataRequest *request,
                        ::sensors::AdcData *response) override;

  /**
   * @brief Stream camera frames to the requesting client.
   */
  ::grpc::Status getCamera(
      ::grpc::ServerContext *context,
      const ::sensors::CameraStreamRequest *request,
      ::grpc::ServerWriter<::sensors::CameraStreamReply> *writer) override;

  /**
   * @brief Puts a scan data in the server queue.
   *
   * @param scan
   */
  void putScan(const std::shared_ptr<msensor::Scan3DI> &scan);
  /**
   * @brief Puts IMU data into the server queue.
   *
   * @param imu_data
   */
  void putImuData(msensor::IMUData imu_data);

  /**
   * @brief Puts (caches one sample) ADC data into the server.
   *
   * @param adc_data
   */
  void putAdcData(msensor::AdcSample adc_data);

  /**
   * @brief Puts (caches one sample) ADC data into the server.
   *
   * @param adc_data
   */
  void putImage(const cv::Mat &image);

private:
  template <typename T>
  using QueuePtrT = boost::lockfree::spsc_queue<std::shared_ptr<T>>;
  template <typename T> using QueueT = boost::lockfree::spsc_queue<T>;

  std::shared_ptr<QueuePtrT<msensor::Scan3DI>> scan_queue_;
  std::shared_ptr<QueueT<msensor::IMUData>> imu_queue_;
  std::shared_ptr<QueueT<cv::Mat>> camera_queue_;

  msensor::AdcSample adc_data_;
};