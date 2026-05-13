#pragma once

#include "camera.pb.h"
#include "imu.pb.h"
#include "lidar.pb.h"
#include "msensor/interface/ICamera.hh"
#include "msensor/interface/IImu.hh"
#include "msensor/interface/ILidar.hh"

/**
 * @brief Convert a gRPC point cloud message into an msensor point cloud.
 */
std::shared_ptr<msensor::Scan3DI> fromProtobuf(const sensors::PointCloud3 &msg);

/**
 * @brief Convert an msensor point cloud to gRPC point cloud message.
 */
sensors::PointCloud3 toProtobuf(const std::shared_ptr<msensor::Scan3DI> &msg);

/**
 * @brief Convert a gRPC IMU message into an msensor IMU sample.
 */
msensor::IMUData fromProtobuf(const sensors::IMUData &msg);

/**
 * @brief Convert a msensor IMU message into a gRPC IMU message.
 */
sensors::IMUData toProtobuf(msensor::IMUData msg);

/**
 * @brief Converts a msensor camera frame to gRPC camera message.
 *
 * @param Frame Camera Frame
 * @param quality JPEG quality [0-100]
 */
sensors::CameraStreamReply toProtobuf(const msensor::CameraFrame &Frame,
                                      int quality = 85);