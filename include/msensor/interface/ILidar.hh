#pragma once

#include "msensor/interface/Header.hh"

#include <functional>
#include <vector>

namespace msensor {

struct Point3 {
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
};

struct Point3I {
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
  float intensity = 0.0F;
};

using PointCloud3 = std::vector<Point3>;
using PointCloud3I = std::vector<Point3I>;

/**
 * @brief 3D Pointcloud scan
 *
 */
struct Scan3D {
  Scan3D() : header(Header{0, 0}) {}
  Header header;
  PointCloud3 points;
};

/**
 * @brief 3D Pointcloud with Intensity.
 *
 */
struct Scan3DI {
  Scan3DI() : header(Header{0, 0}) {}
  Header header;
  PointCloud3I points;
};

/**
 * @brief Interface for LiDAR devices producing point clouds.
 */
class ILidar {
public:
  /// Callback invoked for every produced scan.
  using ScanCallback = std::function<void(Scan3DI)>;

  virtual ~ILidar() = default;

  /**
   * @brief Perform device setup (connection, configuration, etc.).
   */
  virtual void init() = 0;
  /**
   * @brief Start the sampling loop.
   */
  virtual void startSampling() = 0;
  /**
   * @brief Stop the sampling loop.
   */
  virtual void stopSampling() = 0;

  /**
   * @brief Register the single consumer of lidar scans.
   *
   * The callback runs on the driver's sampling thread and receives ownership
   * of each scan. Passing an empty callback clears it. There is at most one
   * consumer per stream.
   *
   * @note The associated timestamp is assumed to be the time
   * point[0] was measured. Unit: ns (1/1000000000 sec).
   */
  virtual void setScanCallback(ScanCallback callback) = 0;
};
} // namespace msensor