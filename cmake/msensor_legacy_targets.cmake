# ---------------------------------------------------------------------------
# Legacy target names, for downstream projects.
#
# Before msensor_ prefixed targets existed, the libraries had generic names
# (timing, file, config, sim_lidar, ...). They are defined here as ALIASes of
# the new real targets so a parent project that links the old names keeps
# building. msensor itself only uses the msensor:: names. Switch these off with
# -DMSENSOR_LEGACY_TARGET_NAMES=OFF if a name collides with the parent project.
#
# CMake cannot alias an alias, so each one points at the real target.
# ---------------------------------------------------------------------------
if(NOT MSENSOR_LEGACY_TARGET_NAMES)
  return()
endif()

set(_msensor_legacy_map
  timing          msensor_timing
  file            msensor_file
  config          msensor_config
  sim_lidar       msensor_sim_lidar
  mid360          msensor_mid360
  rp_lidar        msensor_rp_lidar
  icm-20948       msensor_icm20948
  sim_imu         msensor_sim_imu
  opencv_camera   msensor_opencv_camera
  sim_camera      msensor_sim_camera
  ads1115         msensor_ads1115
  sim_adc         msensor_sim_adc
  scan_recorder   msensor_scan_recorder
  msensor_conversions msensor_conversions_all
  IFile           msensor_interface
  IConfig         msensor_interface
  IImu            msensor_interface
  ILidar          msensor_interface
  IAdc            msensor_interface
  ICamera         msensor_camera_interface
  header_proto    msensor_header_proto
  lidar_proto     msensor_lidar_proto
  lidar_grpc      msensor_lidar_grpc
  imu_proto       msensor_imu_proto
  imu_grpc        msensor_imu_grpc
  camera_proto    msensor_camera_proto
  camera_grpc     msensor_camera_grpc
  adc_proto       msensor_adc_proto
  adc_grpc        msensor_adc_grpc
  recording_proto msensor_recording_proto
  sensors_proto   msensor_sensors_proto
  sensors_grpc    msensor_sensors_grpc)

list(LENGTH _msensor_legacy_map _msensor_legacy_len)
math(EXPR _msensor_legacy_last "${_msensor_legacy_len} - 1")
foreach(_i RANGE 0 ${_msensor_legacy_last} 2)
  math(EXPR _j "${_i} + 1")
  list(GET _msensor_legacy_map ${_i} _old)
  list(GET _msensor_legacy_map ${_j} _new)
  add_library(${_old} ALIAS ${_new})
endforeach()
