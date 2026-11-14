#include "msensor/lidar/rp_lidar.hh"

#include <gtest/gtest.h>
#include <stdexcept>
#include <type_traits>

using msensor::RPLidar;

#if defined(__SANITIZE_THREAD__) || \
    (defined(__has_feature) && __has_feature(thread_sanitizer))
// The vendor SDK's rp::hal::Locker destructor unlocks its mutex without
// holding it, so destroying any SDK driver trips TSan's "unlock of an
// unlocked mutex". Third-party code we cannot change; suppress only that.
extern "C" const char *__tsan_default_suppressions() {
  return "mutex:rp::hal::Locker::~Locker\n";
}
#endif

static_assert(!std::is_copy_constructible_v<RPLidar>);
static_assert(!std::is_copy_assignable_v<RPLidar>);

// No hardware: the serial port is only opened by init().
TEST(RPLidarTest, ConstructAndDestroyWithMissingDevice) {
  RPLidar lidar("/dev/msensor-no-such-lidar");
}

TEST(RPLidarTest, InitOnMissingDeviceThrows) {
  RPLidar lidar("/dev/msensor-no-such-lidar");
  EXPECT_THROW(lidar.init(), std::runtime_error);
}

TEST(RPLidarTest, SamplingOnUnconnectedLidarStopsCleanly) {
  RPLidar lidar("/dev/msensor-no-such-lidar");
  lidar.startSampling(); // grabs fail and back off; destructor must join
  lidar.stopSampling();
  lidar.startSampling();
}
