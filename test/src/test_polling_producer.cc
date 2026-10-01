#include "msensor/hub/PollingProducer.hh"

#include <atomic>
#include <gtest/gtest.h>
#include <stdexcept>

using namespace std::chrono_literals;
using msensor::PollingProducer;
using msensor::SensorHub;
using msensor::SubscribePolicy;

TEST(PollingProducer, PublishesAtFixedPeriod) {
  SensorHub<int> hub;
  auto sub = hub.subscribe(SubscribePolicy::bounded(100));
  std::atomic<int> n{0};
  PollingProducer<int> producer(
      hub, [&] { return std::make_shared<const int>(n++); }, 5ms);
  producer.start();
  std::this_thread::sleep_for(100ms);
  producer.stop();

  int count = 0;
  int expected = 0;
  while (auto m = sub->tryPop()) {
    EXPECT_EQ(*m, expected++);
    ++count;
  }
  EXPECT_GE(count, 5);
  EXPECT_LE(count, 25);
}

TEST(PollingProducer, NullSourceResultIsNotPublished) {
  SensorHub<int> hub;
  auto sub = hub.subscribe();
  PollingProducer<int> producer(
      hub, [] { return std::shared_ptr<const int>(); }, 1ms);
  producer.start();
  std::this_thread::sleep_for(30ms);
  producer.stop();
  EXPECT_EQ(hub.publishedCount(), 0u);
  EXPECT_TRUE(sub->empty());
}

TEST(PollingProducer, KeepsRunningAfterSourceThrows) {
  SensorHub<int> hub;
  auto sub = hub.subscribe(SubscribePolicy::bounded(10));
  std::atomic<int> calls{0};
  PollingProducer<int> producer(hub, [&]() -> std::shared_ptr<const int> {
    if (calls++ == 0)
      throw std::runtime_error("boom");
    return std::make_shared<const int>(1);
  });
  producer.start();
  auto m = sub->waitPop({}, 2s);
  producer.stop();
  ASSERT_NE(m, nullptr);
  EXPECT_EQ(*m, 1);
}

TEST(PollingProducer, StopIsPromptAndIdempotent) {
  SensorHub<int> hub;
  PollingProducer<int> producer(
      hub, [] { return std::make_shared<const int>(0); }, 10s);
  producer.start();
  producer.start();
  EXPECT_TRUE(producer.running());
  auto t0 = std::chrono::steady_clock::now();
  producer.stop();
  producer.stop();
  EXPECT_FALSE(producer.running());
  EXPECT_LT(std::chrono::steady_clock::now() - t0, 2s);
}

TEST(PollingProducer, DestructorStopsThread) {
  SensorHub<int> hub;
  {
    PollingProducer<int> producer(
        hub, [] { return std::make_shared<const int>(0); }, 1ms);
    producer.start();
    std::this_thread::sleep_for(10ms);
  }
  const auto published = hub.publishedCount();
  std::this_thread::sleep_for(20ms);
  EXPECT_EQ(hub.publishedCount(), published);
}
