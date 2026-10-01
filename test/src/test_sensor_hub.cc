#include "msensor/hub/SensorHub.hh"

#include <atomic>
#include <gtest/gtest.h>
#include <thread>

using namespace std::chrono_literals;
using msensor::SensorHub;
using msensor::SubscribePolicy;

TEST(SensorHub, PublishWithoutSubscribersIsNoop) {
  SensorHub<int> hub;
  hub.publish(1);
  EXPECT_EQ(hub.subscriberCount(), 0u);
  EXPECT_EQ(hub.publishedCount(), 1u);
}

TEST(SensorHub, EverySubscriberGetsEveryMessage) {
  SensorHub<int> hub;
  auto a = hub.subscribe(SubscribePolicy::bounded(4));
  auto b = hub.subscribe(SubscribePolicy::bounded(4));
  hub.publish(1);
  hub.publish(2);
  EXPECT_EQ(*a->tryPop(), 1);
  EXPECT_EQ(*a->tryPop(), 2);
  EXPECT_EQ(*b->tryPop(), 1);
  EXPECT_EQ(*b->tryPop(), 2);
  EXPECT_EQ(a->tryPop(), nullptr);
}

TEST(SensorHub, FanOutSharesPayload) {
  SensorHub<std::vector<int>> hub;
  auto a = hub.subscribe();
  auto b = hub.subscribe();
  hub.publish(std::vector<int>(1000, 7));
  EXPECT_EQ(a->tryPop().get(), b->tryPop().get());
}

TEST(SensorHub, LatestOnlyKeepsNewestAndCountsDrops) {
  SensorHub<int> hub;
  auto sub = hub.subscribe(SubscribePolicy::latestOnly());
  hub.publish(1);
  hub.publish(2);
  hub.publish(3);
  EXPECT_EQ(sub->dropped(), 2u);
  EXPECT_EQ(*sub->tryPop(), 3);
  EXPECT_EQ(sub->tryPop(), nullptr);
}

TEST(SensorHub, BoundedDropsOldest) {
  SensorHub<int> hub;
  auto sub = hub.subscribe(SubscribePolicy::bounded(2));
  for (int i = 1; i <= 4; ++i)
    hub.publish(i);
  EXPECT_EQ(sub->dropped(), 2u);
  EXPECT_EQ(*sub->tryPop(), 3);
  EXPECT_EQ(*sub->tryPop(), 4);
}

TEST(SensorHub, SlowSubscriberDoesNotAffectOthers) {
  SensorHub<int> hub;
  auto slow = hub.subscribe(SubscribePolicy::latestOnly());
  auto fast = hub.subscribe(SubscribePolicy::bounded(10));
  for (int i = 0; i < 10; ++i)
    hub.publish(i);
  EXPECT_EQ(slow->dropped(), 9u);
  EXPECT_EQ(fast->dropped(), 0u);
}

TEST(SensorHub, OnlySeesMessagesAfterSubscribe) {
  SensorHub<int> hub;
  hub.publish(1);
  auto sub = hub.subscribe();
  EXPECT_EQ(sub->tryPop(), nullptr);
}

TEST(SensorHub, DestroyedSubscriptionIsRemoved) {
  SensorHub<int> hub;
  {
    auto sub = hub.subscribe();
    EXPECT_EQ(hub.subscriberCount(), 1u);
  }
  EXPECT_EQ(hub.subscriberCount(), 0u);
  hub.publish(1);
}

TEST(SensorHub, SubscriptionOutlivingHubIsSafe) {
  auto hub = std::make_unique<SensorHub<int>>();
  auto sub = hub->subscribe();
  hub.reset();
  EXPECT_EQ(sub->tryPop(), nullptr);
  EXPECT_EQ(sub->waitPop({}, 10ms), nullptr);
}

TEST(SensorHub, NotifyFiresPerPublishAndCanPop) {
  SensorHub<int> hub;
  auto sub = hub.subscribe();
  int got = 0;
  int calls = 0;
  auto *raw = sub.get();
  sub->setNotify([&] {
    ++calls;
    if (auto m = raw->tryPop())
      got = *m;
  });
  hub.publish(5);
  hub.publish(6);
  EXPECT_EQ(calls, 2);
  EXPECT_EQ(got, 6);
}

TEST(SensorHub, NoNotifyAfterClear) {
  SensorHub<int> hub;
  auto sub = hub.subscribe();
  int calls = 0;
  sub->setNotify([&] { ++calls; });
  hub.publish(1);
  sub->setNotify({});
  hub.publish(2);
  EXPECT_EQ(calls, 1);
}

TEST(SensorHub, WaitPopTimesOut) {
  SensorHub<int> hub;
  auto sub = hub.subscribe();
  auto t0 = std::chrono::steady_clock::now();
  EXPECT_EQ(sub->waitPop({}, 30ms), nullptr);
  EXPECT_GE(std::chrono::steady_clock::now() - t0, 25ms);
}

TEST(SensorHub, WaitPopWakesOnPublish) {
  SensorHub<int> hub;
  auto sub = hub.subscribe();
  std::jthread producer([&] {
    std::this_thread::sleep_for(20ms);
    hub.publish(42);
  });
  auto m = sub->waitPop({}, 2s);
  ASSERT_NE(m, nullptr);
  EXPECT_EQ(*m, 42);
}

TEST(SensorHub, WaitPopReturnsOnStopRequest) {
  SensorHub<int> hub;
  auto sub = hub.subscribe();
  std::stop_source src;
  std::jthread stopper([&] {
    std::this_thread::sleep_for(20ms);
    src.request_stop();
  });
  auto t0 = std::chrono::steady_clock::now();
  EXPECT_EQ(sub->waitPop(src.get_token(), 10s), nullptr);
  EXPECT_LT(std::chrono::steady_clock::now() - t0, 5s);
}

TEST(SensorHub, ConcurrentPublishSubscribeUnsubscribe) {
  SensorHub<int> hub;
  std::atomic<bool> run{true};
  std::atomic<int> delivered{0};

  std::jthread producer([&] {
    int i = 0;
    while (run)
      hub.publish(i++);
  });

  std::vector<std::jthread> consumers;
  for (int c = 0; c < 4; ++c)
    consumers.emplace_back([&] {
      for (int n = 0; n < 200; ++n) {
        auto sub = hub.subscribe(SubscribePolicy::bounded(8));
        sub->setNotify([&] { delivered.fetch_add(1); });
        std::this_thread::sleep_for(100us);
        while (sub->tryPop()) {
        }
      }
    });
  consumers.clear(); // join
  run = false;
  producer.join();
  EXPECT_EQ(hub.subscriberCount(), 0u);
  EXPECT_GT(delivered.load(), 0);
}

TEST(SensorHub, EmptyReflectsBufferState) {
  SensorHub<int> hub;
  auto sub = hub.subscribe();
  EXPECT_TRUE(sub->empty());
  hub.publish(1);
  EXPECT_FALSE(sub->empty());
  sub->tryPop();
  EXPECT_TRUE(sub->empty());
}
