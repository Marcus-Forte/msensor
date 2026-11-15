// Tests of the in-process sensor bus: Topic, Subscription and SensorBus.

#include "msensor/bus/sensor_bus.hh"
#include "msensor/bus/topic.hh"
#include "msensor/interface/CallbackSlot.hh"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

namespace {

struct Item {
  int seq = 0;
  std::vector<int> payload; // non-trivial
};

/// A driver stand-in with the single-consumer contract of the real ones.
struct FakeDriver {
  msensor::CallbackSlot<Item> slot;
  std::atomic<int> installs{0};
  std::atomic<int> clears{0};

  msensor::Topic<Item>::Install install() {
    return [this](std::function<void(Item)> callback) {
      (callback ? installs : clears).fetch_add(1);
      slot.setCallback(std::move(callback));
    };
  }
  void emit(int seq) { slot.emit(Item{seq, std::vector<int>(16, seq)}); }
};

/// Waits until `pred` holds, polling; the dispatcher is asynchronous.
template <class Pred>
bool waitFor(Pred pred, std::chrono::milliseconds limit = 10s) {
  const auto end = std::chrono::steady_clock::now() + limit;
  while (!pred()) {
    if (std::chrono::steady_clock::now() > end) {
      return false;
    }
    std::this_thread::sleep_for(100us);
  }
  return true;
}

} // namespace

TEST(Topic, SubscriberGetsSamplesInOrderThroughTryPop) {
  msensor::Topic<Item> topic;
  auto sub = topic.subscribe(8);
  for (int seq = 1; seq <= 5; ++seq) {
    topic.publish(Item{seq, {}});
  }
  for (int seq = 1; seq <= 5; ++seq) {
    const auto sample = sub->tryPop();
    ASSERT_TRUE(sample);
    EXPECT_EQ(sample->seq, seq);
  }
  EXPECT_FALSE(sub->tryPop());
  EXPECT_EQ(sub->dropped(), 0U);
}

TEST(Topic, PublishWithoutSubscribersIsANoOp) {
  msensor::Topic<Item> topic;
  topic.publish(Item{1, {}});
  EXPECT_EQ(topic.subscriberCount(), 0U);
}

TEST(Topic, SubscribersShareTheSameSampleObject) {
  msensor::Topic<Item> topic;
  auto a = topic.subscribe(2);
  auto b = topic.subscribe(2);
  topic.publish(Item{7, std::vector<int>(1000, 7)});
  const auto sa = a->tryPop();
  const auto sb = b->tryPop();
  ASSERT_TRUE(sa);
  ASSERT_TRUE(sb);
  EXPECT_EQ(sa.get(), sb.get()); // one allocation, not one per subscriber
}

TEST(Topic, FullFifoDropsTheOldestAndCountsIt) {
  msensor::Topic<Item> topic;
  auto sub = topic.subscribe(3);
  for (int seq = 1; seq <= 10; ++seq) {
    topic.publish(Item{seq, {}});
  }
  EXPECT_EQ(sub->dropped(), 7U);
  EXPECT_EQ(sub->size(), 3U);
  for (int seq = 8; seq <= 10; ++seq) {
    EXPECT_EQ(sub->tryPop()->seq, seq);
  }
}

TEST(Topic, CapacityIsAtLeastOne) {
  msensor::Topic<Item> topic;
  auto sub = topic.subscribe(0);
  EXPECT_EQ(sub->capacity(), 1U);
  topic.publish(Item{1, {}});
  topic.publish(Item{2, {}});
  EXPECT_EQ(sub->tryPop()->seq, 2);
}

TEST(Topic, BlockingPopWaitsForASampleAndHonoursStop) {
  msensor::Topic<Item> topic;
  auto sub = topic.subscribe(4);
  std::atomic<int> got{-1};
  std::jthread consumer([&](std::stop_token st) {
    if (auto s = sub->pop(st)) {
      got.store(s->seq);
    }
    EXPECT_FALSE(sub->pop(st)); // returns null once stop is requested
  });
  std::this_thread::sleep_for(50ms);
  EXPECT_EQ(got.load(), -1);
  topic.publish(Item{42, {}});
  EXPECT_TRUE(waitFor([&] { return got.load() == 42; }));
  consumer.request_stop();
}

// Invariant 2: hooks run on the dispatcher thread, not the publisher's.
TEST(Topic, WakeHookRunsOnTheDispatcherThreadNotThePublishers) {
  msensor::Topic<Item> topic;
  auto sub = topic.subscribe(4);
  std::atomic<std::thread::id> hook_thread{};
  sub->setWakeHook([&] {
    hook_thread.store(std::this_thread::get_id());
    while (sub->tryPop()) {
    }
  });
  topic.publish(Item{1, {}});
  ASSERT_TRUE(waitFor([&] { return hook_thread.load() != std::thread::id{}; }));
  EXPECT_NE(hook_thread.load(), std::this_thread::get_id());
}

TEST(Topic, SetWakeHookFiresForSamplesAlreadyQueued) {
  msensor::Topic<Item> topic;
  auto sub = topic.subscribe(4);
  topic.publish(Item{1, {}});
  std::atomic<int> calls{0};
  sub->setWakeHook([&] { calls.fetch_add(1); });
  EXPECT_TRUE(waitFor([&] { return calls.load() >= 1; }));
}

// Invariant 2: publishing never waits on a subscriber, even one that is stuck
// inside its hook.
TEST(Topic, PublishDoesNotWaitForAStuckHook) {
  // Declared before the topic: the hook still uses them when it is released,
  // and the subscription's destructor waits for it.
  std::mutex m;
  std::condition_variable cv;
  bool entered = false;
  bool release = false;
  msensor::Topic<Item> topic;
  auto stuck = topic.subscribe(2);
  auto other = topic.subscribe(10000);
  stuck->setWakeHook([&] {
    std::unique_lock lock(m);
    entered = true;
    cv.notify_all();
    cv.wait(lock, [&] { return release; });
  });
  topic.publish(Item{0, {}});
  {
    std::unique_lock lock(m);
    ASSERT_TRUE(cv.wait_for(lock, 10s, [&] { return entered; }));
  }
  const auto start = std::chrono::steady_clock::now();
  for (int seq = 1; seq <= 5000; ++seq) {
    topic.publish(Item{seq, {}});
  }
  EXPECT_LT(std::chrono::steady_clock::now() - start, 2s);
  EXPECT_GT(stuck->dropped(), 0U);
  EXPECT_EQ(other->dropped(), 0U); // 5001 samples fit in 10000
  {
    std::lock_guard lock(m);
    release = true;
  }
  cv.notify_all();
}

// Invariant 4: a subscriber that never reads loses only its own samples.
TEST(Topic, StalledSubscriberDoesNotAffectAnother) {
  msensor::Topic<Item> topic;
  auto stalled = topic.subscribe(4);
  auto healthy = topic.subscribe(1000);
  for (int seq = 1; seq <= 500; ++seq) {
    topic.publish(Item{seq, {}});
  }
  EXPECT_EQ(stalled->dropped(), 496U);
  EXPECT_EQ(healthy->dropped(), 0U);
  for (int seq = 1; seq <= 500; ++seq) {
    const auto sample = healthy->tryPop();
    ASSERT_TRUE(sample);
    ASSERT_EQ(sample->seq, seq);
  }
}

// Invariant 3 at the topic level, with real concurrency: two consumers on
// their own threads each see every sample in order.
TEST(Topic, TwoConcurrentConsumersSeeEverySampleInOrder) {
  msensor::Topic<Item> topic;
  constexpr int kCount = 5000;
  auto a = topic.subscribe(kCount);
  auto b = topic.subscribe(kCount);
  std::vector<int> got_a, got_b;
  auto drain = [&](msensor::Subscription<Item> &sub, std::vector<int> &out) {
    return std::jthread([&](std::stop_token st) {
      while (static_cast<int>(out.size()) < kCount) {
        if (auto s = sub.pop(st)) {
          out.push_back(s->seq);
        } else {
          return;
        }
      }
    });
  };
  {
    auto ta = drain(*a, got_a);
    auto tb = drain(*b, got_b);
    for (int seq = 0; seq < kCount; ++seq) {
      topic.publish(Item{seq, {}});
    }
  } // jthreads join: each has read kCount samples
  ASSERT_EQ(got_a.size(), static_cast<size_t>(kCount));
  ASSERT_EQ(got_b.size(), static_cast<size_t>(kCount));
  for (int i = 0; i < kCount; ++i) {
    ASSERT_EQ(got_a[i], i);
    ASSERT_EQ(got_b[i], i);
  }
}

// Invariant 5: lazy attachment.
TEST(Topic, DriverHasNoCallbackWithoutASubscriber) {
  FakeDriver driver;
  msensor::Topic<Item> topic("t", driver.install());
  EXPECT_FALSE(driver.slot.hasCallback());
  EXPECT_FALSE(topic.attached());
  EXPECT_EQ(driver.installs.load(), 0);

  auto first = topic.subscribe(2);
  EXPECT_TRUE(driver.slot.hasCallback());
  EXPECT_TRUE(topic.attached());
  auto second = topic.subscribe(2);
  EXPECT_EQ(driver.installs.load(), 1); // installed once, not per subscriber

  first.reset();
  EXPECT_TRUE(driver.slot.hasCallback()); // one subscriber left
  second.reset();
  EXPECT_FALSE(driver.slot.hasCallback());
  EXPECT_FALSE(topic.attached());
  EXPECT_EQ(driver.clears.load(), 1);

  auto again = topic.subscribe(2); // and it attaches again
  EXPECT_TRUE(driver.slot.hasCallback());
  EXPECT_EQ(driver.installs.load(), 2);
}

TEST(Topic, SamplesFlowFromTheDriverToSubscribers) {
  FakeDriver driver;
  msensor::Topic<Item> topic("t", driver.install());
  auto sub = topic.subscribe(8);
  driver.emit(3);
  const auto sample = sub->tryPop();
  ASSERT_TRUE(sample);
  EXPECT_EQ(sample->seq, 3);
  EXPECT_EQ(sample->payload.size(), 16U);
}

TEST(Topic, SubscriptionMayOutliveTheTopicObject) {
  FakeDriver driver;
  std::unique_ptr<msensor::Subscription<Item>> sub;
  {
    msensor::Topic<Item> topic("t", driver.install());
    sub = topic.subscribe(2);
  }
  driver.emit(1);
  EXPECT_EQ(sub->tryPop()->seq, 1);
  sub.reset();
  EXPECT_FALSE(driver.slot.hasCallback());
}

// Invariant 6: destroying a subscription while samples are being published
// (and while its hook is firing) is safe, and its hook never runs afterwards.
// The sanitizer presets turn a violation into a report.
TEST(Topic, DestroyingASubscriptionWhilePublishingIsSafe) {
  FakeDriver driver;
  msensor::Topic<Item> topic("t", driver.install());
  auto keep_alive = topic.subscribe(4); // a second subscriber stays throughout

  std::atomic<bool> stop{false};
  std::thread publisher([&] {
    for (int seq = 0; !stop.load(); ++seq) {
      driver.emit(seq);
    }
  });

  struct Probe {
    std::atomic<bool> alive{true};
    std::atomic<int> calls{0};
    std::atomic<int> after_death{0};
  };
  int total_calls = 0;
  for (int round = 0; round < 300; ++round) {
    auto probe = std::make_unique<Probe>();
    auto sub = topic.subscribe(2);
    auto *raw_sub = sub.get();
    auto *raw_probe = probe.get();
    sub->setWakeHook([raw_sub, raw_probe] {
      if (!raw_probe->alive.load()) {
        raw_probe->after_death.fetch_add(1); // would be a use after free
      }
      raw_probe->calls.fetch_add(1);
      while (raw_sub->tryPop()) {
      }
    });
    if (round % 3 == 0) {
      std::this_thread::sleep_for(200us); // let the hook fire a few times
    }
    sub.reset();
    probe->alive.store(false); // the hook must not run from here on
    std::this_thread::sleep_for(20us);
    EXPECT_EQ(probe->after_death.load(), 0);
    total_calls += probe->calls.load();
    probe.reset(); // freed: a late hook would touch freed memory
  }
  stop.store(true);
  publisher.join();
  EXPECT_GT(total_calls, 0);
  EXPECT_TRUE(driver.slot.hasCallback()); // keep_alive still attached
  keep_alive.reset();
  EXPECT_FALSE(driver.slot.hasCallback());
}

// Invariant 6, last subscriber: the driver's callback is cleared while the
// driver is emitting, and attaches again afterwards.
TEST(Topic, AttachAndDetachWhileTheDriverEmitsIsSafe) {
  FakeDriver driver;
  msensor::Topic<Item> topic("t", driver.install());
  std::atomic<bool> stop{false};
  std::thread emitter([&] {
    for (int seq = 0; !stop.load(); ++seq) {
      driver.emit(seq);
    }
  });
  std::atomic<int> hook_calls{0};
  for (int round = 0; round < 300; ++round) {
    auto sub = topic.subscribe(4);
    auto *raw = sub.get();
    sub->setWakeHook([raw, &hook_calls] {
      hook_calls.fetch_add(1);
      while (raw->tryPop()) {
      }
    });
    std::this_thread::sleep_for(50us);
  }
  stop.store(true);
  emitter.join();
  EXPECT_FALSE(driver.slot.hasCallback());
  EXPECT_EQ(driver.installs.load(), driver.clears.load());
}

TEST(SensorBus, TopicsExistOnlyForAvailableSensors) {
  struct FakeImu : msensor::IImu {
    msensor::CallbackSlot<msensor::IMUData> slot;
    void startSampling() override {}
    void stopSampling() override {}
    void setImuCallback(ImuCallback cb) override {
      slot.setCallback(std::move(cb));
    }
  };
  auto imu = std::make_shared<FakeImu>();
  msensor::SensorBus bus(msensor::Sensors{.imu = imu});
  EXPECT_EQ(bus.lidar(), nullptr);
  EXPECT_EQ(bus.camera(), nullptr);
  ASSERT_NE(bus.imu(), nullptr);

  // Invariant 5 through the bus: no subscriber, no callback.
  EXPECT_FALSE(imu->slot.hasCallback());
  {
    auto sub = bus.imu()->subscribe(4);
    EXPECT_TRUE(imu->slot.hasCallback());
    msensor::IMUData sample{};
    sample.header.sequence_number = 9;
    imu->slot.emit(sample);
    const auto got = sub->tryPop();
    ASSERT_TRUE(got);
    EXPECT_EQ(got->header.sequence_number, 9U);
  }
  EXPECT_FALSE(imu->slot.hasCallback());
}

TEST(SensorBus, KeepsTheDriversAliveForItsSubscribers) {
  struct FakeLidar : msensor::ILidar {
    msensor::CallbackSlot<msensor::Scan3DI> slot;
    void init() override {}
    void startSampling() override {}
    void stopSampling() override {}
    void setScanCallback(ScanCallback cb) override {
      slot.setCallback(std::move(cb));
    }
  };
  std::weak_ptr<FakeLidar> weak;
  std::unique_ptr<msensor::Subscription<msensor::Scan3DI>> sub;
  {
    auto lidar = std::make_shared<FakeLidar>();
    weak = lidar;
    auto bus =
        std::make_unique<msensor::SensorBus>(msensor::Sensors{.lidar = lidar});
    lidar.reset();
    EXPECT_FALSE(weak.expired()); // the bus owns it
    sub = bus->lidar()->subscribe(1);
  } // bus gone; the subscription still keeps the driver attached and alive
  EXPECT_FALSE(weak.expired());
  sub.reset();
  EXPECT_TRUE(weak.expired());
}
