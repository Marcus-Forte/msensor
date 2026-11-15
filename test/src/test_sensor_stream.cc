// Unit tests of SensorStreamReactor's bounded-FIFO write logic, driven
// deterministically through a fake reactor base instead of a gRPC server.

#include "msensor/bus/topic.hh"
#include "msensor/interface/CallbackSlot.hh"
#include "msensor/log/log.hh"
#include "sensor_stream.hh"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace {

struct Sample {
  int seq = -1;
  std::vector<int> payload; // non-trivial, so moves matter
};

struct Response {
  int seq = -1;
};

/// Records what the reactor asks of gRPC and checks the callback API rules:
/// at most one write in flight, no write after Finish, Finish exactly once.
struct Wire {
  std::mutex m;
  std::vector<int> written; // seq of every StartWrite, in order
  int finishes = 0;
  int violations = 0;
  bool writing = false;
  bool finished = false;
  int destroyed = 0;
  std::thread::id write_thread; // thread of the last StartWrite

  std::vector<int> writes() {
    std::lock_guard lock(m);
    return written;
  }
};

class FakeWriteReactor {
public:
  explicit FakeWriteReactor(Wire &wire) : wire_(wire) {}
  virtual ~FakeWriteReactor() {
    std::lock_guard lock(wire_.m);
    ++wire_.destroyed;
  }

  virtual void OnWriteDone(bool /*ok*/) {}
  virtual void OnCancel() {}
  virtual void OnDone() = 0;

  void StartWrite(const Response *response) {
    std::lock_guard lock(wire_.m);
    if (wire_.writing || wire_.finished) {
      ++wire_.violations;
    }
    wire_.writing = true;
    wire_.write_thread = std::this_thread::get_id();
    wire_.written.push_back(response->seq);
  }

  void Finish(const grpc::Status & /*status*/) {
    std::lock_guard lock(wire_.m);
    if (wire_.finished) {
      ++wire_.violations;
    }
    wire_.finished = true;
    ++wire_.finishes;
  }

  /// Test side: the in-flight write completes (OnWriteDone follows).
  void completeWrite() {
    std::lock_guard lock(wire_.m);
    wire_.writing = false;
  }

private:
  Wire &wire_;
};

/// SensorStreamReactor needs a default-constructible base; this one binds to
/// the wire of the test that is currently constructing a reactor.
thread_local Wire *g_wire = nullptr;

class TestBase : public FakeWriteReactor {
public:
  TestBase() : FakeWriteReactor(*g_wire) {}
};

using Reactor = msensor::SensorStreamReactor<Sample, Response, TestBase>;

/// A driver stand-in: one callback slot, like every real driver.
struct Driver {
  msensor::CallbackSlot<Sample> slot;
  std::atomic<int> installs{0};
  std::atomic<int> clears{0};

  msensor::Topic<Sample>::Install install() {
    return [this](std::function<void(Sample)> callback) {
      (callback ? installs : clears).fetch_add(1);
      slot.setCallback(std::move(callback));
    };
  }

  void emit(int seq) { slot.emit(Sample{seq, std::vector<int>(64, seq)}); }
};

/// A driver and its topic, plus a barrier that tells when the dispatcher has
/// handled everything emitted so far. The barrier is a subscription made after
/// the clients; hooks of one dispatch pass run in subscription order, so once
/// the barrier has seen a sample, every earlier client has handled it.
struct Feed {
  Driver driver;
  msensor::Topic<Sample> topic{"test", driver.install()};
  std::mutex m;
  std::condition_variable cv;
  int emitted = 0;
  int seen = 0;
  std::unique_ptr<msensor::Subscription<Sample>> barrier;

  /// Call after every client has been created.
  void armBarrier() {
    barrier = topic.subscribe(100000);
    auto *sub = barrier.get();
    sub->setWakeHook([this, sub] {
      int n = 0;
      while (sub->tryPop()) {
        ++n;
      }
      std::lock_guard lock(m);
      seen += n;
      cv.notify_all();
    });
  }
  void dropBarrier() { barrier.reset(); }

  /// Emits from this (the "driver") thread and waits until it is handled.
  void emit(int seq) {
    driver.emit(seq);
    ++emitted;
    std::unique_lock lock(m);
    ASSERT_TRUE(cv.wait_for(lock, std::chrono::seconds(10),
                            [&] { return seen == emitted; }));
  }
};

/// One client of a feed: a reactor bound to a fake wire.
struct Client {
  Wire wire;
  msensor::ClientLimit limit{1};
  std::atomic<int> conversions{0};
  std::atomic<std::thread::id> convert_thread{};
  Reactor *reactor = nullptr;

  Client(Feed &feed, size_t capacity) {
    EXPECT_TRUE(limit.tryAcquire());
    g_wire = &wire;
    reactor = new Reactor(
        feed.topic.subscribe(capacity),
        [this](const Sample &sample, Response &out) {
          conversions.fetch_add(1);
          convert_thread.store(std::this_thread::get_id());
          out.seq = sample.seq;
        },
        limit, "test");
    g_wire = nullptr;
  }

  /// gRPC reports the outstanding write as finished.
  void writeDone(bool ok) {
    reactor->completeWrite();
    reactor->OnWriteDone(ok);
  }
};

/// The usual case: one feed, one client, a barrier.
struct Harness : Feed, Client {
  explicit Harness(size_t capacity = 1) : Feed(), Client(*this, capacity) {
    armBarrier();
  }
  /// gRPC's final callback; the barrier goes first so the driver's callback
  /// is cleared with the last subscription.
  void done() {
    dropBarrier();
    reactor->OnDone();
  }
};

} // namespace

TEST(SensorStreamReactor, InstallsCallbackAndWritesFirstSample) {
  Harness h;
  EXPECT_EQ(h.driver.installs.load(), 1);
  h.emit(1);
  EXPECT_EQ(h.wire.writes(), std::vector<int>{1});
  EXPECT_EQ(h.conversions.load(), 1);
  h.writeDone(true);
  h.reactor->OnCancel();
  h.done();
  EXPECT_EQ(h.wire.violations, 0);
}

TEST(SensorStreamReactor, KeepsOnlyTheLatestSampleWhileWriting) {
  Harness h(1);
  h.emit(1);
  h.emit(2);
  h.emit(3);
  h.emit(4);
  // Samples 2 and 3 were superseded and never converted.
  EXPECT_EQ(h.conversions.load(), 1);
  h.writeDone(true);
  EXPECT_EQ(h.wire.writes(), (std::vector<int>{1, 4}));
  EXPECT_EQ(h.conversions.load(), 2);
  // Nothing pending: the next completion starts no write.
  h.writeDone(true);
  EXPECT_EQ(h.wire.writes(), (std::vector<int>{1, 4}));
  h.emit(5);
  EXPECT_EQ(h.wire.writes(), (std::vector<int>{1, 4, 5}));
  h.writeDone(true);
  h.reactor->OnCancel();
  h.done();
  EXPECT_EQ(h.wire.violations, 0);
}

TEST(SensorStreamReactor, ZeroCapacityBehavesAsOne) {
  Harness h(0);
  h.emit(1);
  h.emit(2);
  h.emit(3);
  h.writeDone(true);
  EXPECT_EQ(h.wire.writes(), (std::vector<int>{1, 3}));
  h.reactor->OnCancel();
  h.writeDone(true);
  h.done();
}

TEST(SensorStreamReactor, QueuePreservesFifoOrder) {
  Harness h(8);
  for (int seq = 1; seq <= 5; ++seq) {
    h.emit(seq);
  }
  EXPECT_EQ(h.conversions.load(), 1); // queued samples are not converted yet
  for (int i = 0; i < 4; ++i) {
    h.writeDone(true);
  }
  EXPECT_EQ(h.wire.writes(), (std::vector<int>{1, 2, 3, 4, 5}));
  EXPECT_EQ(h.conversions.load(), 5);
  h.writeDone(true); // empty queue: no further write
  EXPECT_EQ(h.wire.writes().size(), 5U);
  h.reactor->OnCancel();
  h.done();
  EXPECT_EQ(h.wire.violations, 0);
}

TEST(SensorStreamReactor, OverflowDropsTheOldestAndKeepsTheNewestInOrder) {
  Harness h(3);
  h.emit(1); // written at once
  for (int seq = 2; seq <= 9; ++seq) {
    h.emit(seq); // queue holds the newest 3: 7, 8, 9
  }
  for (int i = 0; i < 4; ++i) {
    h.writeDone(true);
  }
  EXPECT_EQ(h.wire.writes(), (std::vector<int>{1, 7, 8, 9}));
  EXPECT_EQ(h.conversions.load(), 4); // dropped samples were never converted
  h.reactor->OnCancel();
  h.done();
  EXPECT_EQ(h.wire.violations, 0);
}

TEST(SensorStreamReactor, CancelWithQueuedSamplesWritesNothingMore) {
  Harness h(4);
  h.emit(1);
  h.emit(2);
  h.emit(3); // queued
  h.reactor->OnCancel();
  EXPECT_EQ(h.wire.finishes, 1);
  h.writeDone(true); // late success: queue was discarded
  h.emit(4);
  EXPECT_EQ(h.wire.writes(), std::vector<int>{1});
  EXPECT_EQ(h.conversions.load(), 1);
  EXPECT_EQ(h.wire.violations, 0);
  h.done();
  EXPECT_EQ(h.wire.destroyed, 1);
}

TEST(ResolveQueueSize, ZeroGivesTheDefaultInRangeIsKeptAboveCapGivesTheCap) {
  constexpr msensor::QueueLimits limits{200, 4000};
  EXPECT_EQ(msensor::resolveQueueSize(0, limits), 200U);
  EXPECT_EQ(msensor::resolveQueueSize(1, limits), 1U);
  EXPECT_EQ(msensor::resolveQueueSize(777, limits), 777U);
  EXPECT_EQ(msensor::resolveQueueSize(4000, limits), 4000U);
  EXPECT_EQ(msensor::resolveQueueSize(4001, limits), 4000U);
  EXPECT_EQ(msensor::resolveQueueSize(UINT32_MAX, limits), 4000U);
}

TEST(ResolveQueueSize, StreamConstants) {
  EXPECT_EQ(msensor::resolveQueueSize(0, msensor::kImuQueueLimits), 200U);
  EXPECT_EQ(msensor::resolveQueueSize(9999, msensor::kImuQueueLimits), 4000U);
  EXPECT_EQ(msensor::resolveQueueSize(0, msensor::kLidarQueueLimits), 1U);
  EXPECT_EQ(msensor::resolveQueueSize(100, msensor::kLidarQueueLimits), 64U);
  EXPECT_EQ(msensor::resolveQueueSize(0, msensor::kCameraQueueLimits), 1U);
  EXPECT_EQ(msensor::resolveQueueSize(100, msensor::kCameraQueueLimits), 8U);
}

// gRPC may deliver a successful OnWriteDone after OnCancel; a pending sample
// must not be written once Finish() has been called.
TEST(SensorStreamReactor, NoWriteAfterCancelEvenIfAWriteSucceedsLater) {
  Harness h;
  h.emit(1);
  h.emit(2); // pending
  h.reactor->OnCancel();
  EXPECT_EQ(h.wire.finishes, 1);
  h.writeDone(true);
  EXPECT_EQ(h.wire.writes(), std::vector<int>{1});
  h.emit(3);
  EXPECT_EQ(h.wire.writes(), std::vector<int>{1});
  EXPECT_EQ(h.wire.finishes, 1);
  EXPECT_EQ(h.wire.violations, 0);
  h.done();
}

TEST(SensorStreamReactor, FailedWriteFinishesOnceAndStopsWriting) {
  Harness h;
  h.emit(1);
  h.emit(2); // pending
  h.writeDone(false);
  EXPECT_EQ(h.wire.finishes, 1);
  h.emit(3);
  h.reactor->OnCancel(); // may still arrive
  EXPECT_EQ(h.wire.finishes, 1);
  EXPECT_EQ(h.wire.writes(), std::vector<int>{1});
  EXPECT_EQ(h.wire.violations, 0);
  h.done();
}

TEST(SensorStreamReactor, OnDoneClearsCallbackReleasesStreamAndDeletes) {
  Harness h;
  h.emit(1);
  h.writeDone(true);
  h.reactor->OnCancel();
  h.done(); // deletes the reactor
  EXPECT_EQ(h.driver.clears.load(), 1);
  EXPECT_FALSE(h.driver.slot.hasCallback());
  EXPECT_EQ(h.limit.active(), 0U);
  EXPECT_EQ(h.wire.destroyed, 1);
  h.driver.emit(2); // no callback any more: must not touch the reactor
  EXPECT_EQ(h.wire.writes(), std::vector<int>{1});
}

// A producer thread and a "gRPC" thread race the way they do in a server:
// writes complete on one thread while samples arrive on another, and the
// stream is cancelled mid-flight. Run under ThreadSanitizer as well.
TEST(SensorStreamReactor, ConcurrentProducerAndCompletions) {
  for (int round = 0; round < 20; ++round) {
    Harness h;
    std::atomic<bool> stop{false};
    std::thread producer([&] {
      for (int seq = 0; !stop.load(); ++seq) {
        h.driver.emit(seq);
        if (seq % 64 == 0) {
          std::this_thread::yield();
        }
      }
    });
    std::thread grpc([&] {
      size_t seen = 0;
      while (!stop.load()) {
        bool writing;
        size_t writes;
        {
          std::lock_guard lock(h.wire.m);
          writing = h.wire.writing;
          writes = h.wire.written.size();
        }
        if (writing && writes > seen) {
          seen = writes;
          h.writeDone(true);
        } else {
          std::this_thread::yield();
        }
        if (seen > 200) {
          h.reactor->OnCancel();
          stop.store(true);
        }
      }
    });
    producer.join();
    grpc.join();
    bool outstanding;
    {
      std::lock_guard lock(h.wire.m);
      outstanding = h.wire.writing;
    }
    if (outstanding) {
      h.writeDone(true); // the last write completes after the cancel
    }
    h.done();

    const auto writes = h.wire.writes();
    ASSERT_GT(writes.size(), 200U);
    for (size_t i = 1; i < writes.size(); ++i) {
      ASSERT_LT(writes[i - 1], writes[i]) << "samples reordered";
    }
    EXPECT_EQ(h.wire.finishes, 1);
    EXPECT_EQ(h.wire.violations, 0);
    EXPECT_EQ(h.limit.active(), 0U);
  }
}

// ---- Several clients on one stream ----

// Invariant 3: with queues deep enough, every client gets every sample, in
// order.
TEST(SensorStreamReactor, TwoClientsBothReceiveEverySampleInOrder) {
  Feed feed;
  Client a(feed, 64);
  Client b(feed, 64);
  feed.armBarrier();
  for (int seq = 1; seq <= 30; ++seq) {
    feed.emit(seq);
    // Each client's write completes on its own schedule; none is lost.
    if (seq % 3 == 0) {
      a.writeDone(true);
    }
    if (seq % 5 == 0) {
      b.writeDone(true);
    }
  }
  for (int i = 0; i < 40; ++i) {
    a.writeDone(true);
    b.writeDone(true);
  }
  std::vector<int> expected(30);
  for (int i = 0; i < 30; ++i) {
    expected[i] = i + 1;
  }
  EXPECT_EQ(a.wire.writes(), expected);
  EXPECT_EQ(b.wire.writes(), expected);
  EXPECT_EQ(a.wire.violations, 0);
  EXPECT_EQ(b.wire.violations, 0);
  feed.dropBarrier();
  a.reactor->OnCancel();
  b.reactor->OnCancel();
  a.reactor->OnDone();
  EXPECT_TRUE(feed.driver.slot.hasCallback()); // b still subscribed
  b.reactor->OnDone();
  EXPECT_FALSE(feed.driver.slot.hasCallback());
  EXPECT_EQ(feed.driver.installs.load(), 1);
  EXPECT_EQ(feed.driver.clears.load(), 1);
}

// Invariant 4: a client whose writes never complete loses only its own
// samples.
TEST(SensorStreamReactor, StalledClientDoesNotCostAnotherOneSamples) {
  Feed feed;
  Client stalled(feed, 2);
  Client healthy(feed, 64);
  feed.armBarrier();
  for (int seq = 1; seq <= 40; ++seq) {
    feed.emit(seq);
    healthy.writeDone(true); // keeps up; `stalled` never completes a write
  }
  healthy.writeDone(true);
  std::vector<int> all(40);
  for (int i = 0; i < 40; ++i) {
    all[i] = i + 1;
  }
  EXPECT_EQ(healthy.wire.writes(), all);
  // The stalled one wrote the first sample and then queued only the newest.
  EXPECT_EQ(stalled.wire.writes(), std::vector<int>{1});
  stalled.writeDone(true);
  stalled.writeDone(true);
  EXPECT_EQ(stalled.wire.writes(), (std::vector<int>{1, 39, 40}));
  EXPECT_EQ(stalled.conversions.load(), 3); // dropped samples not converted
  EXPECT_EQ(healthy.conversions.load(), 40);
  feed.dropBarrier();
  for (Client *c : {&stalled, &healthy}) {
    c->reactor->OnCancel();
    c->reactor->OnDone();
  }
}

// Invariant 2: the driver's thread only publishes; the protobuf conversion and
// StartWrite run on another thread.
TEST(SensorStreamReactor, ConversionAndWriteDoNotRunOnTheDriverThread) {
  Harness h(4);
  const auto driver_thread = std::this_thread::get_id();
  h.emit(1);
  EXPECT_EQ(h.conversions.load(), 1);
  EXPECT_NE(h.convert_thread.load(), driver_thread);
  EXPECT_NE(h.convert_thread.load(), std::thread::id{});
  EXPECT_NE(h.wire.write_thread, driver_thread);
  EXPECT_NE(h.wire.write_thread, std::thread::id{});
  h.writeDone(true);
  h.done();
}

TEST(SensorStreamReactor, PublishingDoesNotWaitForAStalledHook) {
  // A client whose hook is stuck inside StartWrite (here: blocked in a
  // convert) must not hold up the driver's thread.
  std::mutex gate_m;
  std::condition_variable gate_cv;
  bool open = false;
  bool entered = false;

  Feed feed;
  Wire wire;
  msensor::ClientLimit limit{1};
  ASSERT_TRUE(limit.tryAcquire());
  g_wire = &wire;
  auto *reactor = new Reactor(
      feed.topic.subscribe(4),
      [&](const Sample &, Response &) {
        std::unique_lock lock(gate_m);
        entered = true;
        gate_cv.notify_all();
        gate_cv.wait(lock, [&] { return open; });
      },
      limit, "test");
  g_wire = nullptr;

  feed.driver.emit(1); // the dispatcher is now stuck converting it
  {
    std::unique_lock lock(gate_m);
    ASSERT_TRUE(gate_cv.wait_for(lock, std::chrono::seconds(10),
                                 [&] { return entered; }));
  }
  const auto start = std::chrono::steady_clock::now();
  for (int seq = 2; seq <= 2000; ++seq) {
    feed.driver.emit(seq);
  }
  const auto took = std::chrono::steady_clock::now() - start;
  EXPECT_LT(took, std::chrono::seconds(2)); // returned while the hook was stuck
  {
    std::lock_guard lock(gate_m);
    open = true;
  }
  gate_cv.notify_all();
  reactor->OnCancel();
  reactor->OnDone();
}

// ---- Client limit ----

TEST(ClientLimit, GrantsAtMostMaxAndReleasesSlots) {
  msensor::ClientLimit limit(2);
  EXPECT_TRUE(limit.tryAcquire());
  EXPECT_TRUE(limit.tryAcquire());
  EXPECT_FALSE(limit.tryAcquire());
  EXPECT_EQ(limit.active(), 2U);
  limit.release();
  EXPECT_TRUE(limit.tryAcquire());
  EXPECT_FALSE(limit.tryAcquire());
}

TEST(ClientLimit, ConcurrentAcquiresNeverExceedTheMaximum) {
  msensor::ClientLimit limit(5);
  std::atomic<int> granted{0};
  std::vector<std::thread> threads;
  for (int i = 0; i < 16; ++i) {
    threads.emplace_back([&] {
      if (limit.tryAcquire()) {
        granted.fetch_add(1);
      }
    });
  }
  for (auto &t : threads) {
    t.join();
  }
  EXPECT_EQ(granted.load(), 5);
  EXPECT_EQ(limit.active(), 5U);
}

TEST(ClientLimit, DefaultIsEight) {
  EXPECT_EQ(msensor::kDefaultMaxClientsPerStream, 8U);
}

// ---- Drop logging ----

namespace {
struct LogCapture {
  std::mutex m;
  std::vector<std::string> lines;
  LogCapture() {
    msensor::log::setSink([this](msensor::log::Level, std::string_view text) {
      std::lock_guard lock(m);
      lines.emplace_back(text);
    });
  }
  ~LogCapture() { msensor::log::setSink({}); }
  int count(const std::string &needle) {
    std::lock_guard lock(m);
    int n = 0;
    for (const auto &line : lines) {
      n += line.find(needle) != std::string::npos;
    }
    return n;
  }
};
} // namespace

TEST(SensorStreamReactor, DropsAreLoggedAtMostOncePerSecondAndTotalAtTheEnd) {
  LogCapture log;
  Harness h(1);
  h.emit(1); // in flight
  for (int seq = 2; seq <= 60; ++seq) {
    h.emit(seq); // 58 drops, all within a fraction of a second
  }
  EXPECT_EQ(log.count("slow client, dropped"), 1);

  std::this_thread::sleep_for(std::chrono::milliseconds(1100));
  h.emit(61); // one more drop, a second later: reported again
  EXPECT_EQ(log.count("slow client, dropped"), 2);
  for (int seq = 62; seq <= 70; ++seq) {
    h.emit(seq); // within the new window: silent
  }
  EXPECT_EQ(log.count("slow client, dropped"), 2);

  h.reactor->OnCancel();
  h.done();
  EXPECT_EQ(log.count("stream dropped 68 samples for a slow client"), 1);
}

TEST(SensorStreamReactor, NoDropsNoDropLogs) {
  LogCapture log;
  Harness h(8);
  for (int seq = 1; seq <= 5; ++seq) {
    h.emit(seq);
  }
  h.reactor->OnCancel();
  h.done();
  EXPECT_EQ(log.count("dropped"), 0);
}
