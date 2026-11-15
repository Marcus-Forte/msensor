// Unit tests of SensorStreamReactor's bounded-FIFO write logic, driven
// deterministically through a fake reactor base instead of a gRPC server.

#include "msensor/interface/CallbackSlot.hh"
#include "sensor_stream.hh"

#include <gtest/gtest.h>

#include <atomic>
#include <condition_variable>
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

  Reactor::Install install() {
    return [this](Reactor::Callback callback) {
      (callback ? installs : clears).fetch_add(1);
      slot.setCallback(std::move(callback));
    };
  }

  void emit(int seq) { slot.emit(Sample{seq, std::vector<int>(64, seq)}); }
};

struct Harness {
  Wire wire;
  Driver driver;
  std::atomic<bool> in_use{true}; // set by the service before construction
  std::atomic<int> conversions{0};
  Reactor *reactor = nullptr;

  explicit Harness(size_t capacity = 1) {
    g_wire = &wire;
    reactor = new Reactor(
        driver.install(),
        [this](const Sample &sample, Response &out) {
          conversions.fetch_add(1);
          out.seq = sample.seq;
        },
        in_use, "test", capacity);
    g_wire = nullptr;
  }

  /// gRPC reports the outstanding write as finished.
  void writeDone(bool ok) {
    reactor->completeWrite();
    reactor->OnWriteDone(ok);
  }
};

} // namespace

TEST(SensorStreamReactor, InstallsCallbackAndWritesFirstSample) {
  Harness h;
  EXPECT_EQ(h.driver.installs.load(), 1);
  h.driver.emit(1);
  EXPECT_EQ(h.wire.writes(), std::vector<int>{1});
  EXPECT_EQ(h.conversions.load(), 1);
  h.writeDone(true);
  h.reactor->OnCancel();
  h.reactor->OnDone();
  EXPECT_EQ(h.wire.violations, 0);
}

TEST(SensorStreamReactor, KeepsOnlyTheLatestSampleWhileWriting) {
  Harness h(1);
  h.driver.emit(1);
  h.driver.emit(2);
  h.driver.emit(3);
  h.driver.emit(4);
  // Samples 2 and 3 were superseded and never converted.
  EXPECT_EQ(h.conversions.load(), 1);
  h.writeDone(true);
  EXPECT_EQ(h.wire.writes(), (std::vector<int>{1, 4}));
  EXPECT_EQ(h.conversions.load(), 2);
  // Nothing pending: the next completion starts no write.
  h.writeDone(true);
  EXPECT_EQ(h.wire.writes(), (std::vector<int>{1, 4}));
  h.driver.emit(5);
  EXPECT_EQ(h.wire.writes(), (std::vector<int>{1, 4, 5}));
  h.writeDone(true);
  h.reactor->OnCancel();
  h.reactor->OnDone();
  EXPECT_EQ(h.wire.violations, 0);
}

TEST(SensorStreamReactor, ZeroCapacityBehavesAsOne) {
  Harness h(0);
  h.driver.emit(1);
  h.driver.emit(2);
  h.driver.emit(3);
  h.writeDone(true);
  EXPECT_EQ(h.wire.writes(), (std::vector<int>{1, 3}));
  h.reactor->OnCancel();
  h.writeDone(true);
  h.reactor->OnDone();
}

TEST(SensorStreamReactor, QueuePreservesFifoOrder) {
  Harness h(8);
  for (int seq = 1; seq <= 5; ++seq) {
    h.driver.emit(seq);
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
  h.reactor->OnDone();
  EXPECT_EQ(h.wire.violations, 0);
}

TEST(SensorStreamReactor, OverflowDropsTheOldestAndKeepsTheNewestInOrder) {
  Harness h(3);
  h.driver.emit(1); // written at once
  for (int seq = 2; seq <= 9; ++seq) {
    h.driver.emit(seq); // queue holds the newest 3: 7, 8, 9
  }
  for (int i = 0; i < 4; ++i) {
    h.writeDone(true);
  }
  EXPECT_EQ(h.wire.writes(), (std::vector<int>{1, 7, 8, 9}));
  EXPECT_EQ(h.conversions.load(), 4); // dropped samples were never converted
  h.reactor->OnCancel();
  h.reactor->OnDone();
  EXPECT_EQ(h.wire.violations, 0);
}

TEST(SensorStreamReactor, CancelWithQueuedSamplesWritesNothingMore) {
  Harness h(4);
  h.driver.emit(1);
  h.driver.emit(2);
  h.driver.emit(3); // queued
  h.reactor->OnCancel();
  EXPECT_EQ(h.wire.finishes, 1);
  h.writeDone(true); // late success: queue was discarded
  h.driver.emit(4);
  EXPECT_EQ(h.wire.writes(), std::vector<int>{1});
  EXPECT_EQ(h.conversions.load(), 1);
  EXPECT_EQ(h.wire.violations, 0);
  h.reactor->OnDone();
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
  h.driver.emit(1);
  h.driver.emit(2); // pending
  h.reactor->OnCancel();
  EXPECT_EQ(h.wire.finishes, 1);
  h.writeDone(true);
  EXPECT_EQ(h.wire.writes(), std::vector<int>{1});
  h.driver.emit(3);
  EXPECT_EQ(h.wire.writes(), std::vector<int>{1});
  EXPECT_EQ(h.wire.finishes, 1);
  EXPECT_EQ(h.wire.violations, 0);
  h.reactor->OnDone();
}

TEST(SensorStreamReactor, FailedWriteFinishesOnceAndStopsWriting) {
  Harness h;
  h.driver.emit(1);
  h.driver.emit(2); // pending
  h.writeDone(false);
  EXPECT_EQ(h.wire.finishes, 1);
  h.driver.emit(3);
  h.reactor->OnCancel(); // may still arrive
  EXPECT_EQ(h.wire.finishes, 1);
  EXPECT_EQ(h.wire.writes(), std::vector<int>{1});
  EXPECT_EQ(h.wire.violations, 0);
  h.reactor->OnDone();
}

TEST(SensorStreamReactor, OnDoneClearsCallbackReleasesStreamAndDeletes) {
  Harness h;
  h.driver.emit(1);
  h.writeDone(true);
  h.reactor->OnCancel();
  h.reactor->OnDone(); // deletes the reactor
  EXPECT_EQ(h.driver.clears.load(), 1);
  EXPECT_FALSE(h.driver.slot.hasCallback());
  EXPECT_FALSE(h.in_use.load());
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
    h.reactor->OnDone();

    const auto writes = h.wire.writes();
    ASSERT_GT(writes.size(), 200U);
    for (size_t i = 1; i < writes.size(); ++i) {
      ASSERT_LT(writes[i - 1], writes[i]) << "samples reordered";
    }
    EXPECT_EQ(h.wire.finishes, 1);
    EXPECT_EQ(h.wire.violations, 0);
    EXPECT_FALSE(h.in_use.load());
  }
}
