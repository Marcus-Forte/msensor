#include "msensor/log/log.hh"

#include <gtest/gtest.h>

#include <atomic>
#include <format>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace mlog = msensor::log;

namespace {

struct Record {
  mlog::Level level;
  std::string text;
};

// Restores the global level and sink when a test ends.
class LogTest : public ::testing::Test {
protected:
  void SetUp() override {
    saved_level_ = mlog::level();
    mlog::setSink([this](mlog::Level l, std::string_view m) {
      records_.push_back({l, std::string(m)});
    });
    mlog::setLevel(mlog::Level::info);
  }
  void TearDown() override {
    mlog::setSink({});
    mlog::setLevel(saved_level_);
  }

  void logOnePerLevel() {
    mlog::debug("d");
    mlog::info("i");
    mlog::warn("w");
    mlog::error("e");
  }

  std::vector<Record> records_;
  mlog::Level saved_level_{};
};

// Counts how often it is formatted.
struct Counted {
  static inline std::atomic<int> formatted{0};
};

} // namespace

template <> struct std::formatter<Counted> : std::formatter<int> {
  auto format(const Counted &, std::format_context &ctx) const {
    ++Counted::formatted;
    return std::formatter<int>::format(7, ctx);
  }
};

TEST_F(LogTest, DefaultThresholdIsInfoAfterSetup) {
  EXPECT_EQ(mlog::level(), mlog::Level::info);
}

TEST_F(LogTest, ThresholdFiltersEachLevel) {
  struct Case {
    mlog::Level threshold;
    size_t expected;
  };
  for (const auto [threshold, expected] :
       {Case{mlog::Level::debug, 4}, Case{mlog::Level::info, 3},
        Case{mlog::Level::warn, 2}, Case{mlog::Level::error, 1},
        Case{mlog::Level::off, 0}}) {
    records_.clear();
    mlog::setLevel(threshold);
    EXPECT_EQ(mlog::level(), threshold);
    logOnePerLevel();
    ASSERT_EQ(records_.size(), expected)
        << "threshold " << mlog::levelName(threshold);
    // The surviving messages are the most severe ones, in order.
    const mlog::Level all[] = {mlog::Level::debug, mlog::Level::info,
                              mlog::Level::warn, mlog::Level::error};
    for (size_t i = 0; i < expected; ++i) {
      EXPECT_EQ(records_[i].level, all[4 - expected + i]);
    }
  }
}

TEST_F(LogTest, SinkReceivesLevelAndFormattedText) {
  mlog::warn("value {} and {:>4}|{:.1f}", 42, "ab", 2.25);
  ASSERT_EQ(records_.size(), 1u);
  EXPECT_EQ(records_[0].level, mlog::Level::warn);
  EXPECT_EQ(records_[0].text, "value 42 and   ab|2.2");
}

TEST_F(LogTest, RestoringDefaultSinkStopsCustomDelivery) {
  mlog::info("first");
  mlog::setSink({});
  testing::internal::CaptureStdout();
  mlog::info("to default");
  const std::string out = testing::internal::GetCapturedStdout();
  EXPECT_EQ(out, "[info] to default\n");
  EXPECT_EQ(records_.size(), 1u);
}

TEST_F(LogTest, DefaultSinkWritesWarningsToStderr) {
  mlog::setSink({});
  testing::internal::CaptureStderr();
  testing::internal::CaptureStdout();
  mlog::warn("careful");
  mlog::error("broken {}", 1);
  mlog::info("fine");
  const std::string out = testing::internal::GetCapturedStdout();
  const std::string err = testing::internal::GetCapturedStderr();
  EXPECT_EQ(out, "[info] fine\n");
  EXPECT_EQ(err, "[warn] careful\n[error] broken 1\n");
}

TEST_F(LogTest, BelowThresholdMessageIsNotFormatted) {
  Counted::formatted = 0;
  mlog::debug("{}", Counted{});
  EXPECT_EQ(Counted::formatted.load(), 0);
  EXPECT_TRUE(records_.empty());

  mlog::setLevel(mlog::Level::off);
  mlog::error("{}", Counted{});
  EXPECT_EQ(Counted::formatted.load(), 0);

  mlog::setLevel(mlog::Level::debug);
  mlog::debug("{}", Counted{});
  EXPECT_EQ(Counted::formatted.load(), 1);
  ASSERT_EQ(records_.size(), 1u);
  EXPECT_EQ(records_[0].text, "7");
}

TEST_F(LogTest, ConcurrentMessagesAreDeliveredWholeAndOnce) {
  constexpr int kThreads = 8;
  constexpr int kPerThread = 500;
  mlog::setLevel(mlog::Level::debug);
  // The sink is invoked one at a time, so this unsynchronised check would
  // trip TSan if serialisation were broken.
  int in_sink = 0;
  int overlaps = 0;
  records_.clear();
  mlog::setSink([&](mlog::Level l, std::string_view m) {
    if (++in_sink != 1) {
      ++overlaps;
    }
    records_.push_back({l, std::string(m)});
    --in_sink;
  });

  std::vector<std::thread> threads;
  for (int t = 0; t < kThreads; ++t) {
    threads.emplace_back([t] {
      for (int i = 0; i < kPerThread; ++i) {
        mlog::info("thread={} seq={} end", t, i);
      }
    });
  }
  for (auto &th : threads) {
    th.join();
  }

  EXPECT_EQ(overlaps, 0);
  ASSERT_EQ(records_.size(), static_cast<size_t>(kThreads * kPerThread));
  std::set<std::string> seen;
  for (const auto &r : records_) {
    EXPECT_EQ(r.level, mlog::Level::info);
    EXPECT_TRUE(seen.insert(r.text).second) << "duplicate: " << r.text;
  }
  for (int t = 0; t < kThreads; ++t) {
    for (int i = 0; i < kPerThread; ++i) {
      EXPECT_TRUE(seen.contains(std::format("thread={} seq={} end", t, i)));
    }
  }
}

TEST_F(LogTest, LevelCanChangeWhileOtherThreadsLog) {
  std::atomic<bool> stop{false};
  std::atomic<int> delivered{0};
  mlog::setSink([&](mlog::Level, std::string_view) { ++delivered; });
  std::thread writer([&] {
    while (!stop) {
      mlog::info("x");
    }
  });
  for (int i = 0; i < 200; ++i) {
    mlog::setLevel(i % 2 ? mlog::Level::off : mlog::Level::debug);
  }
  stop = true;
  writer.join();
  SUCCEED();
}

TEST(LogParse, AcceptsAllLevelsCaseInsensitively) {
  EXPECT_EQ(mlog::parseLevel("debug"), mlog::Level::debug);
  EXPECT_EQ(mlog::parseLevel("info"), mlog::Level::info);
  EXPECT_EQ(mlog::parseLevel("warn"), mlog::Level::warn);
  EXPECT_EQ(mlog::parseLevel("error"), mlog::Level::error);
  EXPECT_EQ(mlog::parseLevel("off"), mlog::Level::off);
  EXPECT_EQ(mlog::parseLevel("DEBUG"), mlog::Level::debug);
  EXPECT_EQ(mlog::parseLevel("Warn"), mlog::Level::warn);
  EXPECT_EQ(mlog::parseLevel("OfF"), mlog::Level::off);
}

TEST(LogParse, RejectsUnknownStrings) {
  EXPECT_FALSE(mlog::parseLevel("").has_value());
  EXPECT_FALSE(mlog::parseLevel("verbose").has_value());
  EXPECT_FALSE(mlog::parseLevel("warning").has_value());
  EXPECT_FALSE(mlog::parseLevel(" info").has_value());
  EXPECT_FALSE(mlog::parseLevel("info ").has_value());
}
