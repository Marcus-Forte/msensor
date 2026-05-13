#pragma once

#include <cstdint>

namespace timing {

/**
 * @brief Get epoch time in microseconds.
 *
 * @return uint64_t time in microseconds.
 */
uint64_t getEpochTimeUs();

/**
 * @brief Get epoch time in nanoseconds.
 *
 * @return uint64_t time in nanoseconds.
 */
uint64_t getEpochTimeNs();

/**
 * @brief Get current time in microsseconds. Uses system clock.
 *
 * @return uint64_t time in microsseconds.
 */
uint64_t getNowUs();

/**
 * @brief Get current time in nanoseconds. Uses system clock.
 *
 * @return uint64_t time in nanoseconds.
 */
uint64_t getNowNs();
} // namespace timing