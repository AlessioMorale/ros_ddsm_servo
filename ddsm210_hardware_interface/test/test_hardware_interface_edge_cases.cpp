// Copyright (c) 2024, Alessio Morale
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <limits>

// Test edge cases for atomic timestamp operations
class TestHardwareInterfaceEdgeCases : public ::testing::Test
{
protected:
  std::atomic<int64_t> timestamp_ns_{0};
  double communication_timeout_{1.0};
  
  int64_t now_ns()
  {
    return std::chrono::steady_clock::now().time_since_epoch().count();
  }
};

// Test zero initialization
TEST_F(TestHardwareInterfaceEdgeCases, ZeroInitialization)
{
  EXPECT_EQ(timestamp_ns_.load(), 0);
}

// Test maximum int64_t value
TEST_F(TestHardwareInterfaceEdgeCases, MaxInt64Value)
{
  int64_t max_val = std::numeric_limits<int64_t>::max();
  timestamp_ns_.store(max_val, std::memory_order_release);
  
  int64_t loaded = timestamp_ns_.load(std::memory_order_acquire);
  EXPECT_EQ(loaded, max_val);
}

// Test minimum int64_t value
TEST_F(TestHardwareInterfaceEdgeCases, MinInt64Value)
{
  int64_t min_val = std::numeric_limits<int64_t>::min();
  timestamp_ns_.store(min_val, std::memory_order_release);
  
  int64_t loaded = timestamp_ns_.load(std::memory_order_acquire);
  EXPECT_EQ(loaded, min_val);
}

// Test value wrapping near maximum
TEST_F(TestHardwareInterfaceEdgeCases, NearMaximumValue)
{
  int64_t near_max = std::numeric_limits<int64_t>::max() - 1000;
  timestamp_ns_.store(near_max, std::memory_order_release);
  
  int64_t loaded = timestamp_ns_.load(std::memory_order_acquire);
  EXPECT_EQ(loaded, near_max);
}

// Test timeout calculation with zero timestamp
TEST_F(TestHardwareInterfaceEdgeCases, TimeoutFromZeroTimestamp)
{
  timestamp_ns_.store(0, std::memory_order_release);
  
  int64_t current = now_ns();
  double timeout_s = static_cast<double>(current - 0) / 1e9;
  
  // Should be very large (seconds since epoch)
  EXPECT_GT(timeout_s, communication_timeout_);
}

// Test timeout calculation with negative timestamp
TEST_F(TestHardwareInterfaceEdgeCases, TimeoutFromNegativeTimestamp)
{
  int64_t negative_ts = -1000000LL;
  timestamp_ns_.store(negative_ts, std::memory_order_release);
  
  int64_t current = now_ns();
  double timeout_s = static_cast<double>(current - negative_ts) / 1e9;
  
  // Should be very large
  EXPECT_GT(timeout_s, communication_timeout_);
}

// Test repeated stores to same value
TEST_F(TestHardwareInterfaceEdgeCases, RepeatedSameValueStore)
{
  int64_t value = 12345;
  
  for (int i = 0; i < 1000; ++i) {
    timestamp_ns_.store(value, std::memory_order_release);
  }
  
  int64_t loaded = timestamp_ns_.load(std::memory_order_acquire);
  EXPECT_EQ(loaded, value);
}

// Test alternating stores and loads
TEST_F(TestHardwareInterfaceEdgeCases, AlternatingStoresAndLoads)
{
  for (int i = 0; i < 100; ++i) {
    int64_t value = i;
    timestamp_ns_.store(value, std::memory_order_release);
    int64_t loaded = timestamp_ns_.load(std::memory_order_acquire);
    EXPECT_EQ(loaded, value);
  }
}

// Test timeout boundary with exact timeout value
TEST_F(TestHardwareInterfaceEdgeCases, ExactTimeoutBoundary)
{
  communication_timeout_ = 0.1;  // 100ms
  
  int64_t start = now_ns();
  timestamp_ns_.store(start, std::memory_order_release);
  
  // Check timeout immediately (should be ~0)
  int64_t current = now_ns();
  double elapsed = static_cast<double>(current - start) / 1e9;
  
  EXPECT_LT(elapsed, communication_timeout_);
}

// Test all memory orderings work correctly
TEST_F(TestHardwareInterfaceEdgeCases, AllMemoryOrderings)
{
  int64_t value = 42;
  
  // Test relaxed
  timestamp_ns_.store(value, std::memory_order_relaxed);
  EXPECT_EQ(timestamp_ns_.load(std::memory_order_relaxed), value);
  
  // Test release/acquire
  timestamp_ns_.store(value + 1, std::memory_order_release);
  EXPECT_EQ(timestamp_ns_.load(std::memory_order_acquire), value + 1);
  
  // Test seq_cst
  timestamp_ns_.store(value + 2, std::memory_order_seq_cst);
  EXPECT_EQ(timestamp_ns_.load(std::memory_order_seq_cst), value + 2);
}

// Test atomic compare_exchange_strong
TEST_F(TestHardwareInterfaceEdgeCases, AtomicCompareExchange)
{
  int64_t initial = 100;
  timestamp_ns_.store(initial, std::memory_order_release);
  
  int64_t expected = initial;
  int64_t desired = 200;
  
  bool result = timestamp_ns_.compare_exchange_strong(
    expected, desired, std::memory_order_release, std::memory_order_acquire);
  
  EXPECT_TRUE(result);
  EXPECT_EQ(timestamp_ns_.load(std::memory_order_acquire), desired);
}

// Test atomic exchange operation
TEST_F(TestHardwareInterfaceEdgeCases, AtomicExchange)
{
  int64_t initial = 100;
  timestamp_ns_.store(initial, std::memory_order_release);
  
  int64_t new_value = 200;
  int64_t old_value = timestamp_ns_.exchange(new_value, std::memory_order_release);
  
  EXPECT_EQ(old_value, initial);
  EXPECT_EQ(timestamp_ns_.load(std::memory_order_acquire), new_value);
}

// Test is_lock_free property
TEST_F(TestHardwareInterfaceEdgeCases, IsLockFree)
{
  // On most platforms, int64_t atomics should be lock-free
  // This is important for the safety monitor thread
  EXPECT_TRUE(std::atomic<int64_t>::is_always_lock_free || timestamp_ns_.is_lock_free());
}
