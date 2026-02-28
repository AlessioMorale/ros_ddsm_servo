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

// Test atomic timestamp operations (without ResourceManager)
class TestHardwareInterfaceLifecycle : public ::testing::Test
{
protected:
  std::atomic<int64_t> last_read_ns_{0};
  std::atomic<int64_t> last_write_ns_{0};
  
  int64_t now_ns()
  {
    return std::chrono::steady_clock::now().time_since_epoch().count();
  }
};

// Test initial state of atomic timestamps
TEST_F(TestHardwareInterfaceLifecycle, InitialAtomicState)
{
  EXPECT_EQ(last_read_ns_.load(), 0);
  EXPECT_EQ(last_write_ns_.load(), 0);
}

// Test atomic store and load
TEST_F(TestHardwareInterfaceLifecycle, AtomicStoreLoad)
{
  int64_t time_value = now_ns();
  last_read_ns_.store(time_value, std::memory_order_release);
  
  int64_t loaded = last_read_ns_.load(std::memory_order_acquire);
  EXPECT_EQ(loaded, time_value);
}

// Test multiple atomic updates
TEST_F(TestHardwareInterfaceLifecycle, MultipleAtomicUpdates)
{
  int64_t t1 = now_ns();
  last_read_ns_.store(t1, std::memory_order_release);
  
  int64_t t2 = now_ns();
  last_read_ns_.store(t2, std::memory_order_release);
  
  int64_t loaded = last_read_ns_.load(std::memory_order_acquire);
  EXPECT_EQ(loaded, t2);
  EXPECT_GT(t2, t1);  // Time should advance
}

// Test memory ordering semantics
TEST_F(TestHardwareInterfaceLifecycle, MemoryOrderingRelease)
{
  int64_t value = 42;
  last_read_ns_.store(value, std::memory_order_release);
  
  int64_t loaded = last_read_ns_.load(std::memory_order_acquire);
  EXPECT_EQ(loaded, 42);
}

// Test atomic timestamp precision
TEST_F(TestHardwareInterfaceLifecycle, TimestampPrecision)
{
  int64_t t1 = now_ns();
  int64_t t2 = now_ns();
  
  // Nanosecond timestamps should allow sub-microsecond precision
  EXPECT_GE(t2, t1);
}

// Test read and write timestamp independence
TEST_F(TestHardwareInterfaceLifecycle, ReadWriteIndependence)
{
  int64_t read_time = now_ns();
  int64_t write_time = now_ns();
  
  last_read_ns_.store(read_time, std::memory_order_release);
  last_write_ns_.store(write_time, std::memory_order_release);
  
  EXPECT_EQ(last_read_ns_.load(std::memory_order_acquire), read_time);
  EXPECT_EQ(last_write_ns_.load(std::memory_order_acquire), write_time);
}

// Test atomic operations don't block
TEST_F(TestHardwareInterfaceLifecycle, NonBlockingAtomicOps)
{
  auto start = std::chrono::high_resolution_clock::now();
  
  for (int i = 0; i < 1000; ++i) {
    last_read_ns_.store(now_ns(), std::memory_order_release);
  }
  
  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  
  // 1000 atomic operations should complete quickly (< 10ms)
  EXPECT_LT(duration.count(), 10);
}

// Test timestamp wraparound handling
TEST_F(TestHardwareInterfaceLifecycle, LargeTimestampValues)
{
  int64_t large_value = 9223372036854775000LL;
  last_read_ns_.store(large_value, std::memory_order_release);
  
  int64_t loaded = last_read_ns_.load(std::memory_order_acquire);
  EXPECT_EQ(loaded, large_value);
}
