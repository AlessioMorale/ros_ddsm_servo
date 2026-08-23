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
#include <thread>

// Test integration of atomic operations with timeout calculation
class TestHardwareInterfaceIntegration : public ::testing::Test
{
protected:
  std::atomic<int64_t> last_read_ns_{0};
  std::atomic<int64_t> last_write_ns_{0};
  double communication_timeout_{1.0};
  
  int64_t now_ns()
  {
    return std::chrono::steady_clock::now().time_since_epoch().count();
  }
};

// Test read/write timestamp updates work together
TEST_F(TestHardwareInterfaceIntegration, ReadWriteTimestampIntegration)
{
  int64_t t1 = now_ns();
  last_read_ns_.store(t1, std::memory_order_release);
  
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  
  int64_t t2 = now_ns();
  last_write_ns_.store(t2, std::memory_order_release);
  
  // Both timestamps should be stored
  int64_t read_ts = last_read_ns_.load(std::memory_order_acquire);
  int64_t write_ts = last_write_ns_.load(std::memory_order_acquire);
  
  EXPECT_EQ(read_ts, t1);
  EXPECT_EQ(write_ts, t2);
  EXPECT_GT(write_ts, read_ts);
}

// Test timeout calculation for both read and write
TEST_F(TestHardwareInterfaceIntegration, DualTimeoutDetection)
{
  int64_t old_time = now_ns() - static_cast<int64_t>(2e9);  // 2 seconds ago
  
  last_read_ns_.store(old_time, std::memory_order_release);
  last_write_ns_.store(old_time, std::memory_order_release);
  
  int64_t current_time = now_ns();
  
  double read_timeout_s = static_cast<double>(current_time - old_time) / 1e9;
  double write_timeout_s = static_cast<double>(current_time - old_time) / 1e9;
  
  // Both should exceed 1s timeout
  EXPECT_GT(read_timeout_s, communication_timeout_);
  EXPECT_GT(write_timeout_s, communication_timeout_);
}

// Test alternating read/write updates
TEST_F(TestHardwareInterfaceIntegration, AlternatingReadWriteUpdates)
{
  for (int i = 0; i < 10; ++i) {
    int64_t read_time = now_ns();
    last_read_ns_.store(read_time, std::memory_order_release);
    
    std::this_thread::yield();
    
    int64_t write_time = now_ns();
    last_write_ns_.store(write_time, std::memory_order_release);
    
    // Both should be updated
    EXPECT_GE(last_read_ns_.load(std::memory_order_acquire), read_time);
    EXPECT_GE(last_write_ns_.load(std::memory_order_acquire), write_time);
  }
}

// Test independent timeout checking
TEST_F(TestHardwareInterfaceIntegration, IndependentTimeoutChecking)
{
  int64_t start = now_ns();
  
  // Set reads to recent
  last_read_ns_.store(start, std::memory_order_release);
  
  // Keep writes old
  last_write_ns_.store(start - static_cast<int64_t>(2e9), std::memory_order_release);  // 2s old
  
  int64_t now = now_ns();
  
  double read_timeout = static_cast<double>(now - last_read_ns_.load(std::memory_order_acquire)) / 1e9;
  double write_timeout = static_cast<double>(now - last_write_ns_.load(std::memory_order_acquire)) / 1e9;
  
  // Read should be fresh, write should be timed out
  EXPECT_LT(read_timeout, communication_timeout_);
  EXPECT_GT(write_timeout, communication_timeout_);
}

// Test recovery from timeout
TEST_F(TestHardwareInterfaceIntegration, TimeoutRecovery)
{
  // Initially set old timestamp
  last_read_ns_.store(now_ns() - static_cast<int64_t>(2e9), std::memory_order_release);
  
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
  // Update to current time (recovery)
  last_read_ns_.store(now_ns(), std::memory_order_release);
  
  int64_t current = now_ns();
  int64_t last_read = last_read_ns_.load(std::memory_order_acquire);
  
  double elapsed = static_cast<double>(current - last_read) / 1e9;
  
  // Should NOT be timed out anymore (or just barely)
  EXPECT_LT(elapsed, communication_timeout_ / 2);  // Well below timeout
}


