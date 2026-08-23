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
#include <thread>

// Test safety monitor timeout detection logic
class TestHardwareInterfaceSafety : public ::testing::Test
{
protected:
  std::atomic<int64_t> last_read_time_ns_{0};
  std::atomic<bool> stop_safety_monitor_{false};
  
  double communication_timeout_{1.0};  // seconds
  
  int64_t now_ns()
  {
    return std::chrono::steady_clock::now().time_since_epoch().count();
  }
  
  bool check_timeout()
  {
    int64_t current_time_ns = now_ns();
    int64_t last_read_ns = last_read_time_ns_.load(std::memory_order_acquire);
    
    double read_timeout_s = static_cast<double>(current_time_ns - last_read_ns) / 1e9;
    return read_timeout_s > communication_timeout_;
  }
};

// Test timeout detection doesn't trigger prematurely
TEST_F(TestHardwareInterfaceSafety, TimeoutNotTriggeredWithinLimit)
{
  int64_t start_time = now_ns();
  last_read_time_ns_.store(start_time, std::memory_order_release);
  
  // Wait 100ms (well below 1s timeout)
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
  // Timeout should NOT be detected
  EXPECT_FALSE(check_timeout());
}

// Test timeout detection triggers when exceeded
TEST_F(TestHardwareInterfaceSafety, TimeoutTriggeredWhenExceeded)
{
  communication_timeout_ = 0.1;  // 100ms timeout
  
  int64_t start_time = now_ns();
  last_read_time_ns_.store(start_time, std::memory_order_release);
  
  // Wait 200ms (exceeds 100ms timeout)
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  
  // Timeout SHOULD be detected
  EXPECT_TRUE(check_timeout());
}

// Test timeout detection boundary
TEST_F(TestHardwareInterfaceSafety, TimeoutBoundaryDetection)
{
  communication_timeout_ = 0.15;  // 150ms timeout
  
  int64_t start_time = now_ns();
  last_read_time_ns_.store(start_time, std::memory_order_release);
  
  // Wait 160ms (slightly exceeds 150ms timeout)
  std::this_thread::sleep_for(std::chrono::milliseconds(160));
  
  // Timeout should be detected
  EXPECT_TRUE(check_timeout());
}

// Test multiple read updates reset timeout
TEST_F(TestHardwareInterfaceSafety, MultipleUpdatesResetTimeout)
{
  communication_timeout_ = 0.2;  // 200ms timeout
  
  for (int i = 0; i < 5; ++i) {
    last_read_time_ns_.store(now_ns(), std::memory_order_release);
    
    // Check timeout within each period
    EXPECT_FALSE(check_timeout());
    
    // Wait 100ms
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

// Test timeout with zero initial value
TEST_F(TestHardwareInterfaceSafety, TimeoutFromZeroInitialValue)
{
  communication_timeout_ = 0.2;  // 200ms
  
  // Initially, last_read_time_ns_ is 0
  // This should be treated as very old timestamp, triggering timeout
  // Unless we have special handling
  
  int64_t current_time = now_ns();
  double elapsed_s = static_cast<double>(current_time - 0) / 1e9;
  
  // Elapsed should be very large (seconds since epoch)
  EXPECT_GT(elapsed_s, communication_timeout_);
}

// Test safety flag atomicity
TEST_F(TestHardwareInterfaceSafety, SafetyFlagAtomicity)
{
  std::atomic<bool> emergency_stopped{false};
  std::atomic<int> updates{0};
  std::atomic<bool> reader_running{true};
  
  auto reader = [&emergency_stopped, &updates, &reader_running]() {
    while (reader_running.load()) {
      bool stopped = emergency_stopped.load(std::memory_order_acquire);
      if (stopped) {
        updates.fetch_add(1);
        break;  // Exit once flag is seen
      }
      std::this_thread::yield();
    }
  };
  
  auto writer = [&emergency_stopped]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    emergency_stopped.store(true, std::memory_order_release);
  };
  
  std::thread reader_thread(reader);
  std::thread writer_thread(writer);
  
  writer_thread.join();
  reader_running.store(false);  // Tell reader to stop waiting
  reader_thread.join();
  
  // Reader should have observed the flag change
  EXPECT_GE(updates.load(), 1);
}

// Test concurrent timeout detection
TEST_F(TestHardwareInterfaceSafety, ConcurrentTimeoutDetection)
{
  communication_timeout_ = 0.3;
  std::atomic<int> timeouts_detected{0};
  
  auto timeout_checker = [this, &timeouts_detected]() {
    for (int i = 0; i < 20; ++i) {
      if (check_timeout()) {
        timeouts_detected.fetch_add(1);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  };
  
  auto timestamp_updater = [this]() {
    for (int i = 0; i < 2; ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      last_read_time_ns_.store(now_ns(), std::memory_order_release);
    }
  };
  
  std::thread checker1(timeout_checker);
  std::thread checker2(timeout_checker);
  std::thread updater(timestamp_updater);
  
  checker1.join();
  checker2.join();
  updater.join();
  
  // At least some timeouts should be detected
  EXPECT_GT(timeouts_detected.load(), 0);
}

// Test nanosecond precision for timeout calculation
TEST_F(TestHardwareInterfaceSafety, NanosecondPrecisionTimeout)
{
  communication_timeout_ = 0.001;  // 1ms timeout (very short)
  
  int64_t start = now_ns();
  last_read_time_ns_.store(start, std::memory_order_release);
  
  // Wait 2ms
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
  
  EXPECT_TRUE(check_timeout());
}

// Test very large timeout value
TEST_F(TestHardwareInterfaceSafety, VeryLargeTimeoutValue)
{
  communication_timeout_ = 1e6;  // Very large timeout (1 million seconds)
  
  int64_t start = now_ns();
  last_read_time_ns_.store(start, std::memory_order_release);
  
  // Wait 1 second
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  
  // Should NOT be timed out with such large timeout
  EXPECT_FALSE(check_timeout());
}
