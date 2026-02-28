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
#include <vector>

// Test thread safety of atomic timestamp operations
class TestHardwareInterfaceThreading : public ::testing::Test
{
protected:
  std::atomic<int64_t> timestamp_ns_{0};
  std::atomic<bool> stop_{false};
  
  int64_t now_ns()
  {
    return std::chrono::steady_clock::now().time_since_epoch().count();
  }
};

// Test concurrent updates to atomic timestamp
TEST_F(TestHardwareInterfaceThreading, ConcurrentTimestampUpdates)
{
  const int num_threads = 4;
  const int updates_per_thread = 100;
  
  auto update_timestamp = [this]() {
    for (int i = 0; i < updates_per_thread; ++i) {
      timestamp_ns_.store(now_ns(), std::memory_order_release);
    }
  };
  
  std::vector<std::thread> threads;
  for (int i = 0; i < num_threads; ++i) {
    threads.emplace_back(update_timestamp);
  }
  
  for (auto & t : threads) {
    if (t.joinable()) t.join();
  }
  
  // Should have valid final timestamp
  EXPECT_GT(timestamp_ns_.load(std::memory_order_acquire), 0);
}

// Test concurrent reads of atomic timestamp
TEST_F(TestHardwareInterfaceThreading, ConcurrentTimestampReads)
{
  timestamp_ns_.store(now_ns(), std::memory_order_release);
  
  const int num_threads = 8;
  const int reads_per_thread = 100;
  std::atomic<int> read_count{0};
  
  auto read_timestamp = [this, &read_count]() {
    for (int i = 0; i < reads_per_thread; ++i) {
      int64_t ts = timestamp_ns_.load(std::memory_order_acquire);
      if (ts > 0) {
        read_count.fetch_add(1);
      }
    }
  };
  
  std::vector<std::thread> threads;
  for (int i = 0; i < num_threads; ++i) {
    threads.emplace_back(read_timestamp);
  }
  
  for (auto & t : threads) {
    if (t.joinable()) t.join();
  }
  
  // All reads should have succeeded
  EXPECT_EQ(read_count.load(), num_threads * reads_per_thread);
}

// Test interleaved reads/writes are thread-safe
TEST_F(TestHardwareInterfaceThreading, InterleavedReadWrite)
{
  std::atomic<bool> writer_ready{false};
  std::atomic<int> successful_reads{0};
  
  auto writer = [this, &writer_ready]() {
    for (int i = 0; i < 50; ++i) {
      timestamp_ns_.store(now_ns(), std::memory_order_release);
      std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
    writer_ready.store(true);
  };
  
  auto reader = [this, &writer_ready, &successful_reads]() {
    while (!writer_ready.load()) {
      int64_t ts = timestamp_ns_.load(std::memory_order_acquire);
      if (ts > 0) {
        successful_reads.fetch_add(1);
      }
      std::this_thread::yield();
    }
  };
  
  std::thread w(writer);
  std::vector<std::thread> readers;
  
  for (int i = 0; i < 3; ++i) {
    readers.emplace_back(reader);
  }
  
  w.join();
  for (auto & r : readers) {
    if (r.joinable()) r.join();
  }
  
  // Readers should have successfully read some timestamps
  EXPECT_GT(successful_reads.load(), 0);
}

// Test memory ordering with write-then-read pattern
TEST_F(TestHardwareInterfaceThreading, MemoryOrderingWriteRead)
{
  std::atomic<bool> phase1_complete{false};
  int64_t expected_value = 0;
  
  auto writer = [this, &phase1_complete, &expected_value]() {
    expected_value = now_ns();
    timestamp_ns_.store(expected_value, std::memory_order_release);
    phase1_complete.store(true, std::memory_order_release);
  };
  
  auto reader = [this, &phase1_complete, &expected_value]() {
    while (!phase1_complete.load(std::memory_order_acquire)) {
      std::this_thread::yield();
    }
    int64_t read_value = timestamp_ns_.load(std::memory_order_acquire);
    EXPECT_EQ(read_value, expected_value);
  };
  
  std::thread w(writer);
  std::thread r(reader);
  
  w.join();
  r.join();
}

// Test no data race with rapid alternating updates
TEST_F(TestHardwareInterfaceThreading, RapidAlternatingUpdates)
{
  const int rapid_updates = 1000;
  std::atomic<int> writer_count{0};
  std::atomic<int> reader_count{0};
  
  auto rapid_write = [this, &writer_count]() {
    for (int i = 0; i < rapid_updates; ++i) {
      timestamp_ns_.store(now_ns() + i, std::memory_order_release);
      writer_count.fetch_add(1);
    }
  };
  
  auto rapid_read = [this, &reader_count]() {
    for (int i = 0; i < rapid_updates; ++i) {
      int64_t ts = timestamp_ns_.load(std::memory_order_acquire);
      (void)ts;  // Use variable to avoid compiler warning
      reader_count.fetch_add(1);
    }
  };
  
  std::thread w(rapid_write);
  std::thread r(rapid_read);
  
  w.join();
  r.join();
  
  EXPECT_EQ(writer_count.load(), rapid_updates);
  EXPECT_EQ(reader_count.load(), rapid_updates);
}

// Test multiple writers, multiple readers concurrently
TEST_F(TestHardwareInterfaceThreading, MultiWriterMultiReader)
{
  const int num_writers = 2;
  const int num_readers = 3;
  const int ops_per_thread = 50;
  
  std::atomic<int> valid_reads{0};
  
  auto write_op = [this]() {
    for (int i = 0; i < ops_per_thread; ++i) {
      timestamp_ns_.store(now_ns(), std::memory_order_release);
    }
  };
  
  auto read_op = [this, &valid_reads]() {
    for (int i = 0; i < ops_per_thread; ++i) {
      int64_t ts = timestamp_ns_.load(std::memory_order_acquire);
      if (ts >= 0) {
        valid_reads.fetch_add(1);
      }
    }
  };
  
  std::vector<std::thread> workers;
  
  for (int i = 0; i < num_writers; ++i) {
    workers.emplace_back(write_op);
  }
  
  for (int i = 0; i < num_readers; ++i) {
    workers.emplace_back(read_op);
  }
  
  for (auto & t : workers) {
    if (t.joinable()) t.join();
  }
  
  // All reads should have succeeded
  EXPECT_EQ(valid_reads.load(), num_readers * ops_per_thread);
}
