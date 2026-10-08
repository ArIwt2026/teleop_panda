// Copyright (c) 2026
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <type_traits>

namespace franka_hardware {

// Single producer / single consumer mailbox. The producer, consumer, and exchange
// each own a different slot. Exchanging ownership never waits for the other
// thread, and the consumer retains a complete snapshot between publications.
// Calls on each side must be serialized, including during lifecycle transitions.
// T must have bounded, allocation-free copying (RobotState includes Errors with
// a custom copy operation, so it cannot be transferred with memcpy).
template <typename T>
class LatestValueBuffer {
 public:
  static_assert(std::is_copy_assignable<T>::value, "Mailbox payload must support copying");
  static_assert(std::atomic<uint32_t>::is_always_lock_free, "Slot exchange must be lock-free");

  void publish(const T& value) noexcept {
    slots_[producer_slot_] = value;
    producer_slot_ = exchange_.exchange(producer_slot_ | kUnread, std::memory_order_acq_rel) & kIndex;
  }

  const T& read() noexcept {
    if (exchange_.load(std::memory_order_acquire) & kUnread) {
      consumer_slot_ = exchange_.exchange(consumer_slot_, std::memory_order_acq_rel) & kIndex;
    }
    return slots_[consumer_slot_];
  }

 private:
  static constexpr uint32_t kUnread = 4;
  static constexpr uint32_t kIndex = 3;
  std::array<T, 3> slots_{};
  // Separate producer/consumer bookkeeping to avoid false sharing at 1 kHz.
  alignas(64) uint32_t producer_slot_{2};
  alignas(64) uint32_t consumer_slot_{0};
  alignas(64) std::atomic<uint32_t> exchange_{1};
};

}  // namespace franka_hardware
