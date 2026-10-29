// Single-writer sequence lock: the writer never blocks or waits (safe in the 1 kHz loop);
// readers retry until they get a consistent copy. T must be trivially copyable.
#pragma once

#include <atomic>
#include <cstdint>
#include <type_traits>

template <typename T>
class SeqLock {
  static_assert(std::is_trivially_copyable<T>::value, "SeqLock needs a trivially copyable type");

 public:
  void store(const T& value) {
    const uint32_t s = seq_.load(std::memory_order_relaxed);
    seq_.store(s + 1, std::memory_order_relaxed);  // odd: write in progress
    std::atomic_thread_fence(std::memory_order_release);
    data_ = value;
    std::atomic_thread_fence(std::memory_order_release);
    seq_.store(s + 2, std::memory_order_release);
  }

  // Single attempt, for the real-time thread: false if a write was in progress (try again next
  // tick). A retry loop there could spin forever if it preempted the writer mid-write.
  bool tryLoad(T* out) const {
    const uint32_t before = seq_.load(std::memory_order_acquire);
    if (before & 1u) {
      return false;
    }
    std::atomic_thread_fence(std::memory_order_acquire);
    *out = data_;
    std::atomic_thread_fence(std::memory_order_acquire);
    return seq_.load(std::memory_order_relaxed) == before;
  }

  T load() const {
    T out;
    uint32_t before, after;
    do {
      before = seq_.load(std::memory_order_acquire);
      std::atomic_thread_fence(std::memory_order_acquire);
      out = data_;
      std::atomic_thread_fence(std::memory_order_acquire);
      after = seq_.load(std::memory_order_relaxed);
    } while (before != after || (before & 1u));
    return out;
  }

 private:
  std::atomic<uint32_t> seq_{0};
  T data_{};
};
