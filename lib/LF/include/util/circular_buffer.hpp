#pragma once

#include <cstddef>
#include <array>

namespace tablog::util {

/// Fixed capacity circular buffer.
/// Never allocates, never throws.
/// Indexing type may be replaced for MCU compatibility.
template <typename T, std::size_t N, typename IndexT=std::size_t>
class CircularBuffer {
public:
    using value_type = T;

    /// Push an item to the buffer, removes the oldest one if buffer is full
    void push_back(const T& v) noexcept {
        if (used < N) {
            a[index(used++)] = v;
        }
        else
        {
            a[startOffset] = v;
            startOffset = index(1);
        }

    }

    /// Remove the last item in the buffer, undefined behavior if the
    /// buffer is empty.
    value_type pop_front() noexcept {
        used--;
        auto idx = startOffset;
        startOffset = index(1);
        return a[idx];
    }

    void clear() noexcept {
        used = 0;
    }

    bool empty() const noexcept {
        return used == 0;
    }

    bool full() const noexcept {
        return used >= N;
    }

    /// Number of items currently held in the buffer
    IndexT size() const noexcept {
        return used;
    }

    /// Maximum number of items held in this buffer.
    static IndexT capacity() noexcept {
        return N;
    }

    /// Access i-th element.
    /// Accessing nonexistent item is UB.
    const value_type& operator[](IndexT i) const noexcept
    {
        return a[index(i)];
    }

    /// Access i-th element.
    /// Accessing nonexistent item is UB.
    value_type& operator[](IndexT i) noexcept
    {
        return a[index(i)];
    }

    /// Access first element.
    /// It is UB if the buffer is empty.
    value_type& front() noexcept {
        return a[startOffset];
    }

    /// Access first element.
    /// It is UB if the buffer is empty.
    const value_type& front() const noexcept {
        return a[startOffset];
    }

    /// Access last element.
    /// It is UB if the buffer is empty.
    value_type& back() noexcept {
        return a[index(used - 1)];
    }

    /// Access last element.
    /// It is UB if the buffer is empty.
    const value_type& back() const noexcept {
        return a[index(used - 1)];
    }

private:
    IndexT index(IndexT i) const noexcept {
        IndexT i2 = startOffset + i;
        if (i2 < N)
            return i2;
        else
            return i2 - N;
    }

    IndexT startOffset = 0;
    IndexT used = 0;
    T a[N];
};

/// Fixed size circular buffer. Initialized by pre-filling with a value, cannot pop.
/// Never allocates, never throws.
/// Indexing type may be replaced for MCU compatibility.
template <typename T, std::size_t N, typename IndexT=std::size_t>
class CircularBufferFixed {
public:
    using value_type = T;

    CircularBufferFixed(value_type initialValue = value_type()) {
        for (IndexT i = 0; i < N; ++i)
            a[i] = initialValue;
    }

    /// Push an item to the buffer, removes the oldest one.
    void push_back(const T& v) noexcept {
        a[startOffset] = v;
        startOffset = index(1);
    }

    /// Fixed circuar buffer can never be empty.
    bool empty() const noexcept {
        return false;
    }

    /// Fixed circular buffer is always full.
    IndexT size() const noexcept {
        return N;
    }

    /// Maximum number of items held in this buffer.
    static IndexT capacity() noexcept {
        return N;
    }

    /// Access i-th element.
    /// Accessing nonexistent item is UB.
    const value_type& operator[](IndexT i) const noexcept
    {
        return a[index(i)];
    }

    /// Access i-th element.
    /// Accessing nonexistent item is UB.
    value_type& operator[](IndexT i) noexcept
    {
        return a[index(i)];
    }

    /// Access first element.
    /// It is UB if the buffer is empty.
    value_type& front() noexcept {
        return a[startOffset];
    }

    /// Access first element.
    /// It is UB if the buffer is empty.
    const value_type& front() const noexcept {
        return a[startOffset];
    }

    /// Access last element.
    /// It is UB if the buffer is empty.
    value_type& back() noexcept {
        return a[index(N - 1)];
    }

    /// Access last element.
    /// It is UB if the buffer is empty.
    const value_type& back() const noexcept {
        return a[index(N - 1)];
    }

private:
    IndexT index(IndexT i) const noexcept {
        IndexT i2 = startOffset + i;
        if (i2 < N)
            return i2;
        else
            return i2 - N;
    }

    IndexT startOffset = 0;
    T a[N];
};

}
