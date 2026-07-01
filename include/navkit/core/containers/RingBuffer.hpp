// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include <array>
#include <cstddef>

namespace navkit::core::containers
{

enum class OverflowPolicy
{
    Reject,
    OverwriteOldest
};

template<typename T, std::size_t N, OverflowPolicy Policy = OverflowPolicy::Reject>
class RingBuffer
{
public:
    static_assert(N > 0, "RingBuffer capacity must be greater than zero");

    bool push(const T& value)
    {
        if (m_count == N) {
            if constexpr (Policy == OverflowPolicy::OverwriteOldest) {
                advance_tail();
            }
            else {
                return false;
            }
        }

        m_data[m_head] = value;
        m_head = (m_head + 1U) % N;
        ++m_count;
        return true;
    }

    bool pop(T& out)
    {
        if (empty()) {
            return false;
        }
        out = m_data[m_tail];
        advance_tail();
        return true;
    }

    bool front(T& out) const
    {
        if (empty()) {
            return false;
        }
        out = m_data[m_tail];
        return true;
    }

    [[nodiscard]] bool empty() const
    {
        return m_count == 0U;
    }
    [[nodiscard]] bool full() const
    {
        return m_count == N;
    }
    [[nodiscard]] std::size_t size() const
    {
        return m_count;
    }
    [[nodiscard]] constexpr std::size_t capacity() const
    {
        return N;
    }

    void clear()
    {
        m_head = 0U;
        m_tail = 0U;
        m_count = 0U;
    }

private:
    void advance_tail()
    {
        m_tail = (m_tail + 1U) % N;
        --m_count;
    }

    std::array<T, N> m_data{};
    std::size_t m_head{0U};
    std::size_t m_tail{0U};
    std::size_t m_count{0U};
};

} // namespace navkit::core::containers
