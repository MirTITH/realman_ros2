#pragma once

#include <mutex>

namespace rm_driver
{

template <typename Data_t>
class DataWithLock
{
    using Mutex_t = std::mutex;

public:
    mutable Mutex_t lock_;
    Data_t data_;

    DataWithLock() = default;

    template <typename... Args>
    DataWithLock(Args &&...args)
        : data_(std::forward<Args>(args)...)
    {
    }

    void store(const Data_t &data)
    {
        std::lock_guard lock(lock_);
        data_ = data;
    }

    Data_t load() const
    {
        std::lock_guard lock(lock_);
        return data_;
    }

    std::lock_guard<Mutex_t> lock_guard() const
    {
        return std::lock_guard(lock_);
    }

    std::unique_lock<Mutex_t> unique_lock() const
    {
        return std::unique_lock(lock_);
    }
};

} // namespace rm_driver