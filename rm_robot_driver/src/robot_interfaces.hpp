#pragma once

#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <fmt/core.h>
#include <string>
#include "robot_interface_mode.hpp"

namespace rm_driver
{

class RobotInterfaces
{
public:
    using cb_t = std::function<void(std::vector<double> &)>;

public:
    RobotInterfaces() = default;

    RobotInterfaces(size_t data_length, double initial_value = std::numeric_limits<double>::quiet_NaN(), cb_t update_callback = nullptr)
        : data_(data_length, initial_value), update_callback_(std::move(update_callback)){};

    void Init(size_t data_length, double initial_value = std::numeric_limits<double>::quiet_NaN(), cb_t update_callback = nullptr)
    {
        data_.resize(data_length, initial_value);
        update_callback_ = std::move(update_callback);
        counter_         = 0;
    }

    double &operator[](size_t index)
    {
        return data_[index];
    }

    const double &operator[](size_t index) const
    {
        return data_[index];
    }

    double &at(size_t index)
    {
        return data_.at(index);
    }

    const double &at(size_t index) const
    {
        return data_.at(index);
    }

    size_t GetDataLength() const
    {
        return data_.size();
    }

    void SetUpdateRateDivider(size_t divider)
    {
        update_rate_divider_ = divider;
    }

    size_t GetUpdateRateDivider() const
    {
        return update_rate_divider_;
    }

    void SetUpdateCallback(cb_t update_callback)
    {
        update_callback_ = std::move(update_callback);
    }

    void SetMode(RobotInterfaceMode mode)
    {
        mode_ = mode;
    }

    RobotInterfaceMode GetMode() const
    {
        return mode_;
    }

    void Update()
    {
        if (mode_ == RobotInterfaceMode::STOP) {
            return;
        }

        if (counter_ == 0) {
            if (update_callback_) update_callback_(data_);
        }

        counter_++;
        if (counter_ >= update_rate_divider_) {
            counter_ = 0;
        }
    }

    template <typename T>
    void SetData(const T &input)
    {
        if (input.size() != data_.size()) {
            throw std::runtime_error(fmt::format("{}: input.size() which is {} != {}", __func__, input.size(), data_.size()));
        }
        std::copy(input.begin(), input.end(), data_.begin());
    }

    bool ContainsNaN() const
    {
        for (const auto &d : data_) {
            if (std::isnan(d)) {
                return true;
            }
        }
        return false;
    }

    std::vector<double> GetData() const
    {
        return data_;
    }

private:
    RobotInterfaceMode mode_ = RobotInterfaceMode::STOP;
    std::vector<double> data_; // Be cautious to avoid changing the data pointer. Use std::copy instead of direct assignment
    size_t update_rate_divider_ = 1;
    size_t counter_             = 0;
    cb_t update_callback_;
};

} // namespace rm_driver