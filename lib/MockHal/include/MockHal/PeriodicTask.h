#pragma once

#include <thread>
#include <mutex>
#include <chrono>
#include <functional>

namespace MockHal {

template <typename T>
class PeriodicTask {
public:
    PeriodicTask(const PeriodicTask&) = delete;
    PeriodicTask(PeriodicTask&&) = delete;
    PeriodicTask operator=(const PeriodicTask&) = delete;
    PeriodicTask operator=(PeriodicTask&&) = delete;

    template<typename F>
    void access(F f) {
        std::unique_lock lock(mutex);
        f(data);
    }
private:

    template<typename F, typename Duration>
    PeriodicTask(T&& data, F function, Duration period)
    :
        data(std::move(data)),
        function(function),
        period(std::chrono::duration_cast<std::chrono::steady_clock::duration>(period)),
        thread(&PeriodicTask::task_main, this)
    {
    }

    void task_main() {
        auto awakeTime = std::chrono::steady_clock::now();
        while (true) {
            awakeTime += period;
            std::this_thread::sleep_until(awakeTime);

            std::unique_lock lock(mutex);
            function(data);
        }
    }

    T data;
    std::function<void(T&)> function;
    std::chrono::steady_clock::duration period;

    std::thread thread;
    std::mutex mutex;

    friend class Hal;
};

}
