#pragma once
#include <vector>
#include <thread>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <future>
#include <functional>
#include <atomic>
#include <cstddef>

class ThreadPool {
public:
    explicit ThreadPool(size_t nthreads) : stop_(false) {
        if (nthreads == 0) nthreads = 1;
        workers_.reserve(nthreads);
        for (size_t i = 0; i < nthreads; ++i) {
            workers_.emplace_back([this]() { this->worker_loop_(); });
        }
    }

    ~ThreadPool() {
        {
            std::unique_lock<std::mutex> lock(m_);
            stop_ = true;
        }
        cv_.notify_all();
        for (auto& t : workers_) {
            if (t.joinable()) t.join();
        }
    }

    // Enqueue a void() task
    void enqueue(std::function<void()> fn) {
        {
            std::unique_lock<std::mutex> lock(m_);
            tasks_.push(std::move(fn));
        }
        cv_.notify_one();
    }

    // Parallel for over [begin, end), chunked by 'block' rows.
    // fn(y0, y1) processes a half-open row range.
    void parallel_for(size_t begin, size_t end, size_t block, const std::function<void(size_t, size_t)>& fn) {
        if (end <= begin) return;
        if (block == 0) block = 1;

        size_t total = end - begin;
        size_t nchunks = (total + block - 1) / block;

        // Fast path single thread or very small job
        if (workers_.empty() || nchunks == 1) {
            fn(begin, end);
            return;
        }

        auto remaining = std::make_shared<std::atomic<size_t>>(nchunks);
        auto prom = std::make_shared<std::promise<void>>();
        auto fut = prom->get_future();

        for (size_t i = 0; i < nchunks; ++i) {
            size_t y0 = begin + i * block;
            size_t y1 = std::min(end, y0 + block);
            enqueue([=]() {
                fn(y0, y1);
                if (remaining->fetch_sub(1) == 1) {
                    prom->set_value();
                }
            });
        }

        fut.get(); // wait for all chunks
    }

    size_t size() const { return workers_.size(); }

private:
    void worker_loop_() {
        for (;;) {
            std::function<void()> task;
            {
                std::unique_lock<std::mutex> lock(m_);
                cv_.wait(lock, [this]() { return stop_ || !tasks_.empty(); });
                if (stop_ && tasks_.empty()) return;
                task = std::move(tasks_.front());
                tasks_.pop();
            }
            task();
        }
    }

    std::vector<std::thread> workers_;
    std::mutex m_;
    std::condition_variable cv_;
    std::queue<std::function<void()>> tasks_;
    bool stop_;
};
