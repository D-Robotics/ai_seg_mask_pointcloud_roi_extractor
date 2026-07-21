// Copyright 2025 perception
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

#pragma once

#include <deque>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>
#include <cv_bridge/cv_bridge.h>

namespace robot::mask_pc_roi_extractor
{

// ══════════════════════════════════════════════════════════════════════
// Constants
// ══════════════════════════════════════════════════════════════════════

/** @brief Default QoS depth for subscriptions */
constexpr size_t DEFAULT_QOS_DEPTH = 10;

/** @brief Default QoS depth for publishers (keep-last) */
constexpr size_t DEFAULT_QOS_PUB   = 1;

// ══════════════════════════════════════════════════════════════════════
// BoxInfo
// ══════════════════════════════════════════════════════════════════════

/**
 * @brief Information about a detected object bounding box.
 */
struct BoxInfo
{
    std::string name;            /**< Class name */
    size_t      id{0};           /**< Class ID */
    double      confidence{0.0}; /**< Confidence score [0, 1] */
    int         x_offset{0};     /**< X offset of top-left corner (pixels) */
    int         y_offset{0};     /**< Y offset of top-left corner (pixels) */
    int         width{0};        /**< Box width (pixels) */
    int         height{0};       /**< Box height (pixels) */
};

// ══════════════════════════════════════════════════════════════════════
// TaskTiming
// ══════════════════════════════════════════════════════════════════════

/**
 * @brief Per-task timing accumulator with atomic safety for multi-threaded
 *        workers.  Collects elapsed microseconds across many frames so the
 *        callback thread can periodically report average cost without locks.
 */
struct TaskTiming
{
    std::atomic<uint64_t> acc_us{0};  /**< Accumulated microseconds (fetch_add safe). */
    std::atomic<int>      count{0};   /**< Number of measured executions. */

    /** @brief Running average in milliseconds. */
    double avg_ms() const
    {
        int c = count.load(std::memory_order_relaxed);
        if (c == 0) return 0.0;
        return static_cast<double>(acc_us.load(std::memory_order_relaxed)) / c / 1000.0;
    }

    /** @brief Reset accumulator and counter (non-atomic — call from single writer). */
    void reset()
    {
        acc_us.store(0, std::memory_order_relaxed);
        count.store(0, std::memory_order_relaxed);
    }
};

// ══════════════════════════════════════════════════════════════════════
// ScopedTimer
// ══════════════════════════════════════════════════════════════════════

/**
 * @brief RAII timer that accumulates elapsed microseconds into a TaskTiming
 *        on destruction.  Place at the top of a lambda or function scope to
 *        measure its wall-clock duration.
 *
 *        When @p enabled is false the timer is a no-op (zero overhead beyond
 *        the branch on construction / destruction).  This lets timing run only
 *        in debug mode without littering every call site with conditionals.
 *
 *        Uses std::chrono::high_resolution_clock for monotonic measurement,
 *        matching the project's existing timing style.
 */
struct ScopedTimer
{
    using Clock = std::chrono::high_resolution_clock;

    Clock::time_point      start;
    std::atomic<uint64_t>& acc_us;
    std::atomic<int>&      count;
    bool                   active;

    ScopedTimer(std::atomic<uint64_t>& a, std::atomic<int>& c, bool enabled = true)
        : start(enabled ? Clock::now() : Clock::time_point{})
        , acc_us(a)
        , count(c)
        , active(enabled) {}

    ~ScopedTimer()
    {
        if (!active) return;
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
                           Clock::now() - start)
                           .count();
        acc_us.fetch_add(static_cast<uint64_t>(elapsed), std::memory_order_relaxed);
        count.fetch_add(1, std::memory_order_relaxed);
    }

    ScopedTimer(const ScopedTimer&) = delete;
    ScopedTimer& operator=(const ScopedTimer&) = delete;
};

// ══════════════════════════════════════════════════════════════════════
// MatPool
// ══════════════════════════════════════════════════════════════════════

/**
 * @brief Thread-safe reusable cv::Mat pool with bounded capacity.
 *
 * Avoids per-frame heap allocation of large full-resolution images.
 * Buffers are recycled within a free-list; a size/type mismatch
 * triggers a transparent reallocation.  When the free list reaches
 * max_pool_size, subsequent releases discard the buffer to bound
 * memory usage.
 */
class MatPool
{
public:
    /** @brief Default maximum pool size */
    static constexpr size_t kDefaultMaxPoolSize = 8;

    /**
     * @brief Construct with an optional pool size limit.
     * @param max_size  Maximum number of buffers to keep (0 = unbounded).
     */
    explicit MatPool(size_t max_size = kDefaultMaxPoolSize)
        : max_size_(max_size) {}

    /**
     * @brief Acquire a buffer of the requested dimensions and type.
     *
     * Returns a recycled buffer when available and size-compatible,
     * otherwise allocates a new one.
     *
     * @param rows  Number of rows.
     * @param cols  Number of columns.
     * @param type  OpenCV matrix type (e.g. CV_8UC1).
     * @return A cv::Mat with the requested geometry.
     */
    cv::Mat acquire(int rows, int cols, int type)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!free_.empty())
        {
            cv::Mat m = std::move(free_.back());
            free_.pop_back();
            if (m.rows == rows && m.cols == cols && m.type() == type)
            {
                return m;
            }
        }
        return cv::Mat(rows, cols, type);
    }

    /**
     * @brief Return a buffer to the pool for later reuse.
     *
     * If the pool is full the buffer is silently discarded (its memory
     * is freed by cv::Mat's destructor).
     *
     * @param m  The cv::Mat to recycle (must not be empty).
     */
    void release(const cv::Mat& m)
    {
        if (m.empty()) return;
        std::lock_guard<std::mutex> lock(mtx_);
        if (max_size_ == 0 || free_.size() < max_size_)
        {
            free_.push_back(m);
        }
    }

    /**
     * @brief Release all cached buffers immediately.
     */
    void clear()
    {
        std::lock_guard<std::mutex> lock(mtx_);
        free_.clear();
    }

private:
    std::mutex           mtx_;      /**< Mutex protecting the free list */
    std::vector<cv::Mat> free_;     /**< Recycled buffers */
    size_t               max_size_; /**< Maximum pool capacity (0 = unbounded) */
};

// ══════════════════════════════════════════════════════════════════════
// ThreadPool
// ══════════════════════════════════════════════════════════════════════

/**
 * @brief Fixed-size thread pool with a bounded task queue.
 *
 * Decouples message reception from compute: the synchronized callback
 * enqueues a job and returns immediately, while persistent worker
 * threads execute the work asynchronously.  Eliminates per-frame
 * thread-creation overhead and enables frame-level pipelining.
 *
 * Queue policy: drop-oldest — when the queue is full the earliest
 * (highest-latency) task is discarded to bound memory and delay.
 */
class ThreadPool
{
public:
    /**
     * @brief Construct a thread pool with @p n worker threads.
     * @param n  Number of workers (clamped to ≥ 1).
     */
    explicit ThreadPool(size_t n)
    {
        if (n == 0) n = 1;
        for (size_t i = 0; i < n; ++i)
        {
            workers_.emplace_back([this]() { run(); });
        }
    }

    /**
     * @brief Destructor.  Signals all workers to stop, wakes them,
     *        and joins before returning.
     */
    ~ThreadPool()
    {
        {
            std::lock_guard<std::mutex> lock(mtx_);
            stop_ = true;
        }
        cv_.notify_all();
        for (auto& w : workers_) w.join();
    }

    /**
     * @brief Enqueue a callable task for asynchronous execution.
     *
     * If the queue already holds @p max_pending tasks, the oldest
     * entry is dropped before insertion to bound latency and memory.
     *
     * @param task         Callable to execute on a worker thread.
     * @param max_pending  Queue capacity (tasks beyond this are dropped).
     */
    void enqueue(std::function<void()> task, size_t max_pending)
    {
        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (!tasks_.empty() && tasks_.size() >= max_pending)
            {
                tasks_.pop_front();
            }
            tasks_.push_back(std::move(task));
        }
        cv_.notify_one();
    }

private:
    /**
     * @brief Worker-thread loop.  Blocks on the condition variable,
     *        dequeues a task when available, and executes it.
     *        Exits when stop_ is set and the queue is drained.
     */
    void run()
    {
        while (true)
        {
            std::function<void()> task;
            {
                std::unique_lock<std::mutex> lock(mtx_);
                cv_.wait(lock, [this]() { return stop_ || !tasks_.empty(); });
                if (stop_ && tasks_.empty()) return;
                task = std::move(tasks_.front());
                tasks_.pop_front();
            }
            if (task) task();
        }
    }

    std::mutex                           mtx_;        /**< Protects tasks_ and stop_ */
    std::condition_variable              cv_;         /**< Worker wake-up signal */
    std::deque<std::function<void()>>    tasks_;      /**< Pending task queue */
    std::vector<std::thread>             workers_;    /**< Persistent worker threads */
    bool                                 stop_{false}; /**< Shutdown flag */
};

// ══════════════════════════════════════════════════════════════════════
// FrameJob
// ══════════════════════════════════════════════════════════════════════

/**
 * @brief Self-contained per-frame job dispatched to the thread pool.
 *
 * Owns all input data so it outlives the (non-blocking) callback.
 * Pooled mask buffers are automatically returned to the MatPool on
 * destruction (RAII).
 */
struct FrameJob
{
    cv_bridge::CvImageConstPtr depth_cv;           /**< Shared read-only depth (keeps ROS msg buffer alive) */
    cv::Mat                    mask_img;           /**< Pooled CV_8UC1 binary mask, owned by job */
    cv::Mat                    seg_mask_class_id;  /**< Pooled CV_8UC1 class-ID mask, owned by job */
    std::vector<BoxInfo>       boxes;              /**< Filtered detection boxes */
    std::string                frame_id;           /**< Coordinate frame ID */
    double                     timestamp{0.0};     /**< Frame timestamp (seconds) */
    rclcpp::Time               stamp;              /**< ROS time stamp */
    MatPool*                   mask_pool{nullptr}; /**< Pool to return buffers to on destruction */

    /**
     * @brief Immutable parameter snapshot captured at dispatch time.
     *
     * Worker threads read these fields instead of the live params_,
     * avoiding data races, torn reads, and out-of-bounds access when
     * camera_info updates frame dimensions mid-flight.
     */
    struct
    {
        bool  debug{false};
        int   cam_w{0};
        int   cam_h{0};
        float min_depth{0.0f};
        float max_depth{0.0f};
        bool  use_extractor{false};
    } cfg;

    /** @brief RAII destructor — returns pooled buffers on scope exit. */
    ~FrameJob()
    {
        if (mask_pool)
        {
            mask_pool->release(mask_img);
            mask_pool->release(seg_mask_class_id);
        }
    }
};

}  // namespace robot::mask_pc_roi_extractor
