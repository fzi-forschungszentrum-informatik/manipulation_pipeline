// Copyright 2025 FZI Forschungszentrum Informatik
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/*!\file manipulation_pipeline/worker_thread.h
 * \brief Asynchronous worker thread for detached execution of planning and execution tasks
 *
 * \author  Robert Wilbrandt <wilbrandt@fzi.de>
 * \date    2025-04-08
 *
 */
//----------------------------------------------------------------------
#ifndef MANIPULATION_PIPELINE_WORKER_THREAD_H_INCLUDED
#define MANIPULATION_PIPELINE_WORKER_THREAD_H_INCLUDED

#include <condition_variable>
#include <functional>
#include <mutex>
#include <optional>
#include <queue>
#include <stop_token>
#include <thread>

namespace manipulation_pipeline {

/*! \brief Asynchronous worker thread with task queue
 *
 * \tparam T Type of tasks to execute with worker
 */
template <typename T>
class WorkerThread
{
public:
  using Task = std::function<void(const T&, std::stop_token)>;

  explicit WorkerThread(Task task);
  ~WorkerThread();

  WorkerThread(const WorkerThread&)           = delete;
  WorkerThread& operator=(const WorkerThread) = delete;

  void push(const T& work);

private:
  void run(std::stop_token stop);

  Task m_task;

  std::queue<T> m_queue;
  std::mutex m_queue_mut;
  std::condition_variable m_queue_cv;

  std::jthread m_thread;
};

} // namespace manipulation_pipeline


namespace manipulation_pipeline {

template <typename T>
WorkerThread<T>::WorkerThread(Task task)
  : m_task{std::move(task)}
  , m_thread{std::bind(&WorkerThread<T>::run, this, std::placeholders::_1)}
{
}

template <typename T>
WorkerThread<T>::~WorkerThread()
{
  m_thread.request_stop();
  m_queue_cv.notify_all();
}

template <typename T>
void WorkerThread<T>::push(const T& work)
{
  {
    std::lock_guard lock{m_queue_mut};
    m_queue.push(work);
  }
  m_queue_cv.notify_all();
}

template <typename T>
void WorkerThread<T>::run(std::stop_token stop)
{
  while (!stop.stop_requested())
  {
    const auto new_data = [&]() -> std::optional<T> {
      // Wait until we either get new work or should stop
      std::unique_lock lock{m_queue_mut};
      m_queue_cv.wait(lock, [&] { return !m_queue.empty() || stop.stop_requested(); });

      if (stop.stop_requested())
      {
        return std::nullopt;
      }

      T front = m_queue.front();
      m_queue.pop();
      return front;
    }();

    if (new_data)
    {
      m_task(*new_data, stop);
    }
  }
}

} // namespace manipulation_pipeline

#endif // MANIPULATION_PIPELINE_WORKER_THREAD_H_INCLUDED
