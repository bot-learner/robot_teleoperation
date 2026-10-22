// 专用后台 JSONL 写盘，不在 ROS/Iceoryx 回调中等待文件系统。
#pragma once
#include <chrono>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <json/json.h>
#include <mutex>
#include <stdexcept>
#include <string_view>
#include <thread>
#include <unistd.h>

class DiagnosticLog {
 public:
  DiagnosticLog() {
    std::filesystem::create_directories("logs");
    const auto wall = std::chrono::system_clock::now().time_since_epoch();
    path_ = "logs/mdu_" + std::to_string(
        std::chrono::duration_cast<std::chrono::nanoseconds>(wall).count())
        + "_" + std::to_string(getpid()) + ".jsonl";
    stream_.open(path_);
    if (!stream_) throw std::runtime_error("Cannot open MDU diagnostic log: " + path_);
    worker_ = std::thread([this] { Drain(); });
    std::cout << "Arm diagnostic log: " << path_ << std::endl;
  }
  ~DiagnosticLog() {
    Write("program_stopped");
    { std::lock_guard lock(mutex_); stopping_ = true; }
    wake_.notify_one();
    worker_.join();
  }
  void Write(const char* event, Json::Value row = Json::Value(Json::objectValue)) {
    row["role"] = "mdu";
    row["event"] = event;
    row["pid"] = getpid();
    row["monotonic_ns"] = Json::Int64(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
    row["wall_ns"] = Json::Int64(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count());
    {
      std::lock_guard lock(mutex_);
      if (queue_.size() >= 4096 && std::string_view(event) != "program_stopped") { ++dropped_; return; }
      row["dropped_total"] = Json::UInt64(dropped_);
      queue_.push_back(std::move(row));
    }
    wake_.notify_one();
  }
 private:
  void Drain() {
    Json::StreamWriterBuilder writer;
    writer["indentation"] = "";
    writer["useSpecialFloats"] = false;
    size_t bytes = 0;
    for (;;) {
      Json::Value row;
      {
        std::unique_lock lock(mutex_);
        wake_.wait(lock, [this] { return stopping_ || !queue_.empty(); });
        if (queue_.empty()) return;
        row = std::move(queue_.front()); queue_.pop_front();
      }
      try {
        const auto line = Json::writeString(writer, row) + '\n';
        if (bytes + line.size() > 40 * 1024 * 1024) {
          stream_.close();
          for (int i = 4; i >= 1; --i) {
            const auto from = path_ + (i == 1 ? "" : "." + std::to_string(i - 1));
            if (std::filesystem::exists(from))
              std::filesystem::rename(from, path_ + "." + std::to_string(i));
          }
          stream_.open(path_, std::ios::trunc);
          bytes = 0;
        }
        stream_ << line;
        stream_.flush();
        if (!stream_) throw std::runtime_error("MDU diagnostic log write failed");
        bytes += line.size();
      } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
      }
    }
  }
  std::string path_;
  std::ofstream stream_;
  std::mutex mutex_;
  std::condition_variable wake_;
  std::deque<Json::Value> queue_;
  bool stopping_ = false;
  size_t dropped_ = 0;
  std::thread worker_;
};
