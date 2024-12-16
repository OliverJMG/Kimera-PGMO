#pragma once

#include <rclcpp/rclcpp.hpp>

#include "kimera_pgmo/utils/logging.h"

namespace kimera_pgmo {

struct RosLogSink : logging::LogSink {
  RosLogSink() = default;
  virtual ~RosLogSink() = default;

  void dispatch(const logging::LogEntry& entry) const override {
    std::stringstream ss;
    ss << entry.prefix() << entry.message();
    switch (entry.level) {
      case logging::Level::WARNING:
        RCLCPP_WARN_STREAM(rclcpp::get_logger("kimera_pgmo"), ss.str());
        break;
      case logging::Level::ERROR:
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("kimera_pgmo"), ss.str());
        break;
      case logging::Level::FATAL:
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("kimera_pgmo"), ss.str());
        break;
      case logging::Level::INFO:
        RCLCPP_INFO_STREAM(rclcpp::get_logger("kimera_pgmo"), ss.str());
        break;
      default:
      case logging::Level::DEBUG:
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("kimera_pgmo"), ss.str());
        break;
    }
  }
};

}  // namespace kimera_pgmo
