#include "log.h"

std::shared_ptr<spdlog::async_logger> LOG::LOGGER::my_logger_ = nullptr;    