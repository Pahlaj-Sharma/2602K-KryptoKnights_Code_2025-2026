#pragma once

#include <memory>
#include <array>

#define FMT_HEADER_ONLY
#include "fmt/core.h"

#include "pahlib/logger/baseSink.hpp"
#include "pahlib/logger/infoSink.hpp"
#include "pahlib/logger/telemetrySink.hpp"

namespace pahlib {

/**
 * @brief Get the info sink.
 * @return std::shared_ptr<InfoSink>
 */
std::shared_ptr<InfoSink> infoSink();

/**
 * @brief Get the telemetry sink.
 * @return std::shared_ptr<TelemetrySink>
 */
std::shared_ptr<TelemetrySink> telemetrySink();
} // namespace pahlib
