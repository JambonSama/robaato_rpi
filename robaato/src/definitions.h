#pragma once

#include <cstdint>

namespace {
// queue size for published values
const uint32_t queue_size = 1;

// timing of operations
const double ready_time = 20;
const double operation_rate = 10;
const double tf_rate = 50; // transforms rate in [Hz]
const double tp_rate = 50; // topics rate in [Hz]
} // namespace
