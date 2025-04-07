#ifndef SKIP_IF_NO_GPU_HPP__
#define SKIP_IF_NO_GPU_HPP__
#include <gtest/gtest.h>
#include <ateam_spatial/device_availability.hpp>

#define SKIP_IF_NO_GPU() { if(!ateam_spatial::IsCudaDeviceAvailable()) { GTEST_SKIP(); } }

#endif  // SKIP_IF_NO_GPU_HPP__
