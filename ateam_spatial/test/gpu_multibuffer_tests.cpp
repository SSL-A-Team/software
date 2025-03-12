// Copyright 2025 A Team
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <ateam_spatial/gpu_multibuffer.hpp>


TEST(GpuMultibufferTests, CopyBackAndForth) {
  ateam_spatial::GpuMultibuffer<int> mb(2, 10);

  const std::vector<int> in1{1, 3, 5, 7, 9, 11, 13, 15, 17, 19};
  const std::vector<int> in2{0, 2, 4, 6, 8, 10, 12, 14, 16, 18};
  mb.CopyToGpu(0, in1);
  mb.CopyToGpu(1, in2);

  std::vector<int> out1;
  std::vector<int> out2;
  mb.CopyFromGpu(0, out1);
  mb.CopyFromGpu(1, out2);

  EXPECT_THAT(out1, ::testing::ContainerEq(in1));
  EXPECT_THAT(out2, ::testing::ContainerEq(in2));
}
