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

#include <cuda_runtime.h>
#include <iostream>
#include "ateam_spatial/placeholder.hpp"
#include "ateam_spatial/gpu_object.hpp"
#include "ateam_spatial/gpu_array.hpp"

struct TestStruct {
  int mem;
};

__global__ void cuda_hello_impl(TestStruct * t, int * arr, size_t arr_size) {
  printf("Hello World from GPU!\n");
  printf("The number is %d\n", t->mem);
  for(auto i = 0; i < arr_size; ++i) {
    printf("Array[%d] = %d\n", i, arr[i]);
  }
}

namespace ateam_spatial {

void cuda_hello() {
  GpuObject<TestStruct> t({42});
  std::array<int,5> host_array = {2,4,6,8,10};
  GpuArray<int, 5> a;
  a.CopyToGpu(host_array);
  cuda_hello_impl<<<1,1>>>(t.Get(), a.Get(), a.Size());
  cudaDeviceSynchronize();
}

}
