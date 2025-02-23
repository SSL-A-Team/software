#include "ateam_spatial/placeholder.hpp"
#include <cuda_runtime.h>
#include <iostream>

__global__ void cuda_hello_impl() {
  printf("Hello World from GPU!\n");
}

namespace ateam_spatial {

void cuda_hello() {
  cuda_hello_impl<<<1,1>>>();
  cudaDeviceSynchronize();
}

}
