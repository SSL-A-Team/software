#include "cuda_runtime.h"
#include "world_utils.cuh"

__global__
void distanceFromEdgeKernel(unsigned char* Pout, unsigned char* Pin,
    int width, int height)
{
    int col = blockIdx.x * blockDim.x + threadIdx.x;
    int row = blockIdx.y * blockDim.y + threadIdx.y;

    if (col < width && row < height){
        float distance = 0;
        // Return only one distance per point using row-major
        // indexing.
        int out_coord_array_pos = row * width + col;
        // We have double the number of cols
        // to ensure we include both the x and y coordinates
        // (x first, then y).
        int in_coord_array_pos = out_coord_array_pos * 2;
        float world_pos_x = fabsf(Pin[in_coord_array_pos]);
        float world_pos_y = fabsf(Pin[in_coord_array_pos + 1]);
        // Get absolute value
        // There's probably a way to do this that doesn't involve conditions
        // so we can be more efficient
        if (world_pos_x < (HALF_WORLD_WIDTH_M - HALF_WORLD_HEIGHT_M) ||
            world_pos_x - (HALF_WORLD_WIDTH_M - HALF_WORLD_HEIGHT_M) < world_pos_y){
            distance = HALF_WORLD_HEIGHT_M - world_pos_y;
        } else {
            distance = HALF_WORLD_WIDTH_M - world_pos_x;
        }
        Pout[out_coord_array_pos] = distance;
    }
};

void launchDistanceFromEdgeKernel(unsigned char* d_Pout, unsigned char* d_Pin,
    int width, int height)
{
    dim3 dimBlock(16, 16, 1);
    dim3 dimGrid(ceil(width / 16.0), ceil(height / 16.0), 1);

    distanceFromEdgeKernel<<<dimGrid, dimBlock>>>(d_Pout, d_Pin, width, height);
    cudaDeviceSynchronize();
};

__global__
void distanceDownFieldKernel(unsigned char* Pout, unsigned char* Pin, int width, int height);

void launchDistanceDownFieldKernel(unsigned char* d_Pout, unsigned char* d_Pin, int width, int height);