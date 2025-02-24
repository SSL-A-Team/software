#ifndef DISTANCE_OPS_CUH
#define DISTANCE_OPS_CUH

__global__
void distanceFromEdgeKernel(unsigned char* Pout, unsigned char* Pin, int width, int height);

void launchDistanceFromEdgeKernel(unsigned char* d_Pout, unsigned char* d_Pin, int width, int height);

__global__
void distanceDownFieldKernel(unsigned char* Pout, unsigned char* Pin, int width, int height);

void launchDistanceDownFieldKernel(unsigned char* d_Pout, unsigned char* d_Pin, int width, int height);
#endif 