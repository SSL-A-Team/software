#ifndef WORLD_UTILS_CUH 
#define WORLD_UTILS_CUH

__constant__ float WORLD_WIDTH_M;
__constant__ float WORLD_HEIGHT_M;
__constant__ float HALF_WORLD_WIDTH_M;
__constant__ float HALF_WORLD_HEIGHT_M;

__global__
void layerToWorld(void char* Pin, void char* Pout, float layerWidth, float layerHeight);

__global__
void layerToWorldX(void char* Pin, void char* Pout, float layerWidth);

__global__
void layerToWorldY(void char* Pin, void char* Pout, float layerHeight);

__global__
void worldToLayer(void char* Pin, void char* Pout, float layerWidth, float layerHeight);

__global__
void worldToLayerX(void char* Pin, void char* Pout, float layerWidth);

__global__
void worldToLayerY(void char* Pin, void char* Pout, float layerHeight);
#endif