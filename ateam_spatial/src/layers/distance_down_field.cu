#include "ateam_spatial/layers/distance_down_field.hpp"
#include "ateam_spatial/coordinate_conversions.hpp"

namespace ateam_spatial::layers
{

CUDA_HOSTDEV float DistanceDownField(const int x, const FieldDimensions & field_dims, const SpatialSettings & settings) {
  return SpatialToRealX(x, field_dims, settings) + (WorldWidthReal(field_dims) / 2.0);
}

__global__
void distanceDownFieldKernel(unsigned char* Pout, unsigned char* Pin, const FieldDimensions & field_dims,
    const SpatialSettings & settings)
{
  int col = blockIdx.x * blockDim.x + threadIdx.x;
  int row = blockIdx.y * blockDim.y + threadIdx.y;

  if (col < width && row < height){
    int out_coord_array_pos = row * settings.width + col;

    float distance = DistanceDownField(col, field_dims, settings);
    Pout[out_coord_array_pos]
  }
}
  
} // namespace ateam_spatial::layers
