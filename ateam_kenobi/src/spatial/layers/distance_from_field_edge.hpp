// Copyright 2024 A Team
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

#ifndef SPATIAL__LAYERS__DISTANCE_FROM_FIELD_EDGE_HPP_
#define SPATIAL__LAYERS__DISTANCE_FROM_FIELD_EDGE_HPP_

#include "spatial/spatial_layer_factory.hpp"

namespace ateam_kenobi::spatial::layers
{

class DistanceFromFieldEdge final : public SpatialLayerFactory {
public:
  DistanceFromFieldEdge()
  : SpatialLayerFactory("DistanceFromFieldEdge") {}

  void FillLayer(cv::Mat & layer, const World &) override
  {
    SetupLayer(layer, CV_32FC1);
    if(layer.size() == prev_size) {
      // No need to regenerate this layer unless the field size changes
      return;
    }
    prev_size = layer.size();

    const auto half_world_width = WorldWidth() / 2.0;
    const auto half_world_height = WorldHeight() / 2.0;

    for(auto y = 0; y < LayerHeight(); ++y) {
      auto row = layer.ptr<float>(y);
      for(auto x = 0; x < LayerWidth(); ++x) {
        const auto x_world = LayerToWorldX(x);
        const auto y_world = LayerToWorldY(y);
        double distance = 0.0;
        if(std::abs(x_world) < (half_world_width - half_world_height) ||
          (std::abs(x_world) - (half_world_width - half_world_height)) < std::abs(y_world))
        {
          distance = half_world_height - std::abs(y_world);
        } else {
          distance = half_world_width - std::abs(x_world);
        }

        row[x] = distance;
      }
    }
  }

  void FillLayerGpu(cv::Mat & layer, const World &) override
  {
    // TODO (Christian): Fix this :)
    // TODO (Christian): Abstract this out so we only copy this constant
    // once for all layers we want to run at once
    float* half_world_width = WorldWidth() / 2;
    float* half_world_height = WorldHeight() / 2;
    cudaMemcpyToSymbol(HALF_WORLD_WIDTH, half_world_width, sizeof(float));
    cudaMemcpyToSymbol(HALF_WORLD_HEIGHT, half_world_height, sizeof(float));

    int width = LayerWidth();
    int height = LayerHeight();
    size_t layerBytes = width * height * sizeof(float);

    float* h_Pin = layer.data;
    float* h_Pout = new float[width * height];

    float* d_Pin;
    float* d_Pout;

    cudaMalloc(&d_Pin, layerBytes);
    cudaMalloc(&d_Pout, layerBytes);

    cudaMemcpy(h_Pin, d_Pin, layerBytes, cudaMemcpyHostToDevice);

    launchDistanceFromEdgeKernel(h_Pout, d_Pout, width, height);

    cudaMemcpy(d_Pout, h_Pout, layerBytes, cudaMemcpyDeviceToHost);

    layer.data = h_Pout;

    // Not sure if the data gets copied into the layer, so might not want to delete this.
    delete[] h_Pout;
    // Do we need to free h_Pin (the pointer to the input array) instead/as well?
    cudaFree(d_Pin);
    cudaFree(d_Pout);
  }

private:
  cv::Size prev_size;

};

}  // namespace ateam_kenobi::spatial

#endif  // LAYERS__DISTANCE_FROM_FIELD_EDGE_HPP_
