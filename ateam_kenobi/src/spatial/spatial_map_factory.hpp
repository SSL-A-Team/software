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

#ifndef SPATIAL__SPATIAL_MAP_FACTORY_HPP_
#define SPATIAL__SPATIAL_MAP_FACTORY_HPP_

#include <unordered_map>
#include <opencv2/core/mat.hpp>
#include "types/world.hpp"

namespace ateam_kenobi::spatial
{

class SpatialMapFactory {
public:
  SpatialMapFactory(std::string name) : name_(name) {}

  const std::string & GetName() const {
    return name_;
  }

  virtual void FillMap(cv::Mat & map, const World & world, const std::unordered_map<std::string, cv::Mat> & layers) = 0;

private:
  const std::string name_;

};

}  // namespace ateam_kenobi::spatial

#endif  // SPATIAL__SPATIAL_MAP_FACTORY_HPP_
