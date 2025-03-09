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

#ifndef CORE__SPATIAL__SPATIAL_MAP_COLLECTION_HPP_
#define CORE__SPATIAL__SPATIAL_MAP_COLLECTION_HPP_

#include <string>
#include <unordered_map>
#include <utility>
#include "spatial_map.hpp"

namespace ateam_kenobi::spatial
{

class SpatialMapCollection {
public:
  void AddMap(SpatialMap && map)
  {
    maps_.insert_or_assign(map.name, std::move(map));
  }

  void EmplaceMap(std::string name, cv::Mat data)
  {
    maps_.emplace(name, SpatialMap{name, data});
  }

  void Clear()
  {
    maps_.clear();
  }

  const SpatialMap & operator[](const std::string & name) const
  {
    return maps_.at(name);
  }

  SpatialMap & operator[](const std::string & name)
  {
    return maps_[name];
  }

private:
  std::unordered_map<std::string, SpatialMap> maps_;
};

}  // namespace ateam_kenobi::spatial

#endif  // CORE__SPATIAL__SPATIAL_MAP_COLLECTION_HPP_
