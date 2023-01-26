// Copyright 2021 A Team
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

#ifndef ATEAM_COMMON__PARAMETERS_HPP_
#define ATEAM_COMMON__PARAMETERS_HPP_

#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

using OnSetParamCBPtr = rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr;

// CREATE_PARAM(double, "ilqr/constants/", kMaxIterations, 100);
// Must do outside a class
#define CREATE_PARAM(type, ros_namespace, param_name, default_value) \
  ateam_common::Parameter<type> param_name(default_value, ros_namespace #param_name)

// Must do first in node constructor
#define REGISTER_NODE_PARAMS(node_ptr) \
  static OnSetParamCBPtr param_cb_handler = \
    ateam_common::ParameterRegistry::Get().ApplyAllRegistrations(node_ptr);

namespace ateam_common
{
template<typename T>
class Parameter;

/**
 * Keep track of all the parameters that exists. When connected to a ros node,
 * registers those parameters and updates the specific parameters when they change
 */
class ParameterRegistry
{
public:
  // Registers a parameter with some fully qualified ros parameter name
  void RegisterParam(const std::string & parameter_name, Parameter<int> * parameter);
  void RegisterParam(const std::string & parameter_name, Parameter<double> * parameter);
  void RegisterParam(const std::string & parameter_name, Parameter<bool> * parameter);

  // Registers all known params with the given ros node
  // Do this first inside the constructor for a ros node to construct
  // all the param bindings correctly as well as register the callbacks correctly
  OnSetParamCBPtr ApplyAllRegistrations(rclcpp::Node * node);

  static ParameterRegistry & Get();

private:
  ParameterRegistry() = default;

  // Normal ros node param callback
  rcl_interfaces::msg::SetParametersResult NodeParamCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  std::unordered_map<std::string, Parameter<int> *> int_params;
  std::unordered_map<std::string, Parameter<double> *> double_params;
  std::unordered_map<std::string, Parameter<bool> *> bool_params;
};

// Type of parameter that the user is exposed to
template<typename T>
class Parameter
{
public:
  Parameter(const T & default_value, const std::string & ros_name)
  : t_{default_value}
  {
    ParameterRegistry::Get().RegisterParam(ros_name, this);
  }

  operator T() const {
    return t_;
  }

private:
  // Only allow the callback to set the value of the param after initialization
  friend class ParameterRegistry;
  void set(const T & new_value)
  {
    t_ = new_value;
  }

  T t_;
};

}  // namespace ateam_common

#endif  // ATEAM_COMMON__PARAMETERS_HPP_
