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

#include "ateam_common/parameters.hpp"

#include <functional>
#include <string>
#include <vector>
#include <iostream>

namespace ateam_common
{
void ParameterRegistry::RegisterParam(
  const std::string & parameter_name,
  Parameter<int> * parameter)
{
  if (int_params.count(parameter_name) > 0) {
    std::cout << "Multiple parameters with name " << parameter_name << std::endl;
  } else {
    int_params[parameter_name] = parameter;
  }
}

void ParameterRegistry::RegisterParam(
  const std::string & parameter_name,
  Parameter<double> * parameter)
{
  if (double_params.count(parameter_name) > 0) {
    std::cout << "Multiple parameters with name " << parameter_name << std::endl;
  } else {
    double_params[parameter_name] = parameter;
  }
}

void ParameterRegistry::RegisterParam(
  const std::string & parameter_name,
  Parameter<bool> * parameter)
{
  if (double_params.count(parameter_name) > 0) {
    std::cout << "Multiple parameters with name " << parameter_name << std::endl;
  } else {
    bool_params[parameter_name] = parameter;
  }
}

OnSetParamCBPtr ParameterRegistry::ApplyAllRegistrations(rclcpp::Node * node)
{
  for (const auto & param : int_params) {
    node->declare_parameter(param.first, static_cast<int>(*param.second));
  }
  for (const auto & param : double_params) {
    node->declare_parameter(param.first, static_cast<double>(*param.second));
  }
  for (const auto & param : bool_params) {
    node->declare_parameter(param.first, static_cast<bool>(*param.second));
  }

  return node->add_on_set_parameters_callback(
    std::bind(&ParameterRegistry::NodeParamCallback, this, std::placeholders::_1));
}

ParameterRegistry & ParameterRegistry::Get()
{
  static ParameterRegistry p;
  return p;
}

rcl_interfaces::msg::SetParametersResult ParameterRegistry::NodeParamCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  for (const auto & param : parameters) {
    int num_params_set = 0;
    num_params_set += int_params.count(param.get_name());
    num_params_set += double_params.count(param.get_name());
    num_params_set += bool_params.count(param.get_name());

    if (num_params_set == 1) {
      if (int_params.count(param.get_name()) > 0 &&
        param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
      {
        int_params.at(param.get_name())->set(param.as_int());
        continue;
      }
      if (double_params.count(param.get_name()) > 0 &&
        param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
      {
        double_params.at(param.get_name())->set(param.as_double());
        continue;
      }
      if (bool_params.count(param.get_name()) > 0 &&
        param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
      {
        bool_params.at(param.get_name())->set(param.as_bool());
        continue;
      }

      result.successful = false;
      result.reason = "Wrong type for parameter with name: " + param.get_name();
      break;
    } else {
      result.successful = false;
      result.reason = "Failed to find parameter with name: " + param.get_name();
      break;
    }
  }

  return result;
}

}  // namespace ateam_common
