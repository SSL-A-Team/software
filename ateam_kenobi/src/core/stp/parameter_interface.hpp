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


#ifndef STP__PARAMETER_INTERFACE_HPP_
#define STP__PARAMETER_INTERFACE_HPP_

#include <string>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>

namespace ateam_kenobi::stp
{

struct UninitializedParameterInterface : public std::runtime_error
{
  UninitializedParameterInterface()
  : std::runtime_error("ParameterInterface used before being properly initialized.") {}
};

class ParameterInterface
{
public:
  /**
   * @brief Default constructor
   * @note This constructor does not give you a usable parameter interface. It is included to allow
   * for temporary, paritially initialized instances while the system is getting set up. Make sure
   * to use the full parameterized constructor below before calling any member functions on this
   * object.
   */
  ParameterInterface() = default;

  ParameterInterface(
    std::string param_namespace,
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr ros_interface)
  : namespace_(param_namespace),
    ros_interface_(ros_interface) {}

  const std::string & getNamespace()
  {
    return namespace_;
  }

  std::string getNamespacedName(const std::string & name)
  {
    return namespace_ + "." + name;
  }

  ParameterInterface getChild(const std::string & name)
  {
    return ParameterInterface(getNamespacedName(name), ros_interface_);
  }

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr addSetParameterCallback(
    rclcpp::node_interfaces::NodeParametersInterface::OnSetParametersCallbackType callback)
  {
    throwIfUninitialized();
    return ros_interface_->add_on_set_parameters_callback(callback);
  }

  void removeSetParameterCallback(rclcpp::node_interfaces::OnSetParametersCallbackHandle * handle)
  {
    throwIfUninitialized();
    ros_interface_->remove_on_set_parameters_callback(handle);
  }

  bool hasParameter(const std::string & name)
  {
    throwIfUninitialized();
    return ros_interface_->has_parameter(getNamespacedName(name));
  }

  template<typename ParameterT>
  ParameterT declareParameter(const std::string & name, const ParameterT & default_value)
  {
    throwIfUninitialized();

    return ros_interface_->declare_parameter(
      getNamespacedName(name),
      rclcpp::ParameterValue(default_value)).get<ParameterT>();
  }

  template<typename ParameterT>
  ParameterT getParameter(const std::string & name)
  {
    throwIfUninitialized();
    return ros_interface_->get_parameter(getNamespacedName(name)).get_value<ParameterT>();
  }

private:
  std::string namespace_;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr ros_interface_;

  void throwIfUninitialized()
  {
    if (!ros_interface_) {
      throw UninitializedParameterInterface();
    }
  }
};

}  // namespace ateam_kenobi::stp

#endif  // STP__PARAMETER_INTERFACE_HPP_
