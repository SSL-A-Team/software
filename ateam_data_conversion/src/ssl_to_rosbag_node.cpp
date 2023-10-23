#include <ateam_data_conversion/ssl_to_rosbag.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

// literally just exists so someone wont complain to me about input options
namespace ateam_data_conversion
{

class SslToRosbagNode : public rclcpp::Node
{
public: // ctors
    explict SslToRosbagNode(const rclcpp::NodeOptions & options) : rclcpp::Node("ssl_to_rosbag", options) {
        SslToRosbag temp(
            declare_parameter<std::string>("input_ssl_filename", ""),
            declare_parameter<std::string>("out_bag_filename", "")
        );
    };
};

} // namespace

RCLCPP_COMPONENTS_REGISTER_NODE(ateam_data_conversion::SslToRosbagNode)

#endif // ATEAM_DATA_CONVERSION__SSL_TO_ROSBAG_HPP_