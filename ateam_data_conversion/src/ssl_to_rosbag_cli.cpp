#include <ateam_data_conversion/ssl_to_rosbag.hpp>
#include <cerrno>

int main(int argc, char **argv) {
    if (argc == 2) {
        ateam_data_conversion::SslToRosbag temp(argv[1]);
    else if (argc == 3) {
        ateam_data_conversion::SslToRosbag temp(argv[1], argv[2]);
        return 0;
    } else {
        return E2BIG;
    }
}