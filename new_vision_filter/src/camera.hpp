#include <deque>
#include <ssl_league_msgs/msg/vision_geometry_data.hpp>

class Camera {
    public:   
        // Need to process an individual frame
        // Set geometry from VisionGeometryCameraCalibration.msg
        // Have a queue/buffer that we can remove old frames/have a set capacity
        Camera::Camera(
            int camera_id
        );

        void process_detection_frame();

        void process_camera_geometry();

        void clear_old_messages();

        void process_balls();

        void process_robots();

        void create_new_ball_track();

        void create_new_robot_track();
    
        std::deque<ssl_league_msgs::msg::VisionDetectionFrame> detection_queue;
        std::deque<ssl_league_msgs::msg::VisionGeometryData> geometry_queue;
    
        private:
        int camera_id;
}