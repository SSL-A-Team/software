#include <deque>
#include <vector>
#include <ssl_league_msgs/msg/vision_geometry_data.hpp>
#include <ssl_league_msgs/msg/vision_detection_ball.hpp>
#include <ssl_league_msgs/msg/vision_detection_robot.hpp>
#include <ssl_league_msgs/msg/vision_detection_frame.hpp>
#include "frame_info.hpp"
#include "filtered_ball.hpp"
#include "filtered_robot.hpp"

class Camera {
    public:   
        // Need to process an individual frame
        // Set geometry from VisionGeometryCameraCalibration.msg
        // Have a queue/buffer that we can remove old frames/have a set capacity
        Camera::Camera();

        void process_detection_frame(const ssl_league_msgs::msg::VisionDetectionFrame detection_frame_msg); 

        void process_camera_geometry(const ssl_league_msgs::msg::VisionGeometryData);

        void clear_old_messages();

        void process_balls(const ssl_league_msgs::msg::VisionDetectionFrame detection_frame_msg);

        void process_robots();

        void create_new_ball_track();

        void create_new_robot_track();
    
        std::deque<ssl_league_msgs::msg::VisionDetectionFrame> detection_queue;
        std::deque<ssl_league_msgs::msg::VisionGeometryData> geometry_queue;
    
    private:
        int camera_id;
        LastFrameInfo last_processed_frame;
        std::vector<FilteredBall> tracked_balls;
        std::vector<TrackedRobot> tracked_robots;
}