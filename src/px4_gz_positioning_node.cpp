/* MIT License
*
* Copyright (c) 2025 Manuel Domínguez
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/

#include "rclcpp/rclcpp.hpp"
#include <gz/msgs/pose.pb.h>
#include <gz/transport/Node.hh>
#include <px4_msgs/msg/vehicle_odometry.hpp>


/**
 * @class PX4GZPositioningNode
 * @brief This class injects the pose from gazebo pose publisher to PX4 Vehicle Visual Odometry topic
 */
class PX4GZPositioningNode : public rclcpp::Node {
public:
  PX4GZPositioningNode() : Node("px4_gz_positioning_node") {
    
    this->declare_parameter("gz_topic", "/model/x500_vision_1/pose");
		this->declare_parameter("px4_topic", "/px4_1/fmu/in/vehicle_visual_odometry");

    const std::string gz_topic = this->get_parameter("gz_topic").as_string();
		const std::string px4_topic = this->get_parameter("px4_topic").as_string();

    pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(px4_topic, 10);
    RCLCPP_INFO(this->get_logger(), "PX4 GZ positioning node starts.");

    gz_node_.Subscribe(
      gz_topic,  
      &PX4GZPositioningNode::gz_pose_callback,
      this
    );
  }

private:
  void gz_pose_callback(const gz::msgs::Pose &gz_msg) {
    
    px4_msgs::msg::VehicleOdometry px4_msg;

    px4_msg.pose_frame = px4_msg.POSE_FRAME_FRD;
    px4_msg.timestamp = uint64_t(gz_msg.header().stamp().sec())*1000000 + uint64_t(gz_msg.header().stamp().nsec())/1000;
    px4_msg.timestamp_sample = px4_msg.timestamp;

    px4_msg.position[0] = gz_msg.position().y();
    px4_msg.position[1] = gz_msg.position().x();
    px4_msg.position[2] = -gz_msg.position().z();

    px4_msg.q[0] = gz_msg.orientation().w(); 
    px4_msg.q[1] = gz_msg.orientation().y(); 
    px4_msg.q[2] = gz_msg.orientation().x(); 
    px4_msg.q[3] = -gz_msg.orientation().z();

    pub_ -> publish(px4_msg);
    
    RCLCPP_DEBUG(this->get_logger(), "Vehicle Odometry pose published.");
  }  

  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr pub_;
  gz::transport::Node gz_node_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PX4GZPositioningNode>());
  rclcpp::shutdown();
  return 0;
}


