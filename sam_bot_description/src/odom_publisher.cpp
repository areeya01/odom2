#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


class OdomPublisher : public rclcpp::Node 
{
  public:
    OdomPublisher()
    : Node("odom_publisher"), count_(0)
    {
      
      // initialize publisher with message type of nav_msgs/Odometry message type and topic name of example/odom
      odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("example/odom", 50);
      
      double x=0.0;
      double y=0.0;
      double th=0.0;
      
      double vx=0.1;
      double vy=-0.1;
      double vth=0.1;
      
      rclcpp::Time current_time, last_time;
      current_time = rclcpp::Node::now();
      last_time = rclcpp::Node::now();
      
      rclcpp::Rate r(1.0);
      while(rclcpp::ok()){
      
        current_time = rclcpp::Node::now();
        
        // compute odometry based on velocity
        double dt = (current_time - last_time).seconds();
        double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
        double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
        double delta_th = vth * dt;
        
        x += delta_x;
        y += delta_y;
        th += delta_th;
        
        // create quaternion msg from yaw
  	tf2::Quaternion q;
	q.setRPY(0,0,th);
	geometry_msgs::msg::Quaternion odom_quat=tf2::toMsg(q);
	
      	 // message declaration
        nav_msgs::msg::Odometry odom;
      	odom.header.stamp = current_time;
        odom.header.frame_id="odom";
        odom.child_frame_id="base_link";
        
      	// set the position
      	odom.pose.pose.position.x=x;
      	odom.pose.pose.position.y=y;
      	odom.pose.pose.position.z=0.0;
      	odom.pose.pose.orientation = odom_quat;
      	
      	//set the velocity
      	odom.twist.twist.linear.x = vx;
      	odom.twist.twist.linear.y = vy;
      	odom.twist.twist.angular.z = vth;
      	
      	// publish odometry over ROS
      	odom_pub_->publish(odom);
      	
      	last_time = current_time;
      	r.sleep();
      }
      
    }
   
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    size_t count_;
  };

  int main(int argc, char * argv[])
  {	
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomPublisher>());
    rclcpp::shutdown();
    return 0;
  }
