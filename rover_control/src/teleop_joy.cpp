//written by Çağatay Yürük 08.05.2018
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>


class teleop_rover
{
public:
  teleop_rover();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  int linear_, angular_,throttle_,kill_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;

  bool sent_disable_msg;
};


teleop_rover::teleop_rover():
  linear_(1),
  angular_(2),
  throttle_(3),
  kill_(0)
{

  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  //7nh_.param("scale_angular", a_scale_, a_scale_);
  //nh_.param("scale_linear", l_scale_, l_scale_);

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 1, &teleop_rover::joyCallback, this);

  sent_disable_msg = false;

}

void teleop_rover::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist twist;
  if (joy->buttons[kill_])
  {    
    twist.angular.z = (1+joy->axes[throttle_])*joy->axes[angular_]; //TODO: scale ekle
    twist.linear.x = (1+joy->axes[throttle_])*joy->axes[linear_];
    vel_pub_.publish(twist);
    sent_disable_msg = false;
  }
  else
  {
    if (!sent_disable_msg)
    {
      vel_pub_.publish(twist);
      sent_disable_msg = true;
    }    
  }
  

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_rover");
  teleop_rover teleop_rover;

  ros::spin();
}
