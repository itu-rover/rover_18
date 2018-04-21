#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <utility>
#include <vector>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <robot_localization/navsat_conversions.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <rover_state_mach/RoverStateMsg.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <math.h>


// initialize variables

typedef actionlib::SimpleActionClient <move_base_msgs::MoveBaseAction>
MoveBaseClient; //create a type definition for a client called MoveBaseClient

class GpsWaypoint
{
private:
	MoveBaseClient ac;

	ros::NodeHandle gw_nh;
	ros::Publisher sm_pub; //State machine sublisher
	ros::Subscriber sm_sub; //State machine subscriber
	ros::Subscriber tg_sub; //Target GPS waypoint subscriber
	ros::Subscriber cg_sub; //Current GPS point subscriber 
	ros::Rate rate;

	rover_state_mach::RoverStateMsg roverState;
	move_base_msgs::MoveBaseGoal moveBaseGoal;
	int count = 0, wait_count = 0;
	geometry_msgs::PointStamped UTM_point, map_point, UTM_next, map_next;
	double latiG, longG, latiC, longC;
	std::string utm_zone;
	bool targetFlag = false; // flag for the activating waypoint handler

public:
  	GpsWaypoint() : ac("move_base", true),
  				sm_pub(gw_nh.advertise<std_msgs::String>("/gps_waypoint_handler/status", 10)), //TODO: convert to actual topic and type
  				sm_sub(gw_nh.subscribe<rover_state_mach::RoverStateMsg>("/rover_state_topic",10, &GpsWaypoint::stateMachineCB, this)), //TODO:  convert to state machine msgs type and topic
  				tg_sub(gw_nh.subscribe<sensor_msgs::NavSatFix>("/rover_gps/waypoint", 10, &GpsWaypoint::targetPointCB, this)),
  				cg_sub(gw_nh.subscribe<sensor_msgs::NavSatFix>("/gps/fix", 10, &GpsWaypoint::currentPointCB, this)),
  				rate(1)
	{
    	while(!ac.waitForServer(ros::Duration(5.0)))
   		{
       		wait_count++;
       		if(wait_count > 3)
       		{
        		ROS_ERROR("move_base action server did not come up, killing gps_waypoint_handler node...");
          		// Notify joy_launch_control that waypoint following is complete
           		std_msgs::Bool node_ended;
           		node_ended.data = true;
           		//pubWaypointNodeEnded.publish(node_ended); //TODO: send notification to state machine with same format
           		ros::shutdown();
       		}
       		ROS_INFO("Waiting for the move_base action server to come up");
   		}
   		roverState.state = rover_state_mach::RoverStateMsg::INITIALISE;
  	}

	//Called once when the goal completes
	void doneCb(const actionlib::SimpleClientGoalState& state,
				const move_base_msgs::MoveBaseResultConstPtr& result)
	{
    	ROS_INFO("Finished in state ");
    	//ROS_INFO("Answer: %i", result->sequence.back());
    	//ros::shutdown();
    }

	// Called once when the goal becomes active
  	void activeCb()
  	{
  		ROS_INFO("Goal just went active");
  	}

  	// Called every time feedback is received for the goal
  	void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
  	{
  		ROS_INFO("Got Feedback of move base ");
  		//ROS_INFO("Got Feedback of length %lu", feedback->sequence.size());
  		//geometry_msgs/PoseStamped base_position
  	}

  	void stateMachineCB(rover_state_mach::RoverStateMsg rvrStt)
  	{
  		ROS_INFO("Got Info from State Machine ");
  		roverState.state = rvrStt.state;
  	}

  	void targetPointCB(sensor_msgs::NavSatFix targetPoint )
  	{
  		ROS_INFO("Got Info about target  ");
  		//Gönderilen hedef koordinatları al. lat=long long=lat
    	//check the message
    	if (latiG != targetPoint.longitude)
    	{
        	latiG=targetPoint.longitude;
        	longG=targetPoint.latitude;

        	ROS_INFO("I got new target points");

        	targetFlag = true;
    
        	ROS_INFO("flag is :%s", targetFlag?"true":"false");
    	}
    	else 
    	{
    		ROS_WARN("Same target point has been sent. NO ACTION !");
    	}
  	}

  	void currentPointCB(sensor_msgs::NavSatFix currentPoint )
  	{
  		ROS_INFO("Got Info about current point ");
  		longC=currentPoint.longitude;
  		latiC=currentPoint.latitude;
  	}

  	geometry_msgs::PointStamped latLongtoUTM(double lati_input, double longi_input)
  	{
    	double utm_x = 0, utm_y = 0;
    	geometry_msgs::PointStamped UTM_point_output;

    	//convert lat/long to utm
    	RobotLocalization::NavsatConversions::LLtoUTM(lati_input, longi_input, utm_y, utm_x, utm_zone);

    	//Construct UTM_point and map_point geometry messages
    	UTM_point_output.header.frame_id = "utm";
    	UTM_point_output.header.stamp = ros::Time(0);
    	UTM_point_output.point.x = utm_x;
    	UTM_point_output.point.y = utm_y;
    	UTM_point_output.point.z = 0;

    	return UTM_point_output;
  	}

  	geometry_msgs::PointStamped UTMtoMapPoint(geometry_msgs::PointStamped UTM_input)
  	{
    	geometry_msgs::PointStamped map_point_output;
    	bool notDone = true;
    	tf::TransformListener listener; //create transformlistener object called listener
    	ros::Time time_now = ros::Time::now();
    	while(notDone)
    	{
        	try
        	{
            	UTM_point.header.stamp = ros::Time::now();
            	listener.waitForTransform("odom", "utm", time_now, ros::Duration(3.0));
            	listener.transformPoint("odom", UTM_input, map_point_output);
            	notDone = false;
        	}
        	catch (tf::TransformException& ex)
        	{
            	ROS_WARN("%s", ex.what());
            	ros::Duration(0.01).sleep();
            	//return;
        	}
    	}
    	return map_point_output;
  	}

  	move_base_msgs::MoveBaseGoal buildGoal(geometry_msgs::PointStamped map_point, geometry_msgs::PointStamped map_next)
  	{
    	move_base_msgs::MoveBaseGoal goal;

    	//Specify what frame we want the goal to be published in
    	goal.target_pose.header.frame_id = "odom";
    	goal.target_pose.header.stamp = ros::Time::now();

    	// Specify x and y goal
    	goal.target_pose.pose.position.x = map_next.point.x; //specify x goal !!changed to map_next from map_point
    	goal.target_pose.pose.position.y = map_next.point.y; //specify y goal

    	// Specify heading goal using current goal and next goal (point robot towards its next goal once it has achieved its current goal)
   
    	tf::Matrix3x3 rot_euler;
    	tf::Quaternion rot_quat;

    	// Calculate quaternion
    	float x_curr = map_point.point.x, y_curr = map_point.point.y; // set current coords.
    	float x_next = map_next.point.x, y_next = map_next.point.y; // set coords. of next waypoint
    	float delta_x = x_next - x_curr, delta_y = y_next - y_curr;   // change in coords.
    	float yaw_curr = 0, pitch_curr = 0, roll_curr = 0;
    	yaw_curr = atan2(delta_y, delta_x);

    	// Specify quaternions
    	rot_euler.setEulerYPR(yaw_curr, pitch_curr, roll_curr);
    	rot_euler.getRotation(rot_quat);

    	goal.target_pose.pose.orientation.x = rot_quat.getX();
    	goal.target_pose.pose.orientation.y = rot_quat.getY();
    	goal.target_pose.pose.orientation.z = rot_quat.getZ();
    	goal.target_pose.pose.orientation.w = rot_quat.getW();

	    return goal;
  	}

  	float calculateError()
	{
		//Get current map points
    	UTM_point = latLongtoUTM(latiC, longC);
		map_point = UTMtoMapPoint(UTM_point);

		float error = sqrt((map_next.point.x-map_point.point.x)+(map_next.point.y-map_point.point.y));
    	
    	return error;
	}

  	void doStuff()
  	{
  		while(ros::ok())
    	{
        	if (roverState.state == rover_state_mach::RoverStateMsg::READY)
        	{
        		if (targetFlag == true)
        		{       
            		ROS_INFO("Received current coordinates latitude:%.8f, longitude:%.8f", latiC, longC);
            		ROS_INFO("Received goal coordinates latitude:%.8f, longitude:%.8f", latiG, longG);

            		//Convert lat/long to utm:
            		UTM_point = latLongtoUTM(latiC, longC);
            		UTM_next = latLongtoUTM(latiG, longG);

            		//Transform UTM to map point in odom frame
            		map_point = UTMtoMapPoint(UTM_point);
            		map_next = UTMtoMapPoint(UTM_next);

            		//Build goal to send to move_base
            		moveBaseGoal = buildGoal(map_point, map_next); //initiate a move_base_msg called moveBaseGoal

            		// Send Goal
            		ROS_INFO("Sending goal");
            		ac.sendGoal(moveBaseGoal,
        	    				boost::bind(&GpsWaypoint::doneCb, this, _1, _2),
           	    				boost::bind(&GpsWaypoint::activeCb, this),
                				boost::bind(&GpsWaypoint::feedbackCb, this, _1));

            		std_msgs::String sm_info;
            		sm_info.data = "1";
            		sm_pub.publish(sm_info);  //Send info to SM, we have sended goal to move base

            		//Wait for result 
            		//ROS_INFO("Wait for result");
            		//ac.waitForResult(); //waiting to see if move_base was able to reach goal
            		//TODO: STATE MACHİNE MANTIĞINA GÖRE BURALARI DÜZENLE
            
            		targetFlag = false;
            		ROS_INFO("flag is :%s", targetFlag?"true":"false");
        		}
        		else
        		{
        			ROS_INFO("Rover is ready for GPS waypoint navigation but there is no target point.\nSend target position.");
        		}	
        	}

        	else if (roverState.state == rover_state_mach::RoverStateMsg::REACH_GPS)
        	{
        		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            	{
            		float error = calculateError();
                	ROS_INFO("Rover has reached its goal! with error: %f",error);
            	}
            	else
            	{
                	ROS_WARN("Goal not succeded yet.\n");
            	}
        	}
        	else  //Out of gps waypoint state, if there is an active goal from this node cancel it respect to
        	{
        		/* önceden gönderilen goalları yok et.?Goal ID olmadı flag kullan*/
        	}
        	rate.sleep();
        	ros::spinOnce();
    }
	    
	}
};
//***End of GpsWaypoint Class definition***

int main (int argc, char **argv)
{
  	ros::init(argc, argv, "gps_waypoint_handler");
  	GpsWaypoint waypointHandler;
  	waypointHandler.doStuff();
  	return 0;
}