#include "youbot_driver/youbot/YouBotBase.hpp"
#include "youbot_driver/youbot/YouBotManipulator.hpp"
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <iostream>

//Distance as a float
float dist=0;


void distanceCallback(const std_msgs::Float32::ConstPtr& msg)
{
  ROS_INFO("distance: [%f]", msg->data);
  dist = msg->data;
}



using namespace youbot;

int main(int argc, char **argv) {

	
	// configuration flags for different system configuration (e.g. base without arm)
	bool youBotHasBase = false;
	bool youBotHasArm = false;

	// define velocities
	double vLinear = 0.05; //meter_per_second
	double vRotate = 0.2; //radian_per_second

	// create handles for youBot base and manipulator (if available)
	YouBotBase* myYouBotBase = 0;
	YouBotManipulator* myYouBotManipulator = 0;

	try {
		myYouBotBase = new YouBotBase("youbot-base", YOUBOT_CONFIGURATIONS_DIR);
		myYouBotBase->doJointCommutation();

		youBotHasBase = true;
	} catch (std::exception& e) {
		LOG(warning) << e.what();
		youBotHasBase = false;
	}

	try {
		myYouBotManipulator = new YouBotManipulator("youbot-manipulator", YOUBOT_CONFIGURATIONS_DIR);
		myYouBotManipulator->doJointCommutation();
		myYouBotManipulator->calibrateManipulator();

		youBotHasArm = true;
	} catch (std::exception& e) {
		LOG(warning) << e.what();
		youBotHasArm = false;
	}

	try {
		/*
		 * Simple sequence of commands to the youBot:
		 */

		if (youBotHasBase) {


			// Variable for the base.
			// Here "boost units" is used to set values, that means you have to set a value and a unit.
			quantity<si::velocity> vx = 0 * meter_per_second;
			quantity<si::velocity> vy = 0 * meter_per_second;
			quantity<si::angular_velocity> va = 0 * radian_per_second;

			ros::init(argc, argv, "distance");
  			ros::NodeHandle n;
  			ros::Subscriber sub = n.subscribe("distance", 1000, distanceCallback);
			 			
			
			while(ros::ok()){
				if (dist>15)
				{
				
				// forward
				LOG(info) << "drive forward";
				vx = vLinear * meter_per_second;
				vy = 0 * meter_per_second;
				myYouBotBase->setBaseVelocity(vx, vy, va);
				
				SLEEP_MILLISEC(2000);
				ros::spinOnce();

				}

			
				else {



				// stop base 
				vx = 0 * meter_per_second;
				vy = 0 * meter_per_second;
				va = 0 * radian_per_second;
				myYouBotBase->setBaseVelocity(vx, vy, va);
				LOG(info) << "stop base";
				ros::spinOnce();
				}
			}
					
			
		}

		if (youBotHasArm) {
			
		}

	} catch (std::exception& e) {
		std::cout << e.what() << std::endl;
		std::cout << "unhandled exception" << std::endl;
	}

	// clean up
	if (myYouBotBase) {
		delete myYouBotBase;
		myYouBotBase = 0;
	}
	if (myYouBotManipulator) {
		delete myYouBotManipulator;
		myYouBotManipulator = 0;
	}

	LOG(info) << "Done.";

	return 0;
}
