#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <geometry_msgs/Point.h>
#include <vector> 
#include <cstdlib>
#include <cmath>
#include <numeric>

/**
 * This node averages the last 10 velocity values from the turtle and publishes them to a topic
 */

std::vector<float> errorVec; 
float currentAverage = 0.0; 
float currentMax = 0.0;

void average_max(float recentError)
{
  errorVec.push_back(std::abs(recentError));

  if(errorVec.size() > 1) {
   currentAverage = std::accumulate(errorVec.begin(),errorVec.end(),errorVec[0])/errorVec.size();
   currentMax = *(std::max_element(errorVec.begin(),errorVec.end()));
  }

  else { 
   currentAverage = std::abs(recentError); 
   currentMax = std::abs(recentError);
  }
  
}

void errorCallback(std_msgs::Float64 msg)
{
  average_max(msg.data);
  //ROS_INFO("Current Average: [%f]", currentAverage); 
  //ROS_INFO("Current Max: [%f]", currentMax);
}

int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "g3_average");

  ros::NodeHandle n;

  //Create a subscriber object and subscribe to pid_error
  ros::Subscriber sub = n.subscribe("pid_error", 10, errorCallback);

  //Create a publisher object and publish to the average_velocity node
  ros::Publisher error_pub = n.advertise<geometry_msgs::Point>("wall_following_analysis", 1000);

  // Publish at 5Hz
  ros::Rate loop_rate(5);

  // Publish the average velocity on the topic 
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    geometry_msgs::Point msg;

    msg.x = currentAverage; 
    msg.y = currentMax;
    msg.z = 0.0;

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    error_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}

