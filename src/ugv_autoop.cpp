
/*******************************************************************************
*adaptacion de turtlebot3_drive
*******************************************************************************/

/*#include "ugvAutoop_gazebo/ugv_autoop.h"*/
#include <ugv_simulator/ugv_autoop.h>
UgvAutoop::UgvAutoop()
  : nh_priv_("~")
{
  //Init gazebo ros ugvAutoop node
  ROS_INFO("Autonomus UGV Node Init");
  auto ret = init();
  ROS_ASSERT(ret);
}

UgvAutoop::~UgvAutoop()
{
  updatecommandVelocity(0.0, 0.0);
  ros::shutdown();
}

/*******************************************************************************
* Init function
*******************************************************************************/
bool UgvAutoop::init()
{
  // initialize ROS parameter
  std::string cmd_vel_topic_name = nh_.param<std::string>("cmd_vel_topic_name", "");

  // initialize variables
  escape_range_       = 150 * DEG2RAD;
  check_forward_dist_ = 0.7;
  check_side_dist_    = 1;
  backing = 50;
  iterations = backing;
  ugv_pose_ = 0.0;
  prev_ugv_pose_ = 0.0;

  // initialize publishers
  cmd_vel_pub_   = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_name, 10);

  // initialize subscribers
  laser_scan_sub_  = nh_.subscribe("plctolaser/scan", 10, &UgvAutoop::laserScanMsgCallBack, this);
  odom_sub_ = nh_.subscribe("odom", 10, &UgvAutoop::odomMsgCallBack, this);

  return true;
}

void UgvAutoop::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
  double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
	double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);  

	ugv_pose_ = atan2(siny, cosy);
}

void UgvAutoop::laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  uint16_t scan_angle[3] = {0, 30, 330};

  for (int num = 0; num < 3; num++)
  {
    if (std::isinf(msg->ranges.at(scan_angle[num])))
    {
      scan_data_[num] = msg->range_max;
      /*ROS_INFO("scan_data_: %d", num);
      ROS_INFO(" %f", scan_data_[num]);*/
    }
    else
    {
      scan_data_[num] = msg->ranges.at(scan_angle[num]);
    }
  }
}

void UgvAutoop::updatecommandVelocity(double linear, double angular)
{
  geometry_msgs::Twist cmd_vel;

  cmd_vel.linear.x  = linear;
  cmd_vel.angular.z = angular;

  cmd_vel_pub_.publish(cmd_vel);
}

/*******************************************************************************
* Control Loop function
*******************************************************************************/
bool UgvAutoop::controlLoop()
{
  static uint8_t ugv_state_num = 0;

  switch(ugv_state_num)
  {
    case GET_UGV_DIRECTION:
      //retrocede backing veces segun cuantos sleeping avanzó
      if (sleeping == 3000)
            {
            ugv_state_num = UGV_OP_BACKWARD;
            ROS_INFO("UGV se durmio");
            ROS_INFO("tratando de despertarlo...");
            break;
          }

      if (scan_data_[CENTER] > check_forward_dist_)
      {
        if (scan_data_[LEFT] < check_side_dist_)
        {
          prev_ugv_pose_ = ugv_pose_;
          ugv_state_num = UGV_RIGHT_TURN;
          ROS_INFO("girando a la derecha");
          sleeping = 0;
        }
        else if (scan_data_[RIGHT] < check_side_dist_)
        {
          prev_ugv_pose_ = ugv_pose_;
          ugv_state_num = UGV_LEFT_TURN;
          ROS_INFO("girando a la izquierda");
          sleeping = 0;
        }
        else
        {
          ugv_state_num = UGV_DRIVE_FORWARD;
          ROS_INFO("avanzando");
        }
      }

      if (scan_data_[CENTER] < check_forward_dist_)
      {
        prev_ugv_pose_ = ugv_pose_;
        ugv_state_num = UGV_RIGHT_TURN;
        ROS_INFO("girando a la derecha");
        sleeping = 0;
      }
      break;

    case UGV_OP_BACKWARD:
      if (iterations == 0)
      {
        iterations = backing;
        sleeping = 0;
        updatecommandVelocity(0.0, ANGULAR_VELOCITY);
        ugv_state_num = UGV_LEFT_TURN;
      }else{
        updatecommandVelocity( -1 * LINEAR_VELOCITY, 1.5 * ANGULAR_VELOCITY);
        ugv_state_num = GET_UGV_DIRECTION;
        iterations--;
      }
      break;

    case UGV_DRIVE_FORWARD:
        updatecommandVelocity(LINEAR_VELOCITY, 0.0);
        ugv_state_num = GET_UGV_DIRECTION;
        sleeping++;
        ROS_INFO("veces: %d", sleeping);
      break;

    case UGV_RIGHT_TURN:
      if (fabs(prev_ugv_pose_ - ugv_pose_) >= escape_range_)
        ugv_state_num = GET_UGV_DIRECTION;
      else
        updatecommandVelocity(0.0, -1 * ANGULAR_VELOCITY);
      break;

    case UGV_LEFT_TURN:
      if (fabs(prev_ugv_pose_ - ugv_pose_) >= escape_range_)
        ugv_state_num = GET_UGV_DIRECTION;
      else
        updatecommandVelocity(0.0, ANGULAR_VELOCITY);
      break;

    default:
      ugv_state_num = GET_UGV_DIRECTION;
      break;
  }

  return true;
}

/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "ugv_autoop");
  UgvAutoop ugv_autoop;

  ros::Rate loop_rate(125);

  while (ros::ok())
  {
    ugv_autoop.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}