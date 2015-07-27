// Il nodo crea ed aggiorna la trasformata tra 'world' e 'odom'.

// Libraries
// ROS library
#include <ros/ros.h>

// ROS messages
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/connection.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <nav_msgs/Odometry.h>
#include <projected_game_msgs/Pose2DStamped.h>
#include <geometry_msgs/Twist.h>

// TF libraries
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <boost/bind.hpp>

#define SYNC_POLICY_WINDOW_SIZE 15
#define RAD2DEG 180.0 / 3.14159265359

class OdomWorldTransformEstimator
{
  typedef message_filters::sync_policies::ApproximateTime<projected_game_msgs::Pose2DStamped, nav_msgs::Odometry> SyncPolicy;

public:
  OdomWorldTransformEstimator(double rate_hz, const std::string & robot_base_frame);

  bool isYawOffsetCalibrated() const;
  double getYawOffset() const;
  bool calibrateYawOffset(const tf::Vector3 &velocity, double time_secs, int min_samples);
  void spin();

private:
  void yawOffsetCalibCallback(const projected_game_msgs::Pose2DStampedConstPtr& pose, const nav_msgs::OdometryConstPtr& odom);
  void estimateTransformCallback(const projected_game_msgs::Pose2DStampedConstPtr& pose, const nav_msgs::OdometryConstPtr& odom);
  tf::Vector3 findFittingVector(const std::vector<projected_game_msgs::Pose2DStampedConstPtr> &poses);

  ros::NodeHandle nh_;
  bool yaw_offset_calibrated_;
  std::vector<projected_game_msgs::Pose2DStampedConstPtr> poses_;
  ros::Rate rate_;
  double yaw_offset_;
  message_filters::Synchronizer<SyncPolicy> pose_odom_sub_;
  message_filters::Subscriber<projected_game_msgs::Pose2DStamped> pose_sub_;
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
  tf::TransformBroadcaster transform_bc_;
  tf::TransformListener tf_listener_;
  std::string base_frame_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odom_world_transform_node");
  ros::NodeHandle nh("~");

  double rate, calib_vel_x, calib_vel_y, calib_duration;
  int calib_min_samples;
  std::string base;

  nh.param("rate", rate, 50.0);
  nh.param("calibration_velocity_x", calib_vel_x, 0.0);
  nh.param("calibration_velocity_y", calib_vel_y, 0.0);
  nh.param("calibration_duration", calib_duration, 2.0);
  nh.param("calibration_min_samples", calib_min_samples, 5);
  nh.getParam("robot_base_frame", base);

  OdomWorldTransformEstimator transformEstimator(rate, base);

  if(transformEstimator.calibrateYawOffset(tf::Vector3(calib_vel_x, calib_vel_y, 0), calib_duration, calib_min_samples))
  {
    ROS_INFO("Yaw offset calibration successful! Yaw offset = %.4f deg. Starting to publish transform...",
        transformEstimator.getYawOffset() * RAD2DEG);
    transformEstimator.spin();
  }
  else
  {
    ROS_ERROR("Yaw offset calibration failed. Quitting...");
  }

  return 0;
}

OdomWorldTransformEstimator::OdomWorldTransformEstimator(double rate_hz, const std::string &robot_base_frame)
  : rate_(rate_hz), yaw_offset_calibrated_(false), yaw_offset_(0.0), base_frame_(robot_base_frame),
  pose_sub_(nh_, "robot_pose", 1), odom_sub_(nh_, "odom", 1), pose_odom_sub_(SyncPolicy(SYNC_POLICY_WINDOW_SIZE))
{
  pose_odom_sub_.connectInput(pose_sub_, odom_sub_);
}

bool OdomWorldTransformEstimator::isYawOffsetCalibrated() const
{
  return yaw_offset_calibrated_;
}

double OdomWorldTransformEstimator::getYawOffset() const
{
  return yaw_offset_;
}

bool OdomWorldTransformEstimator::calibrateYawOffset(const tf::Vector3 &velocity, double time_secs, int min_samples)
{
  if(velocity.length() < 0.01)
  {
    ROS_ERROR("Cannot calibrate with velocity < 0.01. Aborting.");
    return false;
  }

  geometry_msgs::Twist twist;
  tf::vector3TFToMsg(velocity, twist.linear);
  ros::Duration duration(time_secs);

  //register calibration callback
  message_filters::Connection conn = pose_odom_sub_.registerCallback(
      boost::bind(&OdomWorldTransformEstimator::yawOffsetCalibCallback, this, _1, _2));
  ros::Publisher cmd_vel_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  //wait for initial pose
  //start moving the robot
  ros::Time finish(ros::Time::now() + duration);
  while(ros::Time::now() < finish)
  {
    cmd_vel_pub.publish(twist);
    ros::spinOnce();
    rate_.sleep();
  }

  //stop the robot
  geometry_msgs::Twist zeroTwist;
  cmd_vel_pub.publish(zeroTwist);
  ros::spinOnce();

  //disconnect calibration callback
  conn.disconnect();

  //cut head and tail of poses vector (consider only the part where the robot was moving)
  //TODO use odom to decide where to cut more precisely?? (e.g. only the section with constant linear velocity)
  int toCut = (int) round(poses_.size() * 0.25);
  std::vector<projected_game_msgs::Pose2DStampedConstPtr>::iterator it = poses_.begin();
  it += toCut;
  poses_.erase(poses_.begin(), it);
  it = poses_.begin();
  it += poses_.size() - toCut;
  poses_.erase(it, poses_.end());

  //check enough samples were collected
  if(poses_.size() >= min_samples)
  {
    //calculate displacement vector
    tf::Vector3 actual_displacement = findFittingVector(poses_);

    //TODO can I use odom info?? should I??
    //maybe use odom instead of velocity for expected displacement?

    //calculate angle b/w displacement vectors
    //angle of 2 relative to 1: atan2(v2.y,v2.x) - atan2(v1.y,v1.x)
    //offset = angle of actual relative to expected
    yaw_offset_ = atan2(actual_displacement.y(), actual_displacement.x()) - atan2(velocity.y(), velocity.x());
    yaw_offset_calibrated_ = true;

    //we can delete these
    poses_.clear();
    return true;
  }
  else
  {
    ROS_ERROR("Not enough samples for calibration. We have %d samples.", (int) poses_.size());
    ROS_ERROR("Try providing a faster calibration velocity, a longer duration or a smaller minimum number of samples. Aborting.");
  }
  return false;
}

void OdomWorldTransformEstimator::spin()
{
  if(yaw_offset_calibrated_)
  {
    pose_odom_sub_.registerCallback(&OdomWorldTransformEstimator::estimateTransformCallback, this);
    while(nh_.ok())
    {
      ros::spinOnce();
      rate_.sleep();
    }
  }
  else
  {
    ROS_ERROR("Yaw offset is not calibrated yet. You must call calibrateYawOffset() first. Aborting...");
  }
}

void OdomWorldTransformEstimator::yawOffsetCalibCallback(const projected_game_msgs::Pose2DStampedConstPtr& pose,
    const nav_msgs::OdometryConstPtr& odom)
{
  poses_.push_back(pose);
}

void OdomWorldTransformEstimator::estimateTransformCallback(
    const projected_game_msgs::Pose2DStampedConstPtr& pose, const nav_msgs::OdometryConstPtr& odom)
{
  //should never happen because of the check in spin()
  ROS_ASSERT_MSG(yaw_offset_calibrated_, "In estimateTransformCallback() but yaw offset not calibrated.");

  //TODO online estimation of yaw offset here

  tf::Stamped<tf::Pose> odom_to_world;
  try
  {
    //pose of base_footprint wrt world
    tf::Transform base_to_world(tf::createQuaternionFromYaw(yaw_offset_), tf::Vector3(pose->pose.x, pose->pose.y, 0.0));
    //pose of world wrt base_footprint
    tf::Stamped<tf::Pose> world_to_base_stamped(base_to_world.inverse(), pose->header.stamp, base_frame_);
    //pose of world wrt odom
    tf_listener_.transformPose(odom->header.frame_id, world_to_base_stamped, odom_to_world);
  }
  catch(tf::TransformException)
  {
    ROS_DEBUG("Failed to subtract base to odom transform");
    return;
  }

  tf::Transform transform(tf::Quaternion(odom_to_world.getRotation()),
      tf::Point(odom_to_world.getOrigin()));

  //TODO publish future stamped??
  //inverse to obtain pose of odom wrt world (world to odom transform)
  transform_bc_.sendTransform(tf::StampedTransform(transform.inverse(), pose->header.stamp, pose->header.frame_id,
        odom->header.frame_id));
}

// regression
tf::Vector3 OdomWorldTransformEstimator::findFittingVector(const std::vector<projected_game_msgs::Pose2DStampedConstPtr> &poses)
{
  double sum_x = 0;       //sum of x values
  double sum_y = 0;       //sum of y values
  double sum_xy = 0;      //sum of x * y
  double sum_xx = 0;      //sum of x^2
  double slope = 0;       //slope of regression line
  double y_intercept = 0; //y intercept of regression line
  double avg_y = 0;       //mean of y
  double avg_x = 0;       //mean of x

  //calculate various sums
  for (int i = 0; i < poses.size(); ++i)
  {
    //sum of x
    sum_x += poses[i]->pose.x;
    //sum of y
    sum_y += poses[i]->pose.y;
    //sum of squared x*y
    sum_xy += poses[i]->pose.x * poses[i]->pose.y;
    //sum of squared x
    sum_xx += poses[i]->pose.x * poses[i]->pose.x;
  }

  //calculate the means of x and y
  avg_y = sum_y / poses.size();
  avg_x = sum_x / poses.size();

  //slope or a1
  slope = (poses.size() * sum_xy - sum_x * sum_y) / (poses.size() * sum_xx - sum_x * sum_x);

  //y itercept or a0
  y_intercept = avg_y - slope * avg_x;

  //fitting line equation: y = y_intercept + slope * x
  //calculate vectors for points at x = 0 and x = 1
  tf::Vector3 x0(0, y_intercept, 0), x1(1, y_intercept + slope, 0);

  //return in vector form, make sure direction is preserved
  if(poses[0]->pose.x < poses[poses.size()-1]->pose.x) return x1 - x0;
  else return x0 - x1;
}
