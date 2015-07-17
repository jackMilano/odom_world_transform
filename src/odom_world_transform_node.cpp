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

#include <boost/bind.hpp>

#define SYNC_POLICY_WINDOW_SIZE 15
#define RAD2DEG 180.0 / 3.14159265359

class OdomWorldTransformEstimator
{
typedef message_filters::sync_policies::ApproximateTime<projected_game_msgs::Pose2DStamped, nav_msgs::Odometry> SyncPolicy;

public:
  OdomWorldTransformEstimator(double rate_hz = 50.0, double update_period_sec = 1.0)
  : rate_(rate_hz), update_period_(update_period_sec), theta_offset_calibrated_(false), theta_offset_(0.0),
    pose_sub_(nh_, "robot_pose", 1), odom_sub_(nh_, "odom", 1), pose_odom_sub_(SyncPolicy(SYNC_POLICY_WINDOW_SIZE))
  {
    pose_odom_sub_.connectInput(pose_sub_, odom_sub_);
  }
  
bool isThetaOffsetCalibrated() const
{
  return theta_offset_calibrated_;
}

double getThetaOffset() const
{
  return theta_offset_;
}

bool calibrateThetaOffset(const tf::Vector3 &velocity, double time_secs, double min_duration_coeff = 0.5)
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
  ROS_INFO("registering calibration callback");
  message_filters::Connection conn = pose_odom_sub_.registerCallback(boost::bind(&OdomWorldTransformEstimator::thetaOffsetCalibCallback, this, _1, _2));
  ros::Publisher cmd_vel_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  //wait for initial pose
  ROS_INFO("waiting for pose");
  while(!initial_pose_odom_received_ && nh_.ok())
  {
    ros::spinOnce();
    rate_.sleep();
  }

  if(nh_.ok())
  {
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

    //just to be sure, but shouldn't happen
    ROS_ASSERT_MSG(initial_pose_->header.frame_id == current_pose_->header.frame_id, "Mismatching reference frames for initial and current pose");
    ROS_ASSERT_MSG(initial_odom_->header.frame_id == current_odom_->header.frame_id, "Mismatching reference frames for initial and current odometry");

    //check if enough time has elapsed b/w initial pose reading and current one
    if(ros::Duration(current_pose_->header.stamp - initial_pose_->header.stamp) < duration * min_duration_coeff)
    {
      ROS_ERROR("Not enough time has passed between initial and final pose readings. Try supplying a longer calibration duration parameter.");
      return false;
    }

    //calculate displacement vector
    tf::Vector3 actual_displacement = tf::Vector3(current_pose_->pose.x, current_pose_->pose.y, 0) - tf::Vector3(initial_pose_->pose.x, initial_pose_->pose.y, 0);
    //TODO test this
    tf::Vector3 actual_displacement_regres = findFittingVector(poses_);

    //TODO can I use odom info?? should I??
    //maybe use odom instead of velocity for expected displacement?  

    //calculate angle b/w displacement vectors
    //angle of 2 relative to 1: atan2(v2.y,v2.x) - atan2(v1.y,v1.x)
    //offset = angle of expected relative to actual
    theta_offset_ = atan2(velocity.y(), velocity.x()) - atan2(actual_displacement.y(), actual_displacement.x());
    //TODO test this
    ROS_INFO("Angle from regression: %.4f deg.", (atan2(velocity.y(), velocity.x()) - atan2(actual_displacement_regres.y(), actual_displacement_regres.x())) * RAD2DEG);

    theta_offset_calibrated_ = true;
    
    //we can delete these
    poses_.clear();
    initial_pose_.reset();
    current_pose_.reset();
    initial_odom_.reset();
    current_odom_.reset();
    return true;
  }
  
  return false;
}

void spin()
{
  if(theta_offset_calibrated_)
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
    ROS_ERROR("Theta offset is not calibrated yet. You must call calibrateThetaOffset() first. Aborting...");
  }
}

private:
  void thetaOffsetCalibCallback(const projected_game_msgs::Pose2DStampedConstPtr& pose, const nav_msgs::OdometryConstPtr& odom)
  {
    ROS_INFO("calib cb");
    if(!initial_pose_odom_received_)
    {
      initial_pose_ = pose;
      initial_odom_ = odom;
      initial_pose_odom_received_ = true;
      ROS_INFO("initial pose recvd");
    }
    
    current_pose_ = pose;
    current_odom_ = odom;
    poses_.push_back(pose);
  }

  void estimateTransformCallback(const projected_game_msgs::Pose2DStampedConstPtr& pose, const nav_msgs::OdometryConstPtr& odom)
  {
    //should never happen because of the check in spin()
    ROS_ASSERT_MSG(theta_offset_calibrated_, "In estimateTransformCallback() but theta offset not calibrated.");
    
    //TODO online estimation of theta offset here
    
    tf::Transform transform;
    ros::Time now(ros::Time::now());
    //if we need to update
    if(now - last_update_time_ >= update_period_)
    {
      double offset_x = pose->pose.x - odom->pose.pose.position.x;
      double offset_y = pose->pose.y - odom->pose.pose.position.y;
      

      //tf::Quaternion rotation;
      //rotation.setRPY(0, 0, theta_offset_);
      //transform.setRotation(rotation);
      transform.setOrigin(tf::Vector3(offset_x, offset_y, 0.0));
      transform.setRotation(tf::Quaternion::getIdentity());
      
      ROS_DEBUG("Publishing transform from %s to %s with origin (%.3f, %.3f) and yaw %.3f", pose->header.frame_id.c_str(), odom->header.frame_id.c_str(), offset_x, offset_y, theta_offset_);
      transform_bc_.sendTransform(tf::StampedTransform(transform, now, pose->header.frame_id, odom->header.frame_id));
      last_update_time_ = now;
    }
  }
  
  tf::Vector3 findFittingVector(const std::vector<projected_game_msgs::Pose2DStampedConstPtr> &poses)
  {
    double sum_x = 0;     //sum of x values
    double sum_y = 0;     //sum of y values
    double sum_xy = 0;    //sum of x * y
    double sum_xx = 0;    //sum of x^2
    double slope = 0;    //slope of regression line
    double y_intercept = 0; //y intercept of regression line
    double avg_y = 0;     //mean of y
    double avg_x = 0;     //mean of x
    
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

    //return vector form of equation
    return x1 - x0;
  }

  ros::NodeHandle nh_;
  bool theta_offset_calibrated_, initial_pose_odom_received_;
  projected_game_msgs::Pose2DStampedConstPtr initial_pose_, current_pose_;
  nav_msgs::OdometryConstPtr initial_odom_, current_odom_;
  std::vector<projected_game_msgs::Pose2DStampedConstPtr> poses_;
  ros::Duration update_period_;
  ros::Rate rate_;
  double theta_offset_;
  ros::Time last_update_time_;
  message_filters::Synchronizer<SyncPolicy> pose_odom_sub_;
  message_filters::Subscriber<projected_game_msgs::Pose2DStamped> pose_sub_;
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
  tf::TransformBroadcaster transform_bc_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odom_world_transform_node");
  
  ros::NodeHandle nh("~");
  
  double rate, update_period, calib_vel_x, calib_vel_y, calib_duration, min_calib_coeff;

  nh.param("rate", rate, 50.0);
  nh.param("update_period", update_period, 1.0);
  nh.param("calibration_velocity_x", calib_vel_x, 0.0);
  nh.param("calibration_velocity_y", calib_vel_y, 0.0);
  nh.param("calibration_duration", calib_duration, 2.0);
  nh.param("min_calibration_duration_coeff", min_calib_coeff, 0.5);
  
  OdomWorldTransformEstimator transformEstimator(rate, update_period);
  
  if(transformEstimator.calibrateThetaOffset(tf::Vector3(calib_vel_x, calib_vel_y, 0), calib_duration, min_calib_coeff))
  {
    ROS_INFO("Theta offset calibration successful! Theta offset = %.4f deg. Starting to publish transform...", transformEstimator.getThetaOffset() * RAD2DEG);
    transformEstimator.spin();
  }
  else
  {
    ROS_ERROR("Theta offset calibration failed. Quitting...");
  }
  
  return 0;
}
















// Typedefs and Enums
// 'ApproximateTime' policy uses an adaptive algorithm to match messages based on their timestamp.
// XXX: attenzione sta venendo utilizzata la politica 'ApproximateTime'.


// Global variables
//bool first_boot;
//const ros::Duration update_period(UPDATE_PERIOD);
//ros::Time last_update_time;
//tf::Transform last_transform;


//void publishWorldOdomTransformCb(
//const projected_game_msgs::Pose2DStamped::ConstPtr& pose_2d_stamped,
//const nav_msgs::Odometry::ConstPtr& odometry)
//{
//static tf::TransformBroadcaster transform_broadcaster;

//ROS_INFO("Sono entrato nella callback di 'publishWorldOdomTransformCb'.");

//// Aggiorniamo la trasformata se:
//// - è passato abbastanza tempo
//// - la confidenza è abbastanza alta
//const ros::Duration time_spent = ros::Time::now() - last_update_time;
//if( (time_spent >= update_period /*&& pose_2d_stamped->confidence > MIN_CONFIDENCE*/) || first_boot )
//{

//ROS_INFO("Aggiornamento della trasformata.");

//first_boot = false; // We need to initialize 'last_transform' at least once.

//// La differenza tra la posa del robot in 'world' e la posa del robot in 'odom'
////  corrisponde (teoricamente) alla differenza tra le origini dei due sistemi
////  di riferimenti.
//tf::Transform transform;

//const float offset_x = odometry->pose.pose.position.x - pose_2d_stamped->pose.x;
//const float offset_y = odometry->pose.pose.position.y - pose_2d_stamped->pose.y;
////const float offset_z = odometry->pose.pose.position.z - pose_2d_stamped->pose.z;
//// Viene settata la posizione del sistema di riferimento 'odom'
////  rispetto al sistema di riferimento 'world'.
//transform.setOrigin( tf::Vector3(offset_x, offset_y, 0.0) );

////FIXME: per il momento la rotazione è ignorata
////tf::Quaternion quaternion;
////XXX: questo theta ha senso?
////quaternion.setRPY(0, 0, odometry->pose.pose.orientation. - pose_2d_stamped->theta);
//// Viene settato l'orientamento del sistema di riferimento 'odom'
////  rispetto al sistema di riferimento 'world'.
////transform.setRotation(quaternion);

//last_transform = transform;

//// Azzeriamo la variabile 'time_spent'.
//last_update_time = ros::Time::now();
//}

//// Broadcast della trasformata.
//try
//{
//tf::StampedTransform stamped_transform = tf::StampedTransform(last_transform, ros::Time::now(), "world", "odom");
//transform_broadcaster.sendTransform(stamped_transform);
//ROS_INFO("Teoricamente è appena stata inviata la 'stamped_transform'.");
//}
//catch(const tf::TransformException& tf_ex)
//{
//ROS_ERROR("%s", tf_ex.what());
//ros::Duration(1.0).sleep();
//}
//catch(const std::exception& ex)
//{
//ROS_ERROR("%s", ex.what());
//ros::Duration(1.0).sleep();
//}

//return;
//}

//int main(int argc, char **argv)
//{
//ros::init(argc, argv, "odom_world_transform_node");
//if(argc!=1)
//{
//ROS_WARN("need no arguments!");
//}
//ROS_INFO("odom_world_transform_node has started");
//ros::NodeHandle node_handle;

//first_boot = true;
//last_update_time = ros::Time::now();

//// Sincronizzazione tra 2 canali di ingresso
//// - la posa 2d del robot restituita dal tracking visuale
//// - l'odometria del robot stesso
//message_filters::Subscriber<projected_game_msgs::Pose2DStamped> pose_2d_sub(node_handle, "robot_2d_pose", 1);
//message_filters::Subscriber<nav_msgs::Odometry> odom_sub_(node_handle, "robot_odom", 1);
//message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), pose_2d_sub, odom_sub_);

//sync.registerCallback( boost::bind(&publishWorldOdomTransformCb, _1, _2) );

//sleep(3); //sleep 3 seconds.

//// 'ros::Rate' makes a best effort at mantaining a particular rate for a loop
////  WARNING: pay attention to the rate
//ros::Rate loop_rate(30);
//ros::spin();

//return 0;
//}
