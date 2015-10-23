/// Il nodo crea ed aggiorna la trasformata tra 'world' e 'odom'.
// - non si tiene conto dell'orientamento
// - la trasformata viene calcolata solamente all'inizio, e poi costantemente pubblicata

// Libraries
// ROS library
#include <ros/ros.h>

// ROS messages
#include <message_filters/connection.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>

// TF libraries
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <boost/bind.hpp>

#define MAX_WAIT_TIME 3 // secondi di attesa
#define RAD2DEG (180.0 / 3.14159265359)
#define SYNC_POLICY_WINDOW_SIZE 15


class OdomWorldTransformEstimator
{
  // Utilizzando 'ExactTime' non si entra mai nella callback.
  typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, nav_msgs::Odometry>
  SyncPolicy;

public:
  OdomWorldTransformEstimator(double rate_hz, const std::string& robot_base_frame);

  bool isYawOffsetCalibrated() const;
  double getYawOffset() const;
  bool calibrateYawOffset(const tf::Vector3& velocity, double time_secs, int min_samples);
  void spin();

private:
  void yawOffsetCalibCallback(const geometry_msgs::PoseStampedConstPtr& pose,
                              const nav_msgs::OdometryConstPtr& odom);
  void estimateTransformCallback(const geometry_msgs::PoseStampedConstPtr& pose,
                                 const nav_msgs::OdometryConstPtr& odom);
  tf::Vector3 findFittingVector(const std::vector<geometry_msgs::PoseStampedConstPtr>& poses);

  ros::NodeHandle nh_;
  std::vector<geometry_msgs::PoseStampedConstPtr> poses_;
  ros::Rate rate_;
  message_filters::Synchronizer<SyncPolicy> pose_odom_sub_;
  message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub_;
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
  tf::TransformBroadcaster transform_bc_;
  tf::TransformListener tf_listener_;
  std::string base_frame_;
  tf::Transform old_transform;
  const ros::Duration WAIT_BEFORE_STATIC_PUB;
  bool yaw_offset_calibrated_;
  double yaw_offset_;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "odom_world_transform_node");
  ros::NodeHandle nh("~");

  ROS_INFO("Nodo 'odom_world_transform' avviato!");

  double rate;
  double calib_vel_x, calib_vel_y, calib_duration;
  int calib_min_samples;
  std::string base;

  nh.param("rate", rate, 60.0);
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


// Per i tipi di dato non POD viene automaticamente chiamato il costruttore di default, se l'oggetto non e' nell'initializer list
OdomWorldTransformEstimator::OdomWorldTransformEstimator(double rate_hz, const std::string& robot_base_frame)
  :
  rate_(rate_hz),
  base_frame_(robot_base_frame),
  pose_sub_(nh_, "robot_pose", 1),
  odom_sub_(nh_, "odom", 1),
  pose_odom_sub_(SyncPolicy(SYNC_POLICY_WINDOW_SIZE)),
  old_transform(),
  WAIT_BEFORE_STATIC_PUB(MAX_WAIT_TIME),
  yaw_offset_calibrated_(false),
  yaw_offset_(0.0)//,
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

bool OdomWorldTransformEstimator::calibrateYawOffset(const tf::Vector3& velocity, double time_secs, int min_samples)
{
  if(velocity.length() < 0.01)
  {
    ROS_ERROR("Cannot calibrate with velocity < 0.01. Aborting.");
    return false;
  }

  geometry_msgs::Twist twist;
  tf::vector3TFToMsg(velocity, twist.linear);
  ros::Duration duration(time_secs);

  // register calibration callback
  message_filters::Connection conn = pose_odom_sub_.registerCallback(boost::bind(
                                       &OdomWorldTransformEstimator::yawOffsetCalibCallback, this, _1, _2));

  ros::Publisher cmd_vel_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  // wait for initial pose
  // start moving the robot
  ros::Time finish(ros::Time::now() + duration);

  while(ros::Time::now() < finish)
  {
    cmd_vel_pub.publish(twist);
    ros::spinOnce();
    rate_.sleep();
  }

  // stop the robot
  geometry_msgs::Twist zeroTwist;
  cmd_vel_pub.publish(zeroTwist);
  ros::spinOnce();

  // disconnect calibration callback
  conn.disconnect();

  // cut head and tail of poses vector (consider only the part where the robot was moving)
  //TODO: use odom to decide where to cut more precisely?? (e.g. only the section with constant linear velocity)
  ROS_DEBUG("'poses_.size()' = %d.", (int) poses_.size());
  int toCut = (int) round(poses_.size() * 0.25);
  std::vector<geometry_msgs::PoseStampedConstPtr>::iterator it = poses_.begin();
  it += toCut;
  poses_.erase(poses_.begin(), it);
  it = poses_.begin();
  it += poses_.size() - toCut;
  poses_.erase(it, poses_.end());

  // check enough samples were collected
  if(poses_.size() >= min_samples)
  {
    // calculate displacement vector
    tf::Vector3 actual_displacement = findFittingVector(poses_);

    //TODO: can I use odom info?? should I??
    // maybe use odom instead of velocity for expected displacement?

    // calculate angle b/w displacement vectors
    // angle of 2 relative to 1: atan2(v2.y,v2.x) - atan2(v1.y,v1.x)
    // offset = angle of actual relative to expected
    yaw_offset_ = atan2(actual_displacement.y(), actual_displacement.x()) - atan2(velocity.y(), velocity.x());
    yaw_offset_calibrated_ = true;

    //we can delete these
    poses_.clear();

  }
  else
  {
    ROS_ERROR("Not enough samples for calibration.");
    ROS_ERROR("We have %d samples. 'min_samples' = %d.", (int) poses_.size(), min_samples);
    ROS_ERROR("Try providing a faster calibration velocity, a longer duration or a smaller minimum number of samples.");
    ROS_ERROR("Aborting.");
  }

  return yaw_offset_calibrated_;
}

void OdomWorldTransformEstimator::spin()
{
  ROS_DEBUG("I'm in the 'spin'.");

  if(yaw_offset_calibrated_)
  {
    //pose_odom_sub_.registerCallback(&OdomWorldTransformEstimator::estimateTransformCallback, this);
    message_filters::Connection conn = pose_odom_sub_.registerCallback(
                                         &OdomWorldTransformEstimator::estimateTransformCallback, this);

    ROS_INFO("Attendiamo %.2f secondi prima di pubblicare la trasformata statica!", WAIT_BEFORE_STATIC_PUB.toSec());

    // Per i primi 'WAIT_BEFORE_STATIC_PUB' continuiamo a produrre dinamicamente trasformate, poi la salviamo.
    const ros::Time start_time = ros::Time::now();
    ros::Duration duration(0.0);

    while(duration < WAIT_BEFORE_STATIC_PUB)
    {
      // 'spinOnce': will call all the callbacks waiting to be called at that point in time; it's not blocking.
      ros::spinOnce();
      rate_.sleep();
      duration = ros::Time::now() - start_time;
    }

    conn.disconnect();

    ROS_INFO("Sono passati %.2f secondi! Ora pubblichiamo la trasformata statica!", WAIT_BEFORE_STATIC_PUB.toSec());

    while(nh_.ok())
    {
      // In questo modo quando 'bb_to_world' non pubblica i messaggi di posa (a causa di falsi positivi o lost tracking
      // o bassa confidenza, si continua pero' a pubblicare l'ultima trasformata valida).
      // - sul lungo periodo le posizioni pero' divergono
      tf::StampedTransform stamped_trans = tf::StampedTransform(old_transform.inverse(), ros::Time::now(), "world", "odom");
      transform_bc_.sendTransform(stamped_trans);

      // 'spinOnce': will call all the callbacks waiting to be called at that point in time; it's not blocking.
      ros::spinOnce();
      rate_.sleep();
    }
  }

  return;
}

void OdomWorldTransformEstimator::yawOffsetCalibCallback(const geometry_msgs::PoseStampedConstPtr& pose,
    const nav_msgs::OdometryConstPtr& odom)
{
  //ROS_DEBUG("Sono nella 'yawOffsetCalibCallback' callback.");
  poses_.push_back(pose);

  return;
}

void OdomWorldTransformEstimator::estimateTransformCallback(const geometry_msgs::PoseStampedConstPtr& pose,
    const nav_msgs::OdometryConstPtr& odom)
{
  // should never happen because of the check in spin()
  ROS_ASSERT_MSG(yaw_offset_calibrated_, "In estimateTransformCallback() but yaw offset not calibrated.");

  ROS_DEBUG("Sono nella 'estimateTransformCallback'.");

  // pose of base_footprint wrt world (world --> base_footprint transform)
  tf::Transform base_to_world(tf::createQuaternionFromYaw(yaw_offset_), tf::Vector3(pose->pose.position.x,
                              pose->pose.position.y, 0.0));

  // pose of world wrt base_footprint (base_footprint --> world transform)
  // - header = 'base_link'
  tf::Stamped<tf::Pose> world_to_base_stamped(base_to_world.inverse(), pose->header.stamp, base_frame_);

  tf::Stamped<tf::Pose> odom_to_world;

  try
  {
    // Prima di eseguire la trasformazione aspettiamo che esista la transform 'odom --> base link'
    // al tempo desiderato
    if(!tf_listener_.waitForTransform("odom", "base_link", pose->header.stamp, ros::Duration(3.0f)))
    {
      ROS_ERROR("odom_world_transform: wait for transform timed out!!");
      return;
    }

    // pose of world wrt odom (odom --> world transform)
    tf_listener_.transformPose(odom->header.frame_id, world_to_base_stamped, odom_to_world);
  }
  catch(const tf::TransformException& tf_ex)
  {
    ROS_ERROR("'odom_world' tf_exception: %s", tf_ex.what());
    ros::Duration(1.0).sleep();
  }
  catch(const std::exception& ex)
  {
    ROS_ERROR("'odom_world' exception: %s", ex.what());
    ros::Duration(1.0).sleep();
  }

  tf::Transform transform(tf::Quaternion(odom_to_world.getRotation()), tf::Point(odom_to_world.getOrigin()));

  // salvataggio trasformata
  old_transform = transform;

  //TODO: publish future stamped??
  // inverse to obtain pose of odom wrt world (world --> odom transform)
  tf::StampedTransform stamped_trans = tf::StampedTransform(old_transform.inverse(), pose->header.stamp, "world", "odom");

  transform_bc_.sendTransform(stamped_trans);

  return;
}


// regression
tf::Vector3 OdomWorldTransformEstimator::findFittingVector(const std::vector<geometry_msgs::PoseStampedConstPtr>& poses)
{
  double sum_x = 0;       // sum of x values
  double sum_xx = 0;      // sum of x^2
  double sum_xy = 0;      // sum of x * y
  double sum_y = 0;       // sum of y values

  // calculate various sums
  for(int i = 0; i < poses.size(); ++i)
  {
    // sum of x
    sum_x += poses[i]->pose.position.x;
    // sum of y
    sum_y += poses[i]->pose.position.y;
    // sum of squared x*y
    sum_xy += poses[i]->pose.position.x * poses[i]->pose.position.y;
    // sum of squared x
    sum_xx += poses[i]->pose.position.x * poses[i]->pose.position.x;
  }

  // calculate the means of x and y
  const double avg_y = sum_y / poses.size();
  const double avg_x = sum_x / poses.size();

  // slope or a1: is the slope of the regression line
  const double slope = (poses.size() * sum_xy - sum_x * sum_y) / (poses.size() * sum_xx - sum_x * sum_x);

  // y intercept or a0: y intercept of regression line
  const double y_intercept = avg_y - slope * avg_x;

  // fitting line equation: y = y_intercept + slope * x
  // calculate vectors for points at x = 0 and x = 1
  const tf::Vector3 x0(0, y_intercept, 0);
  const tf::Vector3 x1(1, y_intercept + slope, 0);
  tf::Vector3 fitting_vector;

  // return in vector form, make sure direction is preserved
  if(poses[0]->pose.position.x < poses[poses.size() - 1]->pose.position.x)
  {
    // return x1 - x0;
    fitting_vector = x1 - x0;
  }
  else
  {
    // return x0 - x1;
    fitting_vector = x0 - x1;
  }

  return fitting_vector;
}
