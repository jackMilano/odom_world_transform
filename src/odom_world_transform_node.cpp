// Il nodo crea ed aggiorna la trasformata tra 'world' e 'odom'.
// - non si tiene conto dell'orientamento
// - la trasformata viene aggiornata ogni volta che e' possibile, nei tempi morti in cui non si entra nella callback
//   viene pubblicata l'ultima calcolata, con lo stamp aggiornato, in modo da non fermare mai la pubblicazione della
//   trasformata, anche se a causa di falsi positivi o confidenza bassa o lost tracking il nodo 'bb_to_world' smette
//   di pubblicare messaggi

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
//#include <geometry_msgs/PoseWithCovarianceStamped.h>

// TF libraries
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <boost/bind.hpp>

#define RAD2DEG (180.0 / 3.14159265359)
#define SYNC_POLICY_WINDOW_SIZE 15

class OdomWorldTransformEstimator
{
  // Ho abbandonato 'ApproximateTime' per cercare di risolvere il problema del 'lookup would require extrapolation into
  // the future', ma se si usa 'ExactTime' non si entra mai nella callback.
  typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, nav_msgs::Odometry>
  SyncPolicy;

public:
  OdomWorldTransformEstimator(double rate_hz, const std::string& robot_base_frame);
  void spin();

private:
  void estimateTransformCallback(const geometry_msgs::PoseStampedConstPtr& pose, const nav_msgs::OdometryConstPtr& odom);

  ros::NodeHandle nh_;
  std::vector<geometry_msgs::PoseStampedConstPtr> poses_;
  ros::Rate rate_;
  message_filters::Synchronizer<SyncPolicy> pose_odom_sub_;
  message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub_;
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
  tf::TransformBroadcaster transform_bc_;
  tf::TransformListener tf_listener_;
  std::string base_frame_;
  bool old_transform_set;
  tf::Transform old_transform;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odom_world_transform_node");
  ros::NodeHandle nh("~");

  ROS_DEBUG("Nodo 'odom_world_transform' avviato.");

  double rate;
  //double calib_vel_x, calib_vel_y, calib_duration;
  //int calib_min_samples;
  std::string base;

  nh.param("rate", rate, 60.0);
  //nh.param("calibration_velocity_x", calib_vel_x, 0.0);
  //nh.param("calibration_velocity_y", calib_vel_y, 0.0);
  //nh.param("calibration_duration", calib_duration, 2.0);
  //nh.param("calibration_min_samples", calib_min_samples, 5);
  nh.getParam("robot_base_frame", base);

  OdomWorldTransformEstimator transformEstimator(rate, base);

  transformEstimator.spin();

  return 0;
}


// per i tipi di dato non POD viene automaticamente chiamato il costruttore di default, se l'oggetto non e' nell'initializer list
OdomWorldTransformEstimator::OdomWorldTransformEstimator(double rate_hz, const std::string& robot_base_frame)
  :
  rate_(rate_hz),
  base_frame_(robot_base_frame),
  pose_sub_(nh_, "robot_pose", 1),
  odom_sub_(nh_, "odom", 1),
  pose_odom_sub_(SyncPolicy(SYNC_POLICY_WINDOW_SIZE)),
  old_transform_set(false),
  old_transform()
{
  pose_odom_sub_.connectInput(pose_sub_, odom_sub_);
}

void OdomWorldTransformEstimator::spin()
{
  ROS_DEBUG("Nello 'spin'.");

  pose_odom_sub_.registerCallback(&OdomWorldTransformEstimator::estimateTransformCallback, this);

  while(nh_.ok())
  {
    // In questo modo anche se non riceviamo dati continuiamo a pubblicare una trasformata (l'ultima calcolata).
    if(old_transform_set)
    {
      tf::StampedTransform stamped_trans = tf::StampedTransform(old_transform.inverse(), ros::Time::now(), "world", "odom");
      transform_bc_.sendTransform(stamped_trans);
    }

    // 'spinOnce': will call all the callbacks waiting to be called at that point in time.
    // 'spinOnce': non dovrebbe essere bloccante.
    ros::spinOnce();
    rate_.sleep();
  }

  return;
}

void OdomWorldTransformEstimator::estimateTransformCallback(const geometry_msgs::PoseStampedConstPtr& pose,
    const nav_msgs::OdometryConstPtr& odom)
{
  ROS_DEBUG("Sono nella 'estimateTransformCallback'.");

  //if(old_transform_set && invalid_visual_tracker)
  //if(old_transform_set)
  //{
  //tf::StampedTransform stamped_trans = tf::StampedTransform(old_transform.inverse(), pose->header.stamp, "world", "odom");
  //transform_bc_.sendTransform(stamped_trans);

  //ROS_INFO("Publishing old transform.");

  //return;
  //}

  //TODO: online estimation of yaw offset here

  tf::Stamped<tf::Pose> odom_to_world;

  // pose of base_footprint wrt world
  tf::Transform base_to_world(tf::createQuaternionFromYaw(0.0), tf::Vector3(pose->pose.position.x, pose->pose.position.y,
                              0.0));

  // pose of world wrt base_footprint
  tf::Stamped<tf::Pose> world_to_base_stamped(base_to_world.inverse(), pose->header.stamp, base_frame_);

  try
  {
    // Prima di eseguire le trasformazioni aspettiamo che esista una trasformata tra odom e
    // base link (il tempo passato e' quello che vogliamo per la trasformata).
    //if(!tf_listener_.waitForTransform("odom", "base_link", odom->header.stamp, ros::Duration(3.0f)))
    if(!tf_listener_.waitForTransform("odom", "base_link", pose->header.stamp, ros::Duration(3.0f)))
    {
      ROS_ERROR("odom_world_transform: wait for transform timed out!!");

      return;
    }

    // pose of world wrt odom
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

  // salvataggio dati vecchi
  old_transform = transform;
  old_transform_set = true;

  //TODO: publish future stamped??
  // inverse to obtain pose of odom wrt world (world to odom transform)
  tf::StampedTransform stamped_trans = tf::StampedTransform(old_transform.inverse(), pose->header.stamp, "world", "odom");

  transform_bc_.sendTransform(stamped_trans);

  return;
}
