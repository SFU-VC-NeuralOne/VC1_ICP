//
// Created by rakesh on 13/08/18.
//
#include <cmath>
#include <map>
#include <numeric>
#include <chrono>

#include <boost/atomic/atomic.hpp>
#include <opencv2/flann/miniflann.hpp>

#include <icp_slam/icp_slam.h>
#include <icp_slam/utils.h>
#include <icp_slam/config.h>
#include <iostream>
using namespace std;

#define TIME_DIFF(tic, toc) ((std::chrono::duration<double, std::milli>((toc) - (tic))).count())

namespace icp_slam
{

ICPSlam::ICPSlam(tfScalar max_keyframes_distance, tfScalar max_keyframes_angle, double max_keyframes_time)
  : max_keyframes_distance_(max_keyframes_distance),
    max_keyframes_angle_(max_keyframes_angle),
    max_keyframes_time_(max_keyframes_time),
    last_kf_laser_scan_(new sensor_msgs::LaserScan()),
    is_tracker_running_(false),
    is_first_frame_(true)
{
  last_kf_tf_odom_laser_.stamp_ = ros::Time(0);
}

bool ICPSlam::track(const sensor_msgs::LaserScanConstPtr &laser_scan,
                    const tf::StampedTransform &current_frame_tf_odom_laser,
                    tf::StampedTransform &tf_map_laser)
{
  //laser_scan is pointer it might be change, might need to save the content

  if (is_tracker_running_)
  {
    ROS_WARN_THROTTLE(1.0, "Couldn't track frame, tracker already running");
    return false;
  }
  is_tracker_running_ = true;

  last_kf_tf_odom_laser_.frame_id_ = "odom";
  last_kf_tf_odom_laser_.child_frame_id_ = current_frame_tf_odom_laser.child_frame_id_;

  if(is_first_frame_){ //initialzation
    *last_kf_laser_scan_ = *laser_scan;
    last_kf_tf_odom_laser_=current_frame_tf_odom_laser;
    tf::StampedTransform tf_map_odom;
    tf_map_odom.frame_id_ = "map";
    tf_map_odom.child_frame_id_ = "odom";
    tf_map_odom.stamp_ = ros::Time::now();
    tf_map_odom.setOrigin(tf::Vector3(0, 0, 0));
    tf_map_odom.setRotation(tf::createQuaternionFromYaw(0.0));
    tf_map_laser = tf_map_odom;
    is_tracker_running_=false;
    is_first_frame_=false;
    return true;
  }
  if(isCreateKeyframe(current_frame_tf_odom_laser, last_kf_tf_odom_laser_)){
    cout<<"key frame created!!"<<endl;

    tf::Transform tf_estimation = current_frame_tf_odom_laser.inverse() * last_kf_tf_odom_laser_ ;
    cout<<"last x "<<last_kf_tf_odom_laser_.getOrigin().getX()<<"last y "<<last_kf_tf_odom_laser_.getOrigin().getY()<<endl;
    cout<<"current x "<<current_frame_tf_odom_laser.getOrigin().getX()<<"current y "<<current_frame_tf_odom_laser.getOrigin().getY()<<endl;
    cout<<"after x"<<(current_frame_tf_odom_laser*tf_estimation).getOrigin().getX()<<"after y "<<(current_frame_tf_odom_laser*tf_estimation).getOrigin().getY()<<endl;
    tf::Transform refined_tf = icpRegistration(last_kf_laser_scan_, laser_scan, tf_estimation);
    
    tf_map_laser = tf::StampedTransform(,
                                        current_frame_tf_odom_laser.stamp_,
                                         "map",
                                         "odom");

    
    last_kf_tf_odom_laser_ = current_frame_tf_odom_laser;
    *last_kf_laser_scan_ = *laser_scan;
  }
  is_tracker_running_ = false;
  return true;
  

  // TODO: find the pose of laser in map frame
  // if a new keyframe is created, run ICP
  // if not a keyframe, obtain the laser pose in map frame based on odometry update
}

bool ICPSlam::isCreateKeyframe(const tf::StampedTransform &current_frame_tf, const tf::StampedTransform &last_kf_tf) const 
{
  // cout<<"current frame "<<current_frame_tf.frame_id_<<"last frame "<<last_kf_tf.frame_id_<<endl;
  //cout<<"current frame child "<<current_frame_tf.child_frame_id_<<"last frame child "<<last_kf_tf.child_frame_id_<<endl;
  assert(current_frame_tf.frame_id_ == last_kf_tf.frame_id_);
  assert(current_frame_tf.child_frame_id_ == last_kf_tf.child_frame_id_);

  // TODO: check whether you want to create keyframe (based on max_keyframes_distance_, max_keyframes_angle_, max_keyframes_time_)
  bool is_keyframe = false;

  cv::Point2d a(current_frame_tf.getOrigin().getX(), current_frame_tf.getOrigin().getY());
  cv::Point2d b(last_kf_tf.getOrigin().getX(), last_kf_tf.getOrigin().getY());

  double distance = cv::norm(a-b);
  cout<<"a:"<<a<< " b:"<<b<<" distance:"<<distance<<endl;

  if(distance>=max_keyframes_distance_){
  return true;}

  double angle_now = tf::getYaw((current_frame_tf.getRotation()) * 180 / M_PI);
  double angle_previous = tf::getYaw((last_kf_tf.getRotation()) * 180 / M_PI);  
  double angle = 0.0;
  if (angle_now >= angle_previous){
    angle = angle_now-angle_previous;
    }else{map
      angle = angle_previous-angle_now;
      }

  if(angle>=max_keyframes_angle_){
    return true;
    }
  if((double)(current_frame_tf.stamp_.toSec() - last_kf_tf.stamp_.toSec()) >= max_keyframes_time_){
    return true;
  }

  return false;
} 

cv::Mat ICPSlam::closestPoints(cv::Mat &point_mat1,
                            cv::Mat &point_mat2,
                            std::vector<int> &closest_indices,
                            std::vector<float> &closest_distances_2)
{
  // uses FLANN for K-NN e.g. http://www.morethantechnical.com/2010/06/06/iterative-closest-point-icp-with-opencv-w-code/
  closest_indices = std::vector<int>(point_mat1.rows, -1);
  closest_distances_2 = std::vector<float>(point_mat1.rows, -1);


  cv::Mat multi_channeled_mat1;
  cv::Mat multi_channeled_mat2;

  point_mat1.convertTo(multi_channeled_mat1, CV_32FC2);
  point_mat2.convertTo(multi_channeled_mat2, CV_32FC2);

  cv::flann::Index flann_index(multi_channeled_mat2, cv::flann::KDTreeIndexParams(2));  // using 2 randomized kdtrees

  cv::Mat mat_indices(point_mat1.rows, 1, CV_32S);
  cv::Mat mat_dists(point_mat1.rows, 1, CV_32F);
  flann_index.knnSearch(multi_channeled_mat1, mat_indices, mat_dists, 1, cv::flann::SearchParams(64) );

  int* indices_ptr = mat_indices.ptr<int>(0);
  //float* dists_ptr = mat_dists.ptr<float>(0);
  cv::Mat xy_values(point_mat2.rows, 2, CV_32F);

  for (int i=0;i<mat_indices.rows;++i) {
  closest_indices[i] = indices_ptr[i];
  if(closest_indices[i] == -1){
  xy_values.row(i) = point_mat1.row(closest_indices[i]);
  }else{
  xy_values.row(i) = point_mat2.row(closest_indices[i]);
  }

  return xy_values;

}

  mat_dists.copyTo(cv::Mat(closest_distances_2));

  // ---------------------------- naive version ---------------------------- //
  // max allowed distance between corresponding points
//  const float max_distance = 0.5;
//
//  for (size_t i = 0, len_i = (size_t)point_mat1.rows; i < len_i; i++)
//  {
//    int closest_point_idx = -1;
//    float closest_distance_2 = std::pow(max_distance, 2.0f);
//
//    for (size_t j = 0, len_j = (size_t)point_mat2.rows; j < len_j; j++)
//    {
//      auto distance2 =
//        std::pow(point_mat2.at<float>(j, 0) - point_mat1.at<float>(i, 0), 2.0f)
//        + std::pow(point_mat2.at<float>(j, 1) - point_mat1.at<float>(i, 1), 2.0f);
//
//      if (distance2 < closest_distance_2)
//      {
//        closest_distance_2 = distance2;
//        closest_point_idx = (int)j;
//      }
//    }
//
//    if (closest_point_idx >= 0)
//    {
//      closest_indices[i] = closest_point_idx;
//      closest_distances_2[i] = closest_distance_2;
//    }
//  }
}

void ICPSlam::vizClosestPoints(cv::Mat &point_mat1,
                               cv::Mat &point_mat2,
                               const tf::Transform &T_2_1)
{
  assert(point_mat1.size == point_mat2.size);

  const float resolution = 0.005;

  float *float_array = (float*)(point_mat1.data);
  float size_m = std::accumulate(
    float_array, float_array + point_mat1.total(), std::numeric_limits<float>::min(),
    [](float max, float current)
    {
      return current > max ? current : max;
    }
  );
  // add some slack
  size_m += 0.5;

  int size_pix = (int)(size_m / resolution);

  cv::Mat img(
    size_pix,
    size_pix,
    CV_8UC3,
    cv::Scalar(0, 0, 0)
  );

  auto meters_to_pix = [&size_pix, resolution](float meters) {
    int pix = (int)(meters / resolution + size_pix / 2.0f);
    pix = std::max(0, pix);
    pix = std::min(size_pix - 1, pix);
    return pix;
  };

  cv::Mat transformed_point_mat2 = utils::transformPointMat(T_2_1.inverse(), point_mat2);

  for (size_t i = 0, len_i = (size_t)point_mat1.rows; i < len_i; i++)
  {
    float x1 = point_mat1.at<float>(i, 0);
    float y1 = point_mat1.at<float>(i, 1);
    float x2 = transformed_point_mat2.at<float>(i, 0);
    float y2 = transformed_point_mat2.at<float>(i, 1);

    auto pix_x1 = meters_to_pix(x1);
    auto pix_y1 = meters_to_pix(y1);
    auto pix_x2 = meters_to_pix(x2);
    auto pix_y2 = meters_to_pix(y2);

    cv::Point point1(pix_x1, pix_y1);
    cv::Point point2(pix_x2, pix_y2);

    cv::circle(img, point1, 5, cv::Scalar(0, 0, 255), -1);
    cv::circle(img, point2, 5, cv::Scalar(255, 0, 0), -1);

    cv::line(img, point1, point2, cv::Scalar(0, 255, 0), 2);
  }

  cv::Mat tmp;
  cv::flip(img, tmp, 0);
  cv::imwrite("/tmp/icp_laser.png", img);
}

tf::Transform ICPSlam::icpRegistration(const sensor_msgs::LaserScanConstPtr &laser_scan1,
                                    const sensor_msgs::LaserScanConstPtr &laser_scan2,
                                    const tf::Transform &T_2_1)
{
  //2*T21
  cv::Mat points1 = utils::laserScanToPointMat(laser_scan1);
  cv::Mat points2 = utils::laserScanToPointMat(laser_scan2);
  cv::Mat points2_new = utils::transformPointMat(T_2_1, points2);

  vizClosestPoints(points1, points2, T_2_1);

  std::vector<int> closest_indices;
  std::vector<float> closest_distances_2;
  cv::Mat points2_ordered = closestPoints(points1, points2, closest_indices, closest_distances_2);
  tf::Transform refined_T_2_1 = icpIteration(points1, points2_ordered);
  return refined_T_2_1;

}

tf::Transform ICPSlam::icpIteration(cv::Mat &point_mat1,
                                    cv::Mat &point_mat2) 
{
  // cv::Mat test(5,2,CV_32F);
  // for(int i=0; i<5; i++)
  // {
  //   test.at<float>(i,0)=i+0.5;
  //   test.at<float>(i,1)=i+1.5;
  // }
  // cv::Mat u1;
  // cv::reduce(test, u1, 0, CV_REDUCE_AVG);
  // cout<<"here is test !!!!!!!!!!!!!!!!!!!!!"<<test<<endl;
  // cout<<"here is test mean!!!!!!!!!!!!!!!!!!!!!"<<u1<<endl;
  // cv::Mat subs;
  // subtract(test,(cv::Scalar)(u1.at<float>(0,0), u1.at<float>(0,0)),subs);
  // cout<<"try substract"<<subs<<endl;

  
  cv::Mat ux;
  cv::reduce(point_mat1, ux, 0, CV_REDUCE_AVG);
  cv::Mat up;
  cv::reduce(point_mat1, up, 0, CV_REDUCE_AVG);
  
  cv::Mat x_prime;
  subtract(point_mat1,(cv::Scalar)(ux.at<float>(0,0), ux.at<float>(0,0)),x_prime);

  cv::Mat p_prime;
  subtract(point_mat2,(cv::Scalar)(up.at<float>(0,0), up.at<float>(0,0)),p_prime);
  
  cv::SVD svd(x_prime * p_prime.t());

  cv::Mat r = svd.u*svd.vt;
  cv::Mat t = ux - r * up;

  float rotation = atan2(r.at<float>(0,1), r.at<float>(0,0));

  tf::Transform refined_T_2_1;
  refined_T_2_1.setOrigin(tf::Vector3(t.at<float>(0,0),t.at<float>(0,1),0.0));
  refined_T_2_1.setRotation(tf::createQuaternionFromYaw(rotation));

  return refined_T_2_1;

}               

} // namespace icp_slam

