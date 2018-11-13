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
    //tf::StampedTransform tf_map_odom;
    // tf_map_odom.frame_id_ = "map";
    // tf_map_odom.child_frame_id_ = laser_scan->header.frame_id;
    // tf_map_odom.stamp_ = ros::Time::now();
    // tf_map_odom.setOrigin(tf::Vector3(0, 0, 0));
    // tf_map_odom.setRotation(tf::createQuaternionFromYaw(0.0));
    tf_map_laser = current_frame_tf_odom_laser;
    tf_map_laser.frame_id_ = "map";
    tf_map_laser.child_frame_id_ = laser_scan->header.frame_id;
    last_kf_tf_map_laser_ = tf_map_laser;
    is_tracker_running_=false;
    is_first_frame_=false;
       return true;
  }
  if(isCreateKeyframe(current_frame_tf_odom_laser, last_kf_tf_odom_laser_)){
    cout<<"key frame created!!"<<endl;

    tf::Transform tf_estimation = last_kf_tf_odom_laser_.inverse() * current_frame_tf_odom_laser;
    cout<<"last x "<<last_kf_tf_odom_laser_.getOrigin().getX()<<"last y "<<last_kf_tf_odom_laser_.getOrigin().getY()<<" "<<tf::getYaw(last_kf_tf_odom_laser_.getRotation()) * 180 / M_PI<<endl;
    cout<<"current x "<<current_frame_tf_odom_laser.getOrigin().getX()<<"current y "<<current_frame_tf_odom_laser.getOrigin().getY()<<" "<<tf::getYaw(current_frame_tf_odom_laser.getRotation()) * 180 / M_PI<<endl;
    cout<<"after x"<<(current_frame_tf_odom_laser*tf_estimation.inverse()).getOrigin().getX()<<"after y "<<(current_frame_tf_odom_laser*tf_estimation.inverse()).getOrigin().getY()<<" "<<tf::getYaw(current_frame_tf_odom_laser*tf_estimation.inverse().getRotation()) * 180 / M_PI<<endl;
    tf::Transform refined_tf = icpRegistration(last_kf_laser_scan_, laser_scan, tf_estimation);

    tf_map_laser = tf::StampedTransform(last_kf_tf_map_laser_*refined_tf,
                                        current_frame_tf_odom_laser.stamp_,
                                         "map",
                                         laser_scan->header.frame_id);

    last_kf_tf_odom_laser_ = current_frame_tf_odom_laser;
    last_kf_tf_map_laser_ = tf_map_laser;
    *last_kf_laser_scan_ = *laser_scan;
    is_tracker_running_ = false;
    return true;
  }
  else
  {
    tf::Transform tf_estimation = last_kf_tf_odom_laser_.inverse() * current_frame_tf_odom_laser ;
    tf_map_laser = tf::StampedTransform(last_kf_tf_map_laser_*tf_estimation,
                                        current_frame_tf_odom_laser.stamp_,
                                        "map",
                                         laser_scan->header.frame_id);
  }
  is_tracker_running_ = false;
  return true;
  

  // TODO: find the pose of laser in map frame
  // if a new keyframe is created, run ICP
  // if not a keyframe, obtacv::Mat points1in the laser pose in map frame based on odometry update
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
  //cout<<"a:"<<a<< " b:"<<b<<" distance:"<<distance<<endl;

  if(distance>=max_keyframes_distance_){
  return true;}

  double angle_now = tf::getYaw((current_frame_tf.getRotation()) * 180 / M_PI);
  double angle_previous = tf::getYaw((last_kf_tf.getRotation()) * 180 / M_PI);  
  double angle = angle_now-angle_previous;
  // if (angle_now >= angle_previous){
  //   angle = angle_now-angle_previous;
  //   }else{
  //     angle = angle_previous-angle_now;
  //     }

  if(abs(angle)>=max_keyframes_angle_){
    return true;
    }
  if((double)(current_frame_tf.stamp_.toSec() - last_kf_tf.stamp_.toSec()) >= max_keyframes_time_){
    return true;
  }

  return false;
} 

void ICPSlam::intersectionPoints(cv::Mat &point_mat1,
                                    cv::Mat &point_mat2,
                                    std::vector<int> &closest_indices,
                                   std::vector<float> &closest_distances_2,
                                   cv::Mat &points1_out,
                                    cv::Mat &points2_out)
{float mean;
  float std_dev;
  utils::meanAndStdDev(closest_distances_2, mean, std_dev);

  // cv::Mat intersection_point_mat1; // = point_mat1.at(i);
  // cv::Mat intersection_point_mat2; // = point_mat2.at(closest_indices.at(i));

  cv::Mat mat_indices(point_mat1.rows, 1, CV_32S);

  int j = 0;
    // for (auto i = closest_indices.begin(); i != closest_indices.end(); ++i)
    // std::cout << *i << ' ';
  for (int i=0;i<mat_indices.rows;++i) {
    if(closest_distances_2[i] <= mean+2*(std_dev) || closest_distances_2[i] >= mean-2*(std_dev)){
      if((point_mat1.at<float>(i,0) == 0.0) || (point_mat2.at<float>(i,0) == 0.0))
      {
        // cout<<"found a absolute zero!!"<<endl;
        continue;
      }
      if(j == 0){
        point_mat1.row(i).copyTo(points1_out);
        point_mat2.row(closest_indices[i]).copyTo(points2_out);
        j = j+1;
      }else{
        cv::Mat intersection_1_to_append; // = point_mat1.at(i);
        cv::Mat intersection_2_to_append; // = point_mat2.at(closest_indices.row(i));
        point_mat1.row(i).copyTo(intersection_1_to_append);
        // cv::vconcat(intersection_point_mat1, intersection_1_to_append, intersection_point_mat1);
        points1_out.push_back(intersection_1_to_append);
        point_mat2.row(closest_indices[i]).copyTo(intersection_2_to_append);
        // cv::vconcat(intersection_point_mat2, intersection_2_to_append, intersection_point_mat2);
        points2_out.push_back(intersection_2_to_append);
        j = j+1;
      }
    }
  }

}

void ICPSlam::closestPoints(cv::Mat &point_mat1,
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
  for (int i=0;i<mat_indices.rows;++i) {
    closest_indices[i] = indices_ptr[i];
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
                               const tf::Transform &T_2_1, int num)
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
  cv::Mat transformed_point_mat2 = utils::transformPointMat(T_2_1, point_mat2);

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
    //cout<<"points2!!!!!! "<<point2<<endl;

    cv::circle(img, point1, 5, cv::Scalar(0, 0, 255), -1);
    cv::circle(img, point2, 5, cv::Scalar(255, 0, 0), -1);

    cv::line(img, point1, point2, cv::Scalar(0, 255, 0), 2);
  }

  cv::Mat tmp;
  cv::flip(img, tmp, 0);
  string name = "/tmp/icp_laser" + to_string(num) +".png";
  cv::imwrite(name, img);
}

tf::Transform ICPSlam::icpRegistration(const sensor_msgs::LaserScanConstPtr &laser_scan1,
                                    const sensor_msgs::LaserScanConstPtr &laser_scan2,
                                    const tf::Transform &T_2_1)
{
  //2*T21
  cv::Mat points1 = utils::laserScanToPointMat(laser_scan1);
  cv::Mat points2 = utils::laserScanToPointMat(laser_scan2);
  float error_threshold = 1e-7;

  // //  //======test
  // cv::Mat points1(3, 2, CV_32F);
  // cv::Mat points2(3, 2, CV_32F);
  // points1.at<float>(0,0)=1.0;
  // points1.at<float>(0,1)=1.0;
  // points1.at<float>(1,0)=2.0;
  // points1.at<float>(1,1)=1.0;
  // points1.at<float>(2,0)=4.0;
  // points1.at<float>(2,1)=4.0;

  // points2.at<float>(0,0)=0;
  // points2.at<float>(0,1)=1;
  // points2.at<float>(1,0)=0;
  // points2.at<float>(1,1)=2;
  // points2.at<float>(2,0)=-3;
  // points2.at<float>(2,1)=4;
  // cout<<"test data points 1 "<<points1<<endl;
  // cout<<"test data points 2 "<<points2<<endl;
  // tf::Transform T_2_1;
  // T_2_1.setOrigin(tf::Vector3(0, 1, 0));
  // T_2_1.setRotation(tf::createQuaternionFromYaw(-1.6));
  // //  //===========end test

  cv::Mat points2_new = utils::transformPointMat(T_2_1, points2);
  //cout<<"after est transform points 2 "<<points2_new<<endl;
  tf::Transform refined_T_2_1; 
  cout<<"Original Transform T: "<<T_2_1.getOrigin().getX()<<" "<<T_2_1.getOrigin().getY()<<" Rotation: "<<tf::getYaw(T_2_1.getRotation()) * 180/M_PI <<endl;
  
  cv::Mat error;
  cv::reduce((points2_new-points1).mul(points2_new-points1), error, 0, CV_REDUCE_AVG);
  float last_error = error.at<float>(0,0) + error.at<float>(0,1);
  cout<<"Original Error: "<<last_error<<endl;
  // cout<<"this is matrix point 2!!!!!!"<<points2<<endl;

  vizClosestPoints(points1, points2, T_2_1, 100);
  for(int i =0; i<20; i++){
    cout<<i<<" iteration"<<endl;
    std::vector<int> closest_indices;
    std::vector<float> closest_distances_2;
    closestPoints(points1, points2_new, closest_indices, closest_distances_2);
    //cout<<"+++++++++++++++indices: "<<endl;
    // for (auto i = closest_indices.begin(); i != closest_indices.end(); ++i)
    // std::cout << *i << ' ';

    cv::Mat points1_out;
    cv::Mat points2_out;

    intersectionPoints(points1, points2, closest_indices, closest_distances_2, points1_out, points2_out);
    
    //testing without rejection
    // cv::Mat points2_reordered;
    // for(int i = 0; i < closest_indices.size(); i++)
    // {
    // int ind = closest_indices[i];
    // points2_reordered.push_back(points2.row(ind));
    // }
    // cout<<"points2_reordered "<<points2_reordered<<endl;
    
    refined_T_2_1 = icpIteration(points1_out, points2_out);

    points2_new = utils::transformPointMat(refined_T_2_1, points2);
    cv::reduce((points2_new-points1).mul(points2_new-points1), error, 0, CV_REDUCE_AVG);
    float current_error = error.at<float>(0,0) + error.at<float>(0,1);
    cout<<"Error: "<<current_error<<endl;

    vizClosestPoints(points1_out, points2_out, refined_T_2_1, i);

    cout<<"Refined Transform T: "<<refined_T_2_1.getOrigin().getX()<<" "<<refined_T_2_1.getOrigin().getY()<<" Rotation: "<<tf::getYaw(refined_T_2_1.getRotation()) * 180/M_PI <<endl;
    if(abs(last_error-current_error)<error_threshold){
        return refined_T_2_1;
      }
    last_error = current_error;
    }
    
  
  
  return refined_T_2_1;

}

tf::Transform ICPSlam::icpIteration(cv::Mat &point_mat1,
                                    cv::Mat &point_mat2) 
{
  // cout<<"test starts======================="<<endl;
  // cv::Mat map_(5,5,CV_16S);
  // for(int i=0; i<5; i++)
  // {
  //   for(int j=0; j<5; j++)
  //           map_.at<int>(i,j)=0;
  // }
  // cv::LineIterator it(map_,
  //                   cv::Point(1, 1),
  //                   cv::Point(4, 2)
  // );
  // for(int j = 0; j < it.count; j++, ++it) {
  //     cv::Point point = it.pos(); // (point.x, point.y)
  //     cout<<it.count<<endl;
  //     cout<<point.x<<"hahahahaha"<<point.y<<endl;
  // }
  // cout<<"===========test end======================="<<endl;

  
  cv::Mat ux;
  cv::reduce(point_mat1, ux, 0, CV_REDUCE_AVG);
  cv::Mat up;
  cv::reduce(point_mat2, up, 0, CV_REDUCE_AVG);
  // cout<<"this is ux "<<ux<<endl;
  // cout<<"this is up "<<up<<endl;

  // cout<<"this is point_mat1 \n"<<point_mat1<<endl;
  // cout<<"this is point_mat2 \n"<<point_mat2<<endl;

  cv::Mat x_prime;
  for(int i=0; i<point_mat1.rows; i++){
    cv::Mat temp(1,2, CV_32F);
    temp.at<float>(0,0) = point_mat1.at<float>(i,0)-ux.at<float>(0,0);
    temp.at<float>(0,1) = point_mat1.at<float>(i,1)-ux.at<float>(0,1);
    x_prime.push_back(temp);
  }
  
  cv::Mat p_prime;
  for(int i=0; i<point_mat2.rows; i++){
    cv::Mat temp(1,2, CV_32F);
    temp.at<float>(0,0) = point_mat2.at<float>(i,0)-up.at<float>(0,0);
    temp.at<float>(0,1) = point_mat2.at<float>(i,1)-up.at<float>(0,1);
    p_prime.push_back(temp);
  }
  // cout<<"this is x_prime "<<x_prime<<endl;
  // cout<<"this is p_prime "<<p_prime<<endl;
  x_prime = x_prime.t();
  p_prime = p_prime.t(); 

  auto w = x_prime * p_prime.t();
  
  cv::SVD svd(w);
  // cout<<"this is w "<<w<<endl;
  
  cv::Mat r = svd.u*svd.vt;
  // cout<<"this is r "<<r<<endl;
  cv::Mat t = ux.t() - (r*up.t()) ;
  // cout<<"this is t "<<t<<endl;
  // cout<<"im here!!!!!!!!!!!!!"<<endl;
  
  float rotation = atan2(r.at<float>(1,0), r.at<float>(0,0));
  //cout<<"this is r "<<rotation<<endl;
  //cout<<"this is t "<<t<<endl;
  tf::Transform refined_T_2_1;
  refined_T_2_1.setOrigin(tf::Vector3(t.at<float>(0,0),t.at<float>(0,1),0.0));
  refined_T_2_1.setRotation(tf::createQuaternionFromYaw(rotation));

  return refined_T_2_1;

}               

} // namespace icp_slam

