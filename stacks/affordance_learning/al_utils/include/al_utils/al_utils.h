/*
 * al_utils.h
 * Copyright (c) 2011, Kadir Firat Uyanik, Kovan Research Lab, METU
 * kadir@ceng.metu.edu.tr
 *
 * All rights reserved.
 *
 * Software License Agreement (BSD License)
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Kovan Lab nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef AL_UTILS_H_
#define AL_UTILS_H_

#include "iostream"
#include "string"
#include "sstream"
#include "vector"
#include "limits"
#include "algorithm"

//keyboard event
#include <termios.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <dirent.h>
#include "fstream"
#include "cstdlib"

#include "ros/package.h"
#include "tf/transform_listener.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/ColorRGBA.h"
#include "arm_navigation_msgs/CollisionObject.h"
#include "tf/transform_datatypes.h"

#include "opencv2/opencv.hpp"

#include "pcl/ros/register_point_struct.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/impl/point_types.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/ros/conversions.h>
#include "pcl_ros/impl/transforms.hpp"
#include "pcl/search/kdtree.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/impl/voxel_grid.hpp"
#include <pcl/filters/extract_indices.h>
#include "bullet/LinearMath/btMatrix3x3.h"
#include "bullet/LinearMath/btVector3.h"

//these are necessary so that corresponding modules can work with custom type points
#include "pcl/filters/impl/extract_indices.hpp"
#include "pcl/kdtree/impl/kdtree_flann.hpp"
#include "pcl/search/impl/organized.hpp"
#include "pcl/features/impl/normal_3d_omp.hpp"
#include "pcl/features/impl/normal_3d.hpp"

#include "pcl/features/normal_3d.h"
#include <pcl/features/normal_3d_omp.h>
#include "pcl/octree/octree_pointcloud.h"

#include "pcl/search/organized.h"

#include "al_msgs/Behavior.h"
#include "al_msgs/Effect.h"
#include "al_msgs/Shape.h"
#include "al_msgs/Spec.h"
#include "al_msgs/FeatureVector.h"

#include <Eigen/SVD>
#include <Eigen/Core>
#include <Eigen/Eigen>

//#include "arff_parser.h"
//#include "/home/kadir/interconnection/workspace/work/metu-ros-pkg/trunk/affordance_learning/al_ext_utils/arff/src/arff_parser.h"

namespace pcl
{
  struct PointXYZCurvatures
  {
    PCL_ADD_POINT4D
    ; // preferred way of adding a XYZ+padding
    int hk_label;
    int sc_label;
    float h;
    float k;
    float s;
    float c;
    float pc1;
    float pc2;EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
  }EIGEN_ALIGN16;
// enforce SSE padding for correct memory alignment

  struct PointXYZPCurvs
  {
    PCL_ADD_POINT4D
    ; // preferred way of adding a XYZ+padding
    float pc_1;
    float pc_2;EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
  }EIGEN_ALIGN16;
// enforce SSE padding for correct memory alignment
}

namespace al
{
  namespace learning
  {
    enum AdjType
    {
      adj_thin_thick = 1, adj_round_edgy, adj_tall_short, adj_big_small
    };

//    ArffData*
//    initARFF (ArffParser*& arff_parser, std::string file_name);
//
//    //saves file under the directoru given with the relation name
//    void
//    saveARFF (ArffData* data, const std::string &directory_name);
  }

  namespace speech
  {
    std::vector<std::string>
    tokenizeSentence (std::string speech_text);
  }

  namespace perception
  {
    const int MAX_N_OBJECTS_IN_ENVIRONMENT = 100;
    const int N_NORMAL_HISTOGRAM_BINS = 20;
    const int N_CURVATURE_HISTOGRAM_BINS = 20;
    const int N_SHAPE_ID_HISTOGRAM_BINS = 20;

    const float TABLE_SURFACE_THICKNESS = 0.03;

    sensor_msgs::PointCloud
    toPCMsg (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_data);

    bool
    readPCD (std::string file_name, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud_data);

    bool
    writePCD (std::string file_name, pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_data);

    bool
    writePCD (std::string file_name, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_data);
  }

  namespace facilities
  {
    std::string
    toString (const int int_);

    int
    toInt (const std::string str_);

    std::string
    shapeEnumToString (int8_t shape_enum);

    std::string
    behaviorEnumToString (int8_t behavior_enum);

    std::string
    effectEnumToString (int8_t effect_enum);

    std::string
    adjTypeEnumToString (int adj_type);

    std::string
    specEnumToString (int8_t spec_enum);

    int8_t
    effectStringToEnum (std::string effect_str);

//    std::vector<float>
//    cvtRawFeatureVector (const al_msgs::FeatureVector& feature_vector);

    std::vector<double>
    cvtRawFeatureVector (const al_msgs::FeatureVector& feature_vector);

    bool
    createFeatureVector (const std::vector<float>& raw_data, al_msgs::FeatureVector& feature_vector);
  }

  namespace memory
  {
    arm_navigation_msgs::CollisionObject*
    getCollisionObject (std::vector<arm_navigation_msgs::CollisionObject> &collision_objects, std::string id);
  }

  namespace behavior
  {
    bool
    getTransformBetween (const std::string parent_frame, const std::string child_frame,
                         tf::StampedTransform& transform);
  }

  namespace viz
  {
    enum ColorID
    {
      GREY, MAGENTA, RED, ORANGE, YELLOW, GREEN, CYAN, BLUE, BLACK, WHITE
    };

    std_msgs::ColorRGBA
    colorize (int color_id, float alpha);

    std_msgs::ColorRGBA
    cvtHSVToRGB (float hue, float saturation, float value);

    bool
    cvtTo8BitImage (const cv::Mat &src, cv::Mat &dest, float min_val, float max_val, bool discard_zeroes = true);

    void
    vizPyramid (const std::vector<cv::Mat> &img_levels);

  }

  namespace sim
  {
    int
    spawnObjectByModel (std::string model_name);
  }

  namespace system
  {
    const int ENTER_KEY = 10;
    const int SPACE_KEY = 32;

    std::string
    execCmd (std::string sys_cmd);

    int
    execSysCmd (std::string sys_cmd);

    int
    getProcessIdByName (std::string p_name);

    int
    killProcessByName (std::string p_name, int kill_signal = SIGINT);

    int
    killNodeByName (std::string n_name);

    void
    changeInputMode (int dir);

    char
    keyboardHit ();

    bool
    getKey (char& key);

    bool
    getPackagePath (std::string &package_path, const std::string ros_package_name);

    bool
    getFileList (std::vector<std::string> &file_list, const std::string parent_dir_name);
  }

  namespace math
  {
    const float PI = 3.14159265;
    const float PI_2 = 1.57079633;

//    const double C_ZERO = 0.03;
    //this wasn't bad
//    const double C_ZERO = 0.5;
    const double C_ZERO = 0.03;
    const double H_ZERO = C_ZERO;
//    const double H_ZERO = 0.05;
    const double K_ZERO = H_ZERO * H_ZERO;
//    const double K_ZERO = 0.008;

    const uint IMAGE_WIDTH = 640;
    const uint IMAGE_HEIGHT = 480;

    const int KERNEL_SIZE = 15;
    const int N_LEVEL = 5;
    const int PAD_SIZE = (int)(KERNEL_SIZE / 2);
    const float SIGMA = 10.0;

    const float MAX_ELEMENT_VAL = 1000000;

    enum BoxSide //according to the robot's point of view

    {
      TOP, BOTTOM, LEFT, RIGHT, REAR, FRONT
    };

    enum DiffMode
    {
      F_X, F_Y, F_XX, F_XY, F_YY
    };

    struct Curvatures
    {
      double H;
      double K;
      double S;
      double C;
      double pc_1;
      double pc_2;

      void
      print (std::ostream& out)
      {
        out.precision (5);
        out << std::fixed << std::setw (10) << H << std::setw (14) << K << std::setw (14) << S << std::setw (14) << C
            << std::setw (14) << pc_1 << std::setw (14) << pc_2 << " \n";
      }
    };

    enum CurvatureSpace
    {
      HK, SC, HKSC
    };

    enum CurvatureEstimationMethod
    {
      GOLDFEATHER2004, QUADRIC_FITTING
    };

    struct LineSegment
    {
      tf::Vector3 start_;
      tf::Vector3 end_;
      LineSegment ()
      {
      }

      LineSegment (const tf::Vector3 &start, const tf::Vector3 &end)
      {
        start_ = start;
        end_ = end;
      }
    };

//stores ccw oriented points, considers only XY components (like a projection)
//Polyongs are supposed to be convex
    struct Polygon
    {
      std::vector<tf::Vector3> vertices_;
      float area_;

      Polygon ();

      Polygon (const std::vector<tf::Vector3> vertices);

      bool
      setVertices (const std::vector<tf::Vector3> vertices);

      //calculates the area of a convex polygon by triangulation
      float
      getArea ();
    };

//in the XY plane
    float
    getAngle (tf::Vector3 v);

    float
    fPrecision (const ros::Time &time, const int n_int_digits = 4, const int n_floating_digits = 3);

    float
    fPrecision (const ros::Duration &duration, const int n_floating_digits = 6);

    float
    fPrecision (const float number, const int n_int_digits = 4, const int n_floating_digits = 3);

//this is actually determinant of the column matrix where v1 and v2 are the column vectors
    float
    get2DCross (tf::Vector3 v1, tf::Vector3 v2);

    float
    fRand (float min, float max);

    int
    iRand (int min, int max);

    float
    getAngleBetween (tf::Vector3 v1, tf::Vector3 v2);

    float
    getAreaBetween (tf::Vector3 v1, tf::Vector3 v2);

    float
    getIntersectionVolume (const arm_navigation_msgs::CollisionObject &obj1,
                           const arm_navigation_msgs::CollisionObject &obj2);

    float
    getVolume (const arm_navigation_msgs::CollisionObject &obj);

    bool
    innerPoint (const tf::Vector3 &v, const Polygon& rect);

    bool
    intersectionPoint (const LineSegment seg1, const LineSegment seg2, tf::Vector3 &v);

    std::vector<tf::Vector3>
    getInnerPoints (const Polygon& rect1, const Polygon& rect2);

    std::vector<tf::Vector3>
    getIntersectionPoints (const Polygon& rect1, const Polygon& rect2);

    Polygon
    getXYCrossSection (const arm_navigation_msgs::CollisionObject &obj);

    float
    getDistanceBetween (const geometry_msgs::Vector3& v1, const geometry_msgs::Vector3& v2);

    float
    getDistanceBetween (const geometry_msgs::Point& p1, const geometry_msgs::Point& p2);

    double
    calBoxSimilarity (const geometry_msgs::Vector3& v1, const geometry_msgs::Vector3& v2, double w_x, double w_y,
                      double w_z);

    double
    calBoxSimilarity (const geometry_msgs::Vector3& v1, const geometry_msgs::Vector3& v2);

    btMatrix3x3
    getTransformationBetween (tf::Vector3 v_src, tf::Vector3 v_dest);

    template<typename T>
      int
      sgn (T val)
      {
        return (val > T (0)) - (val < T (0));
      }

    //mat_coeffs specifies the equation to be differentiated. Its raws and columns are accessed and modified
    //considering the powers of the terms consisting of x and y.
    //corresponding derivative is calculated at the point p
    double
    differentiate (const cv::Mat &mat_eqn_coeffs, const tf::Vector3 &point, DiffMode diff_mode);

    //calculates hessian of a polynomial function
    cv::Mat
    calcHessian (const cv::Mat &mat_eqn_coeffs, const tf::Vector3 &point);

    //calculates hessian of a polynomial function, returns first derivatives also
    cv::Mat
    calcHessian (const cv::Mat &mat_eqn_coeffs, const tf::Vector3 &point, double& f_x, double &f_y);

    //calculates hessian of a square-rooted function
    //TODO: this can easily be generalized to any rooted polynomial function
    //TODO: handle the fnc=0 conditions
    cv::Mat
    calcHessianSqrted (const cv::Mat &mat_eqn_coeffs, const tf::Vector3 &point, double& f_x, double &f_y);

    bool
    calcCurvatures (Curvatures &curvatures, const cv::Mat &hessian, const double f_x, const double f_y);

    bool
    calcCurvatures (Curvatures &curvatures, const Eigen::MatrixXd &weingarten_matrix);

    bool
    calcPointPCurvs (pcl::PrincipalCurvatures &pcurvs, pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_data,
                     pcl::PointCloud<pcl::Normal>::Ptr pointcloud_normals, pcl::search::KdTree<pcl::PointXYZ>::Ptr tree,
                     const int pt_idx, const int curv_estimation_method = GOLDFEATHER2004, const int n_neighbors = 15);

    bool
    calcPointPCurvs (pcl::PointCloud<pcl::PointXYZPCurvs>::Ptr pointcloud_data,
                     pcl::PointCloud<pcl::Normal>::Ptr pointcloud_normals,
                     pcl::search::KdTree<pcl::PointXYZPCurvs>::Ptr tree, const int pt_idx,
                     const int curv_estimation_method = GOLDFEATHER2004, const int n_neighbors = 15);

    //calculates curvatures of a sub-pointcloud --points are indicated with the indices- which is generated
    //by the polynomial function --in the form of f(x,y)=z- represented with the mat_eqn_coeffs
    std::vector<al::math::Curvatures>
    calcCloudCurvatures (pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_data, const std::vector<int> &indices,
                         const cv::Mat &mat_eqn_coeffs);

    //calculates curvatures of a sub-pointcloud --points are indicated with the indices-
    //considering the n_neighbors with the pre-calculated surface normals at these points (at least 3 points are necessary)
    //based on the method proposed in [Goldfeather2004]

    //calculates curvatures of a sub-pointcloud --points are indicated with the indices-
    //considering the n_neighbors (at least 7 points are necessary) by fitting the function
    //z = a00 + a10x + a11 xy + a12 xy^2 + a20x^2 + a21x^2y + a22x^2y^2
    std::vector<al::math::Curvatures>
    calcCloudCurvatures (pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_data,
                         pcl::PointCloud<pcl::Normal>::Ptr pointcloud_normals,
                         pcl::search::KdTree<pcl::PointXYZ>::Ptr tree, const std::vector<int> &indices,
                         const int n_neighbors, const int curv_estimation_method = GOLDFEATHER2004);

    //assumes NANF-free, segmented-object cloud
    bool
    calcCloudPCurvs (pcl::PointCloud<pcl::PointXYZPCurvs>::Ptr pointcloud_data,
                     pcl::PointCloud<pcl::Normal>::Ptr pointcloud_normals,
                     pcl::search::KdTree<pcl::PointXYZPCurvs>::Ptr tree, const int n_neighbors,
                     const int curv_estimation_method = GOLDFEATHER2004);

    Eigen::MatrixXd
    calcWeingartenMatrix (pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_data, const int pt_idx,
                          const std::vector<int> &neighbor_ids, pcl::PointCloud<pcl::Normal>::Ptr pointcloud_normals);

    std::vector<double>
    fitQuadricSurface (pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_data, const int pt_idx,
                       const std::vector<int> &neighbor_ids, pcl::PointCloud<pcl::Normal>::Ptr pointcloud_normals);

    std::vector<double>
    fitQuadricSurface (pcl::PointCloud<pcl::PointXYZPCurvs>::Ptr pointcloud_data, const int pt_idx,
                       const std::vector<int> &neighbor_ids, pcl::PointCloud<pcl::Normal>::Ptr pointcloud_normals);

    std::vector<std_msgs::ColorRGBA>
    labelCurvatures (std::vector<Curvatures> &all_curvatures, int label_space = HKSC, float sampling_ratio = 0.0005);

    //TODO: this might be better inside al::viz
    //sampling_ration by default 0.5mm/sample
    std_msgs::ColorRGBA
    labelCurvature (Curvatures &curvatures, int label_space = HKSC, float sampling_ratio = 0.0005);

    //extracts depth map for the given indices of the ptr_cloud, based on the transformation given for the camera_frame
    //zero-pads the image for the unused points, and assigns depth values of the points in the camera_frame
//    cv::Mat
//    extractDepthMap (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr ptr_cloud, const std::vector<int> &indices,
//                     const tf::TransformListener &listener, const std::string sensor_frame);
    cv::Mat
    extractDepthMap (pcl::PointCloud<pcl::PointXYZ>::ConstPtr ptr_cloud, const std::vector<int> &indices,
                     const tf::TransformListener &listener, const std::string sensor_frame, const uint image_width =
                         IMAGE_WIDTH);

    cv::Mat
    extractDepthMap (pcl::PointCloud<pcl::PointXYZ>::ConstPtr ptr_cloud, const std::vector<int> &indices,
                     const tf::TransformListener &listener, const std::string sensor_frame, const float sampling_ratio);

    cv::Mat
    generateGaussianKernel (const float sigma, const int k_width, const int k_height);

    //K = [ 0.25-a/2 0.25 a 0.25 0.25-a/2 ] here a can be between 0.3 and 0.6, it converges to the gaussian when a =0.4
    cv::Mat
    generateKernel (const float a);

    //sampling ratio: 0.5mm/sample = 0.0005m/sample
    bool
    downSampleCloud (pcl::PointCloud<pcl::PointXYZ>::ConstPtr ptr_cloud,
                     pcl::PointCloud<pcl::PointXYZ>::Ptr &ptr_downsampled_cloud, float sampling_ratio = 0.0005);

    bool
    downSampleCloud (pcl::PointCloud<pcl::PointXYZ>::ConstPtr ptr_cloud,
                     pcl::PointCloud<pcl::PointXYZ>::Ptr &ptr_downsampled_cloud, pcl::VoxelGrid<pcl::PointXYZ> &grid,
                     float sampling_ratio = 0.0005);

    //assumes that the type of src image is CV_32FC1
    //assumes that generative_kernel is already normalized
    cv::Mat
    expand (const cv::Mat & src, const cv::Mat & generative_kernel);

    cv::Mat
    expand (const cv::Mat & src, const cv::Mat & generative_kernel, const cv::Size dst_dize);

    //assumes that the type of src image is CV_32FC1
    //assumes that generative_kernel is already normalized
    cv::Mat
    reduce (const cv::Mat & src, const cv::Mat & generative_kernel);

    pcl::PointCloud<pcl::PointXYZPCurvs>::Ptr
    reduce (pcl::PointCloud<pcl::PointXYZPCurvs>::ConstPtr ptr_resampled_cloud, const cv::Mat & kernel);

    pcl::PointCloud<pcl::PointXYZ>::Ptr
    reduce (pcl::PointCloud<pcl::PointXYZ>::ConstPtr ptr_resampled_cloud, const cv::Mat & kernel);

    pcl::PointCloud<pcl::PointXYZPCurvs>::Ptr
    expand (pcl::PointCloud<pcl::PointXYZPCurvs>::ConstPtr ptr_resampled_cloud, const cv::Mat & kernel,
            const Eigen::Vector2i expansion_size);

    std::vector<cv::Mat>
    buildGaussianPyramid (const cv::Mat &src, const cv::Mat & generative_kernel, bool resize_to_original = true,
                          const uint n_level = 4);

    std::vector<pcl::PointCloud<pcl::PointXYZPCurvs>::Ptr>
    buildPyramid (pcl::PointCloud<pcl::PointXYZ>::ConstPtr ptr_resampled_cloud, const cv::Mat & kernel,
                  bool resize_to_original = true, const uint n_level = 4, const int n_neighbors = 15);

    //**********************************************************************************************************//
    //******************************************THESIS __ WORK**************************************************//

    //raw cloud: in sensor frame
    //kernel: gaussian kernel for gaussian pyramiding
    std::vector<pcl::PointCloud<pcl::PointXYZCurvatures>::Ptr>
    extractScaledLabeledCloud (pcl::PointCloud<pcl::PointXYZ>::ConstPtr ptr_raw_cloud, int n_pyr_levels, cv::Mat kernel,
                               float sampling_ratio = 0.0005, int n_neighbors = 15, int curv_est_method =
                                   GOLDFEATHER2004);

    //returns ptr to the raw resampled point cloud. Actual processed cloud is returned with ptr_resampled_cloud.
    //It is a square cross-sectioned pointcloud, where non-existing elements are filled with nanf. Careful with this.
    pcl::PointCloud<pcl::PointXYZ>::Ptr
    resampleCloud (pcl::PointCloud<pcl::PointXYZ>::ConstPtr ptr_cloud,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr &ptr_resampled_cloud, float sampling_ratio = 0.0005);

    //ptr_src_cloud is reduced to obtain higher-scale less-detailed ptr_dst_cloud (reduced in details and halved in size)
    bool
    reduce (pcl::PointCloud<pcl::PointXYZCurvatures>::ConstPtr ptr_src_cloud,
            pcl::PointCloud<pcl::PointXYZCurvatures>::Ptr &ptr_dst_cloud, const cv::Mat & kernel);

    //assumes NANF-free, segmented-object cloud
    bool
    calcCloudPCurvatures (pcl::PointCloud<pcl::PointXYZCurvatures>::Ptr pointcloud_data,
                          pcl::PointCloud<pcl::Normal>::Ptr pointcloud_normals,
                          pcl::search::KdTree<pcl::PointXYZCurvatures>::Ptr tree, const int n_neighbors,
                          const int curv_estimation_method = GOLDFEATHER2004);

    bool
    calcPointPCurvatures (pcl::PointCloud<pcl::PointXYZCurvatures>::Ptr pointcloud_data,
                          pcl::PointCloud<pcl::Normal>::Ptr pointcloud_normals,
                          pcl::search::KdTree<pcl::PointXYZCurvatures>::Ptr tree, const int pt_idx,
                          const int curv_estimation_method = GOLDFEATHER2004, const int n_neighbors = 15);

    std::vector<double>
    fitQuadricSurface (pcl::PointCloud<pcl::PointXYZCurvatures>::Ptr pointcloud_data, const int pt_idx,
                       const std::vector<int> &neighbor_ids, pcl::PointCloud<pcl::Normal>::Ptr pointcloud_normals);

    //ptr_src_cloud is expanded to obtain lower-scale larger-sized ptr_dst_cloud
    bool
    expand (pcl::PointCloud<pcl::PointXYZCurvatures>::ConstPtr ptr_src_cloud,
            pcl::PointCloud<pcl::PointXYZCurvatures>::Ptr &ptr_dst_cloud, const cv::Mat & kernel,
            const Eigen::Vector2i expansion_size);

    bool
    calcPointHKSCFromPCurvatures (pcl::PointCloud<pcl::PointXYZCurvatures>::Ptr &pointcloud_data, const int pt_idx);

    bool
    calcCloudHKSCFromPCurvatures (pcl::PointCloud<pcl::PointXYZCurvatures>::Ptr &pointcloud_data);

    //sampling_ration by default 0.5mm/sample
    bool
    labelPointCurvature (pcl::PointCloud<pcl::PointXYZCurvatures>::Ptr &pointcloud_data, const int pt_idx,
                         int label_space = HKSC, float sampling_ratio = 0.0005);

    //sampling_ration by default 0.5mm/sample
    bool
    labelCloudCurvatures (pcl::PointCloud<pcl::PointXYZCurvatures>::Ptr &pointcloud_data, int label_space = HKSC,
                          float sampling_ratio = 0.0005);

  //**********************************************************************************************************//
  //******************************************THESIS __ WORK**************************************************//

  }

  namespace naming
  {
    const std::string PR2_KINECT_FRAME = "head_mount_kinect_rgb_optical_frame";
    const std::string OPENNI_DEPTH_FRAME = "openni_depth_frame";
    const std::string BASE_LINK = "base_link";

    const std::string LEFT_SHOULDER = "left_shoulder";
    const std::string RIGHT_SHOULDER = "right_shoulder";

    const std::string LEFT_ELBOW = "left_elbow";
    const std::string RIGHT_ELBOW = "right_elbow";

    const std::string LEFT_HAND = "left_hand";
    const std::string RIGHT_HAND = "right_hand";

    const std::string TORSO = "torso";
    const std::string NECK = "neck";
    const std::string HEAD = "head";

    std::string
    createFrameName (std::string part, int user_id = 1);

    std::string
    createJointName (std::string part_parent, std::string part_child, int dimension);

    std::string
    createJointName (std::string part_parent, std::string part_child, std::string dim);

    std::string
    getPartName (std::string frame_name);
  }
}

POINT_CLOUD_REGISTER_POINT_STRUCT(
    pcl::PointXYZCurvatures, // here we assume a XYZ + "test" (as fields)
    (float, x, x) (float, y, y) (float, z, z) (int, hk_label, hk_label) (int, sc_label, sc_label) (float, h, h) (float, k, k) (float, s, s) (float, c, c) (float, pc1, pc1) (float, pc2, pc2));

POINT_CLOUD_REGISTER_POINT_STRUCT( pcl::PointXYZPCurvs, // here we assume a XYZ + "test" (as fields)
    (float, x, x) (float, y, y) (float, z, z)(float, pc_1, pc_1) (float, pc_2, pc_2));

#endif /* AL_UTILS_H_ */
