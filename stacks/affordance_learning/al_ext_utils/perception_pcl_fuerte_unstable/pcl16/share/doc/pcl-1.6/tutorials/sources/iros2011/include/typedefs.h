#ifndef TYPEDEFS_H
#define TYPEDEFS_H

#include <pcl16/point_types.h>
#include <pcl16/point_cloud.h>

/*  Define some custom types to make the rest of our code easier to read */

// Define "PointCloud" to be a pcl16::PointCloud of pcl16::PointXYZRGB points
typedef pcl16::PointXYZRGB PointT;
typedef pcl16::PointCloud<PointT> PointCloud;
typedef pcl16::PointCloud<PointT>::Ptr PointCloudPtr;
typedef pcl16::PointCloud<PointT>::ConstPtr PointCloudConstPtr;

// Define "SurfaceNormals" to be a pcl16::PointCloud of pcl16::Normal points
typedef pcl16::Normal NormalT;
typedef pcl16::PointCloud<NormalT> SurfaceNormals;
typedef pcl16::PointCloud<NormalT>::Ptr SurfaceNormalsPtr;
typedef pcl16::PointCloud<NormalT>::ConstPtr SurfaceNormalsConstPtr;

// Define "SurfaceElements" to be a pcl16::PointCloud of pcl16::PointNormal points
typedef pcl16::PointNormal SurfelT;
typedef pcl16::PointCloud<SurfelT> SurfaceElements;
typedef pcl16::PointCloud<SurfelT>::Ptr SurfaceElementsPtr;
typedef pcl16::PointCloud<SurfelT>::ConstPtr SurfaceElementsConstPtr;


// Define "LocalDescriptors" to be a pcl16::PointCloud of pcl16::FPFHSignature33 points
typedef pcl16::FPFHSignature33 LocalDescriptorT;
typedef pcl16::PointCloud<LocalDescriptorT> LocalDescriptors;
typedef pcl16::PointCloud<LocalDescriptorT>::Ptr LocalDescriptorsPtr;
typedef pcl16::PointCloud<LocalDescriptorT>::ConstPtr LocalDescriptorsConstPtr;

// Define "GlobalDescriptors" to be a pcl16::PointCloud of pcl16::VFHSignature308 points
typedef pcl16::VFHSignature308 GlobalDescriptorT;
typedef pcl16::PointCloud<GlobalDescriptorT> GlobalDescriptors;
typedef pcl16::PointCloud<GlobalDescriptorT>::Ptr GlobalDescriptorsPtr;
typedef pcl16::PointCloud<GlobalDescriptorT>::ConstPtr GlobalDescriptorsConstPtr;

#endif
