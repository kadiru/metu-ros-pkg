#ifndef SURFACE_H_
#define SURFACE_H_

#include <pcl16/kdtree/kdtree_flann.h>
#include <pcl16/surface/mls.h>
#include <pcl16/surface/convex_hull.h>
#include <pcl16/surface/concave_hull.h>
#include <pcl16/surface/gp3.h>
#include <pcl16/surface/marching_cubes_greedy.h>

#include "typedefs.h"


class Mesh
{
  public:
    Mesh () : points (new PointCloud) {}
    PointCloudPtr points;
    std::vector<pcl16::Vertices> faces;
};

typedef boost::shared_ptr<Mesh> MeshPtr;

PointCloudPtr
smoothPointCloud (const PointCloudPtr & input, float radius, int polynomial_order)
{
  pcl16::MovingLeastSquares<PointT, NormalT> mls;
  mls.setSearchMethod (pcl16::KdTreeFLANN<PointT>::Ptr (new pcl16::KdTreeFLANN<PointT>));
  mls.setSearchRadius (radius);
  mls.setSqrGaussParam (radius*radius);
  mls.setPolynomialFit (polynomial_order > 1);
  mls.setPolynomialOrder (polynomial_order);
  
  mls.setInputCloud (input);

  PointCloudPtr output (new PointCloud);
  mls.reconstruct (*output);

  return (output);
}

SurfaceElementsPtr
computeSurfaceElements (const PointCloudPtr & input, float radius, int polynomial_order)
{
  pcl16::MovingLeastSquares<PointT, NormalT> mls;
  mls.setSearchMethod (pcl16::KdTreeFLANN<PointT>::Ptr (new pcl16::KdTreeFLANN<PointT>));
  mls.setSearchRadius (radius);
  mls.setSqrGaussParam (radius*radius);
  mls.setPolynomialFit (polynomial_order > 1);
  mls.setPolynomialOrder (polynomial_order);
  
  mls.setInputCloud (input);

  PointCloudPtr points (new PointCloud);
  SurfaceNormalsPtr normals (new SurfaceNormals);
  mls.setOutputNormals (normals);
  mls.reconstruct (*points);

  SurfaceElementsPtr surfels (new SurfaceElements);
  pcl16::copyPointCloud (*points, *surfels);
  pcl16::copyPointCloud (*normals, *surfels);
  return (surfels);
}

MeshPtr
computeConvexHull (const PointCloudPtr & input)
{
  pcl16::ConvexHull<PointT> convex_hull;
  convex_hull.setInputCloud (input);

  MeshPtr output (new Mesh);
  convex_hull.reconstruct (*(output->points), output->faces);

  return (output);
}


MeshPtr
computeConcaveHull (const PointCloudPtr & input, float alpha)
{
  pcl16::ConcaveHull<PointT> concave_hull;
  concave_hull.setInputCloud (input);
  concave_hull.setAlpha (alpha);

  MeshPtr output (new Mesh);
  concave_hull.reconstruct (*(output->points), output->faces);

  return (output);
}

pcl16::PolygonMesh::Ptr
greedyTriangulation (const SurfaceElementsPtr & surfels, float radius, float mu, int max_nearest_neighbors, 
                     float max_surface_angle, float min_angle, float max_angle)

{
  pcl16::GreedyProjectionTriangulation<pcl16::PointNormal> gpt;
  gpt.setSearchMethod (pcl16::KdTreeFLANN<pcl16::PointNormal>::Ptr (new pcl16::KdTreeFLANN<pcl16::PointNormal>));

  gpt.setSearchRadius (radius);
  gpt.setMaximumNearestNeighbors (max_nearest_neighbors);
  gpt.setMu (mu);
  gpt.setMaximumSurfaceAgle (max_surface_angle);
  gpt.setMinimumAngle (min_angle);
  gpt.setMaximumAngle (max_angle);
  gpt.setNormalConsistency (true);

  gpt.setInputCloud (surfels);
  pcl16::PolygonMesh::Ptr output (new pcl16::PolygonMesh);
  gpt.reconstruct (*output);

  return (output);
}


pcl16::PolygonMesh::Ptr
marchingCubesTriangulation (const SurfaceElementsPtr & surfels, float leaf_size, float iso_level)
{
  pcl16::MarchingCubesGreedy<SurfelT> marching_cubes;
  marching_cubes.setSearchMethod (pcl16::KdTree<SurfelT>::Ptr (new pcl16::KdTreeFLANN<SurfelT> ()));
  marching_cubes.setLeafSize (leaf_size);
  marching_cubes.setIsoLevel (iso_level);

  marching_cubes.setInputCloud (surfels);
  pcl16::PolygonMesh::Ptr output (new pcl16::PolygonMesh);
  marching_cubes.reconstruct (*output);
  
  return (output);
}

#endif
