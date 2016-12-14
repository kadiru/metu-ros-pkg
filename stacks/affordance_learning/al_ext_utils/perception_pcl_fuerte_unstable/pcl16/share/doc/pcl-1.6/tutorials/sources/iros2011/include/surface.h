#ifndef SURFACE_H_
#define SURFACE_H_

#include <pcl16/kdtree/kdtree_flann.h>
#include <pcl16/surface/mls.h>
#include <pcl16/surface/convex_hull.h>
#include <pcl16/surface/concave_hull.h>
#include <pcl16/surface/gp3.h>
#include <pcl16/surface/marching_cubes_hoppe.h>

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
  PointCloudPtr output (new PointCloud);
  return (output);
}

SurfaceElementsPtr
computeSurfaceElements (const PointCloudPtr & input, float radius, int polynomial_order)
{
  SurfaceElementsPtr surfels (new SurfaceElements);
  return (surfels);
}

MeshPtr
computeConvexHull (const PointCloudPtr & input)
{
  MeshPtr output (new Mesh);
  return (output);
}


MeshPtr
computeConcaveHull (const PointCloudPtr & input, float alpha)
{
  MeshPtr output (new Mesh);
  return (output);
}

pcl16::PolygonMesh::Ptr
greedyTriangulation (const SurfaceElementsPtr & surfels, float radius, float mu, int max_nearest_neighbors, 
                     float max_surface_angle, float min_angle, float max_angle)

{
  pcl16::PolygonMesh::Ptr output (new pcl16::PolygonMesh);
  return (output);
}


pcl16::PolygonMesh::Ptr
marchingCubesTriangulation (const SurfaceElementsPtr & surfels, float leaf_size, float iso_level)
{
  pcl16::PolygonMesh::Ptr output (new pcl16::PolygonMesh);
  return (output);
}

#endif
