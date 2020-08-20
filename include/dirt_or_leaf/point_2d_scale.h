
#ifndef POINT_2DSCALE_
#define POINT_2DSCALE_
#define PCL_NO_PRECOMPILE

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/impl/instantiate.hpp>

#include <pcl/filters/passthrough.h>


namespace pcl{

struct Point2DScale
  {
    float x;
    float y;
    float scale;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // Ensure proper alignment
  } EIGEN_ALIGN16;
}

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::Point2DScale,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, scale, scale)
)

namespace pcl {
template <>
class DefaultPointRepresentation<Point2DScale> : public PointRepresentation<Point2DScale>
{
public:
  DefaultPointRepresentation ()
  {
    nr_dimensions_ = 2;
  }
  
  virtual void
  copyToFloatArray (const Point2DScale &p, float * out) const
  {
    out[0] = p.x;
    out[1] = p.y;
  }
};
}

//typedef pcl::PointCloud<pcl::Point2DScale> Point2DScale;

#endif // POINT_2DSCALE_