
#ifndef VEG_POINT_
#define VEG_POINT_
#define PCL_NO_PRECOMPILE

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/impl/instantiate.hpp>

#include <pcl/filters/passthrough.h>


namespace pcl{

struct PointVeg
  {
    PCL_ADD_POINT4D                     // Macro quad-word XYZ
    float intensity;                    // Laser intensity
    float classification;               // Surface classification
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // Ensure proper alignment
  } EIGEN_ALIGN16;
}

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::LASPoint,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (float, classification, classification)
)

namespace pcl {
template <>
class DefaultPointRepresentation<LASPoint> : public PointRepresentation<LASPoint>
{
public:
  DefaultPointRepresentation ()
  {
    nr_dimensions_ = 3;
  }
  
  virtual void
  copyToFloatArray (const LASPoint &p, float * out) const
  {
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
  }
};
}

//typedef pcl::PointCloud<pcl::LASPoint> PointCloudLASPoint;

#endif // VEG_POINT_