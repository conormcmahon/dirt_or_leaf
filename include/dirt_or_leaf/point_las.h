
#ifndef POINT_LAS_
#define POINT_LAS_
#define PCL_NO_PRECOMPILE

#include <pcl/point_types.h>
#include <pcl/point_representation.h>

namespace pcl{

struct PointLAS
  {
    PCL_ADD_POINT4D                     // Macro quad-word XYZ
    float intensity;                    // Laser intensity
    float classification;               // Surface classification
    float returnnumber;                // Laser Return Number
    float numberofreturns;                // Laser Return Number
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // Ensure proper alignment
  } EIGEN_ALIGN16;
}

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointLAS,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (float, returnnumber, returnnumber)
                                  (float, numberofreturns, numberofreturns)
                                  (float, classification, classification)
)

namespace pcl {
template <>
class DefaultPointRepresentation<PointLAS> : public PointRepresentation<PointLAS>
{
public:
  DefaultPointRepresentation ()
  {
    nr_dimensions_ = 3;
  }
  
  virtual void
  copyToFloatArray (const PointLAS &p, float * out) const
  {
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
  }
};
}

//typedef pcl::PointCloud<pcl::PointLAS> PointCloudPointLAS;

#endif // POINT_LAS_