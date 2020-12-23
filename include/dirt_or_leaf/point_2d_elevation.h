
#ifndef POINT_2DELEVATION_
#define POINT_2DELEVATION_
#define PCL_NO_PRECOMPILE

#include <pcl/point_types.h>
#include <pcl/point_representation.h>

namespace pcl{

struct Point2DElevation
  {
    PCL_ADD_POINT4D;
    float elevation;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // Ensure proper alignment
  } EIGEN_ALIGN16;
}

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::Point2DElevation,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
)

namespace pcl {
template <>
class DefaultPointRepresentation<Point2DElevation> : public PointRepresentation<Point2DElevation>
{
public:
  DefaultPointRepresentation ()
  {
    nr_dimensions_ = 2;
  }
  
  virtual void
  copyToFloatArray (const Point2DElevation &p, float * out) const
  {
    out[0] = p.x;
    out[1] = p.y;
  }
};
}

#endif // POINT_2DELEVATION_