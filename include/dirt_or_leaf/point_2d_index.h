
#ifndef POINT_2DINDEX_
#define POINT_2DINDEX_
#define PCL_NO_PRECOMPILE

#include <pcl/point_types.h>
#include <pcl/point_representation.h>

namespace pcl{

struct Point2DIndex
  {
    float x;
    float y;
    int index;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // Ensure proper alignment
  } EIGEN_ALIGN16;
}

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::Point2DIndex,
                                  (float, x, x)
                                  (float, y, y)
                                  (int, index, index)
)

namespace pcl {
template <>
class DefaultPointRepresentation<Point2DIndex> : public PointRepresentation<Point2DIndex>
{
public:
  DefaultPointRepresentation ()
  {
    nr_dimensions_ = 2;
  }
  
  virtual void
  copyToFloatArray (const Point2DIndex &p, float * out) const
  {
    out[0] = p.x;
    out[1] = p.y;
  }
};
}

#endif // POINT_2DINDEX_