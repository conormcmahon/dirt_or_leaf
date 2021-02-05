
#ifndef POINT_2DGROUND_
#define POINT_2DGROUND_
#define PCL_NO_PRECOMPILE

#include <pcl/point_types.h>
#include <pcl/point_representation.h>

namespace pcl{

struct Point2DGround
  {
    PCL_ADD_POINT4D;
    float intensity;
    PCL_ADD_NORMAL4D;
    float curvature;
    float height_diff_avg;
    float norm_diff_avg;
    float slope;
    float aspect;
    float height_over_stream;
    float dist_from_stream;
    int index;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // Ensure proper alignment
  } EIGEN_ALIGN16;
}

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::Point2DGround,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (float, normal_x, normal_x)
                                  (float, normal_y, normal_y)
                                  (float, normal_z, normal_z)
                                  (float, curvature, curvature)
                                  (float, height_diff_avg, height_diff_avg)
                                  (float, norm_diff_avg, norm_diff_avg)
                                  (float, slope, slope)
                                  (float, aspect, aspect)
                                  (float, height_over_stream, height_over_stream)
                                  (float, height_over_stream, dist_from_stream)
                                  (int, index, index)
)

namespace pcl {
template <>
class DefaultPointRepresentation<Point2DGround> : public PointRepresentation<Point2DGround>
{
public:
  DefaultPointRepresentation ()
  {
    nr_dimensions_ = 2;
  }
  
  virtual void
  copyToFloatArray (const Point2DGround &p, float * out) const
  {
    out[0] = p.x;
    out[1] = p.y;
  }
};
}

#endif // POINT_2DGROUND_