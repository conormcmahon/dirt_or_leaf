
#ifndef LAS_ARITHMETIC_
#define LAS_ARITHMETIC_

#include <pcl/point_types.h>

namespace pcl
{
    // Euclidean distance between two points in XY 
    template<typename PointTypeFirst, typename PointTypeSecond>
    float pointDistance2D(PointTypeFirst first, PointTypeSecond second);
    // Euclidean distance between two points in XYZ
    template<typename PointTypeFirst, typename PointTypeSecond>
    float pointDistance3D(PointTypeFirst first, PointTypeSecond second);
}


#endif// LAS_ARITHMETIC_