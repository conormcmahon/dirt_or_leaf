
#ifndef LAS_ARITHMETIC_HPP_
#define LAS_ARITHMETIC_HPP_

#include "dirt_or_leaf/las_arithmetic.h"

namespace pcl
{
    template<typename PointTypeFirst, typename PointTypeSecond>
    float pointDistance2D(PointTypeFirst first, PointTypeSecond second)
    {
        return sqrt(pow(first.x - second.x,2) + pow(first.y - second.y,2));
    }
    
    template<typename PointTypeFirst, typename PointTypeSecond>
    float pointDistance3D(PointTypeFirst first, PointTypeSecond second)
    {
        return sqrt(pow(first.x - second.x,2) + pow(first.y - second.y,2) + pow(first.z - second.z,2));
    }
}


#endif// LAS_ARITHMETIC_HPP_