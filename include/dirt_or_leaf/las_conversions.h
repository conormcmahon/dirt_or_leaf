
#ifndef LAS_CONVERSIONS_
#define LAS_CONVERSIONS_
#define PCL_NO_PRECOMPILE

// Default PCL Point Types
#include <pcl/point_types.h>
// Custom PCL Point Types
#include <dirt_or_leaf/point_las.h>
#include <dirt_or_leaf/point_veg.h>
#include <dirt_or_leaf/point_2d_scale.h>
#include <dirt_or_leaf/point_2d_index.h>
// Instantiate Custom PCL Templates 
#include <pcl/impl/instantiate.hpp>

namespace pcl
{
    // Copy PointCloud between two XYZ-based point clouds
    template <typename CloudIn, typename CloudOut> void 
    copyPointCloud3D(CloudIn input, CloudOut output);
    // Copy PointCloud between two XY-based point clouds
    template <typename CloudIn, typename CloudOut> void 
    copyPointCloud2D(CloudIn input, CloudOut output);
    // Copy PointCloud from 3D to 2.5D 
    template <typename CloudIn, typename CloudOut> void 
    copyPointCloud2_5D(CloudIn input, CloudOut output);
}


#endif // LAS_CONVERSIONS_