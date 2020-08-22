
#ifndef LAS_CONVERSIONS_
#define LAS_CONVERSIONS_
#define PCL_NO_PRECOMPILE

// Default PCL Point Types
#include <pcl/point_types.h>
// Custom PCL Point Types
#include <dirt_or_leaf/point_las.h>
#include <dirt_or_leaf/point_veg.h>
#include <dirt_or_leaf/point_2d_elevation.h>
#include <dirt_or_leaf/point_2d_index.h>
#include <dirt_or_leaf/point_2d_ground.h>
// Instantiate Custom PCL Templates 
#include <pcl/impl/instantiate.hpp>

namespace pcl
{
    // Resize an output cloud to match size of imput cloud
    template <typename CloudIn, typename CloudOut>
    bool alignCloudSizes(CloudIn input, CloudOut output);

    // Copy PointCloud between two XYZ-based point clouds
    template <typename CloudIn, typename CloudOut> 
    void copyPointCloud3D(CloudIn input, CloudOut output);
    // Copy PointCloud between two XY-based point clouds
    template <typename CloudIn, typename CloudOut> 
    void copyPointCloud2D(CloudIn input, CloudOut output);
    // Copy PointCloud from 3D two XY-based point clouds, retaining Point Index
    template <typename CloudIn, typename CloudOut> 
    void copyPointCloud2DIndex(CloudIn input, CloudOut output);
    // Copy PointCloud from 3D to 2.5D 
//    template <typename CloudIn, typename CloudOut> 
//    void copyPointCloud2_5D(CloudIn input, CloudOut output);
    // Copy PointCloud indices (for another external data cloud of larger size)
    template <typename CloudIn, typename CloudOut> 
    void copyPointCloudIndices(CloudIn input, CloudOut output);
    // Copy PointCloud indices (for another external data cloud of larger size)
    template <typename CloudIn> 
    void generatePointCloudIndices(CloudIn input);
    // Copy PointCloud from 2D + Index to 3D
    template <typename Cloud3D, typename CloudIn, typename CloudOut> 
    void copyPointCloud3D(Cloud3D xyz, CloudIn input, CloudOut output);
    // Copy Normals information from one cloud to another
    template <typename CloudIn, typename CloudOut>
    void copyPointCloudNormals(CloudIn input, CloudOut output);
}


#endif // LAS_CONVERSIONS_