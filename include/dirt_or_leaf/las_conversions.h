
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
    template <typename CloudIn, typename CloudOut> void 
    copyPointCloud3D(CloudIn input, CloudOut output);
    // LAS -> XY+Scale
    /*void copyPointCloud(pcl::PointCloud<pcl::PointLAS>::Ptr input, pcl::PointCloud<pcl::Point2DScale>::Ptr output);
    // LAS -> XY+Index
    void copyPointCloud(pcl::PointCloud<pcl::PointLAS>::Ptr input, pcl::PointCloud<pcl::Point2DIndex>::Ptr output);
    // XY+Scale -> XYZ
    void copyPointCloud(pcl::PointCloud<pcl::Point2DScale>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output);
    // LAS -> Veg
    void copyPointCloud(pcl::PointCloud<pcl::PointLAS>::Ptr input, pcl::PointCloud<pcl::PointVeg>::Ptr output);
    // LAS -> XYZ
    void copyPointCloud(pcl::PointCloud<pcl::PointLAS>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output);
    // LAS -> XYZI
    void copyPointCloud(pcl::PointCloud<pcl::PointLAS>::Ptr input, pcl::PointCloud<pcl::PointXYZI>::Ptr output);
    // XYZ -> XYZI
    void copyPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZI>::Ptr output);
    // Veg -> XYZ
    void copyPointCloud(pcl::PointCloud<pcl::PointVeg>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output); */
}


#endif // LAS_CONVERSIONS_