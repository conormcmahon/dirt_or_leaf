
#include "dirt_or_leaf/las_conversions.h"

namespace pcl
{
    // Create Pointcloud in 2D space to do XY neighbor searches
    template <typename CloudIn, typename CloudOut> 
    void copyPointCloud3D(CloudIn input, CloudOut output)
    {
    output->points.resize(input->size());
    for(std::size_t i=0; i<input->points.size(); i++)
    {
        output->points[i].x = input->points[i].x;
        output->points[i].y = input->points[i].y;
        output->points[i].z = input->points[i].z;
    }
    output->width = 1;
    output->height = output->points.size();
    }

    // Create Pointcloud in 2D space to do XY neighbor searches
    template <typename CloudIn, typename CloudOut> 
    void copyPointCloud2D(CloudIn input, CloudOut output)
    {
    output->points.resize(input->size());
    for(std::size_t i=0; i<input->points.size(); i++)
    {
        output->points[i].x = input->points[i].x;
        output->points[i].y = input->points[i].y;
    }
    output->width = 1;
    output->height = output->points.size();
    }

    // Create Pointcloud in 2D space to do XY neighbor searches, retaining point index
    template <typename CloudIn, typename CloudOut> 
    void copyPointCloud2DIndex(CloudIn input, CloudOut output)
    {
    output->points.resize(input->size());
    for(std::size_t i=0; i<input->points.size(); i++)
    {
        output->points[i].x = input->points[i].x;
        output->points[i].y = input->points[i].y;
        output->points[i].index = i;
    }
    output->width = 1;
    output->height = output->points.size();
    }

    // Create Pointcloud in 2D space to do XY neighbor searches
    template <typename CloudIn, typename CloudOut> 
    void copyPointCloud2_5D(CloudIn input, CloudOut output)
    {
    output->points.resize(input->size());
    for(std::size_t i=0; i<input->points.size(); i++)
    {
        output->points[i].x = input->points[i].x;
        output->points[i].y = input->points[i].y;
        output->points[i].scale = input->points[i].z;
    }
    output->width = 1;
    output->height = output->points.size();
    }
}