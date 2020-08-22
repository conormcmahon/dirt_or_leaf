
#include "dirt_or_leaf/las_conversions.h"

namespace pcl
{
    // Force output cloud to match size of input cloud
    // If output cloud has nonzero size which does not match input, returns false without changing anything
    template <typename CloudIn, typename CloudOut>
    bool alignCloudSizes(CloudIn input, CloudOut output)
    {    
        if(output->points.size() != input->points.size())
        {
            if(output->points.size() != 0)
            {
                std::cout << "ERROR: Attempting to copy point normals between two clouds of different nonzero size.";
                return false;
            }
            output->points.resize(input->size());
        }
        output->width = 1;
        output->height = output->points.size();
        return true;
    }

    // Create Pointcloud in 2D space to do XY neighbor searches
    template <typename CloudIn, typename CloudOut> 
    void copyPointCloud3D(CloudIn input, CloudOut output)
    {
        if(!alignCloudSizes(input, output)) 
            return;
        for(std::size_t i=0; i<input->points.size(); i++)
        {
            output->points[i].x = input->points[i].x;
            output->points[i].y = input->points[i].y;
            output->points[i].z = input->points[i].z;
        }
    }

    // Create Pointcloud in 2D space to do XY neighbor searches
    template <typename CloudIn, typename CloudOut> 
    void copyPointCloud2D(CloudIn input, CloudOut output)
    {
        if(!alignCloudSizes(input, output)) 
            return;
        for(std::size_t i=0; i<input->points.size(); i++)
        {
            output->points[i].x = input->points[i].x;
            output->points[i].y = input->points[i].y;
        }
    }

    // Create Pointcloud in 2D space to do XY neighbor searches, retaining point index
    template <typename CloudIn, typename CloudOut> 
    void copyPointCloud2DIndex(CloudIn input, CloudOut output)
    {
        if(!alignCloudSizes(input, output)) 
            return;
        for(std::size_t i=0; i<input->points.size(); i++)
        {
            output->points[i].x = input->points[i].x;
            output->points[i].y = input->points[i].y;
            output->points[i].index = i;
        }
    }

//    // Create Pointcloud in 2D space to do XY neighbor searches
//    template <typename CloudIn, typename CloudOut> 
//    void copyPointCloud2_5D(CloudIn input, CloudOut output)
//    {
//        if(!alignCloudSizes(input, output)) 
//            return;
//        for(std::size_t i=0; i<input->points.size(); i++)
//        {
//            output->points[i].x = input->points[i].x;
//            output->points[i].y = input->points[i].y;
//            output->points[i].z = input->points[i].z;
//        }
//    }

    // Create Pointcloud indices between clouds
    template <typename CloudIn, typename CloudOut> 
    void copyPointCloudIndices(CloudIn input, CloudOut output)
    {
        if(!alignCloudSizes(input, output)) 
            return;
        for(std::size_t i=0; i<input->points.size(); i++)
        {
            output->points[i].index = input->points[i].index;
        }
    }

    // Create Pointcloud indices between clouds
    template <typename CloudIn> 
    void generatePointCloudIndices(CloudIn input)
    {
        for(std::size_t i=0; i<input->points.size(); i++)
        {
            input->points[i].index = i;
        }
    }

    // Create Pointcloud in 2D space to do XY neighbor searches
    template <typename Cloud3D, typename CloudIn, typename CloudOut> 
    void copyPointCloud3D(Cloud3D xyz, CloudIn input, CloudOut output)
    {
        if(!alignCloudSizes(input, output)) 
            return;
        for(std::size_t i=0; i<input->points.size(); i++)
        {
            output->points[i].x = xyz->points[input->points[i].index].x;
            output->points[i].y = xyz->points[input->points[i].index].y;
            output->points[i].z = xyz->points[input->points[i].index].z;
        }
    }

    
    // Create Pointcloud in 2D space to do XY neighbor searches
    template <typename CloudIn, typename CloudOut> 
    void copyPointCloudNormals(CloudIn input, CloudOut output)
    {
        if(!alignCloudSizes(input, output)) 
            return;
        for(std::size_t i=0; i<input->points.size(); i++)
        {
            output->points[i].normal_x = input->points[i].normal_x;
            output->points[i].normal_y = input->points[i].normal_y;
            output->points[i].normal_z = input->points[i].normal_z;
            output->points[i].curvature = input->points[i].curvature;
        }
    }
}