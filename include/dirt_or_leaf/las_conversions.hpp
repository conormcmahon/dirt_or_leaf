
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

/*    // Create Pointcloud in 2D space to do XY neighbor searches
    void copyPointCloud(pcl::PointCloud<pcl::PointLAS>::Ptr input, pcl::PointCloud<pcl::Point2DScale>::Ptr output)
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

    void copyPointCloud(pcl::PointCloud<pcl::Point2DScale>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output)
    {
    output->points.resize(input->size());
    for(std::size_t i=0; i<input->points.size(); i++)
    {
        output->points[i].x = input->points[i].x;
        output->points[i].y = input->points[i].y;
        output->points[i].z = input->points[i].scale;
    } 
    output->width = 1;
    output->height = output->points.size();
    }

    // Convert cloud with LAS Point format to Veg Point format
    void copyPointCloud(pcl::PointCloud<pcl::PointLAS>::Ptr input, pcl::PointCloud<pcl::PointVeg>::Ptr output)
    {
    for(std::size_t i = 0; i<input->points.size(); i++)
    {
        pcl::PointLAS las = input->points[i];
        pcl::PointVeg veg;

        veg.x = las.x;  
        veg.y = las.y;
        veg.z = las.z;
        veg.intensity = las.intensity;
        output->points.push_back(veg);
    }
    output->width = 1;
    output->height = output->points.size();
    }
    // Convert cloud with LAS Point format to XYZ Point format
    void copyPointCloud(pcl::PointCloud<pcl::PointLAS>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output)
    {
    for(std::size_t i = 0; i<input->points.size(); i++)
    {
        pcl::PointLAS las = input->points[i];
        pcl::PointXYZ xyz;

        xyz.x = las.x;  
        xyz.y = las.y;
        xyz.z = las.z;
        output->points.push_back(xyz);
    }
    output->width = 1;
    output->height = output->points.size();
    }
    // Convert cloud with LAS Point format to XYZ Point format
    void copyPointCloud(pcl::PointCloud<pcl::PointLAS>::Ptr input, pcl::PointCloud<pcl::PointXYZI>::Ptr output)
    {
    for(std::size_t i = 0; i<input->points.size(); i++)
    {
        pcl::PointLAS las = input->points[i];
        pcl::PointXYZI xyzi;

        xyzi.x = las.x;   
        xyzi.y = las.y;
        xyzi.z = las.z;
        xyzi.intensity = 0;
        output->points.push_back(xyzi);
    }
    output->width = 1;
    output->height = output->points.size();
    }
    // Convert cloud with Veg Point format to XYZ Point format
    void copyPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZI>::Ptr output)
    {
    for(std::size_t i = 0; i<input->points.size(); i++)
    {
        pcl::PointXYZ xyz = input->points[i];
        pcl::PointXYZI xyzi;

        xyzi.x = xyz.x;
        xyzi.y = xyz.y;
        xyzi.z = xyz.z;

        output->points.push_back(xyzi);
    }
    output->width = 1;
    output->height = output->points.size();
    }
    // Convert cloud with Veg Point format to XYZ Point format
    void copyPointCloud(pcl::PointCloud<pcl::PointVeg>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output)
    {
    for(std::size_t i = 0; i<input->points.size(); i++)
    {
        pcl::PointVeg veg = input->points[i];
        pcl::PointXYZ point;

        point.x = veg.x;
        point.y = veg.y;
        point.z = veg.z;

        output->points.push_back(point);
    }
    output->width = 1;
    output->height = output->points.size();
    } */
}