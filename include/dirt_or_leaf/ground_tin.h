
#ifndef GROUND_TIN_
#define GROUND_TIN_

#include <dirt_or_leaf/las_triangulation.hpp>
#include <dirt_or_leaf/las_filtering.hpp>
#include <pcl/kdtree/kdtree_flann.h>

template <typename GroundType>
class GroundTIN
{
public:
    // 2D Point Type with Ground Information (to LAS source point)
    typedef typename pcl::PointCloud<GroundType> GC;
    typedef typename pcl::PointCloud<GroundType>::Ptr GCP;
    // 2D Search on Ground 
    typedef typename pcl::KdTreeFLANN<GroundType> Tree2D;
    typedef typename pcl::KdTreeFLANN<GroundType>::Ptr Tree2DP;

    GroundTIN();
    void setInputCloud(GCP cloud);
    void setInputCloud(GCP cloud, Tree2DP tree);
    void generateTIN();
    template <typename CloudType, typename PointType>
    float getPointHeight(CloudType source_cloud, PointType point);
    void saveTIN(std::string filename, bool remean=false, Eigen::Vector3d offset=Eigen::Vector3d::Zero());

private: 
    Tree2DP tree_;
    GCP cloud_;
    las_triangulation::Delaunay TIN_;
    std::vector<las_triangulation::Delaunay::Face_handle> face_mapping_;

};

#endif // GROUND_TIN_