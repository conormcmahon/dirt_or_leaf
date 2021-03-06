
#ifndef LAS_TIN_
#define LAS_TIN_

#include <dirt_or_leaf/las_triangulation.hpp>
#include <dirt_or_leaf/las_filtering.hpp>
#include <pcl/kdtree/kdtree_flann.h>

template <typename PointType>
class LAS_TIN
{
public:
    // 2.5D Point Type 
    typedef typename pcl::PointCloud<PointType> PC;
    typedef typename pcl::PointCloud<PointType>::Ptr PCP;
    // 2D Search on Points 
    typedef typename pcl::KdTreeFLANN<PointType> Tree2D;
    typedef typename pcl::KdTreeFLANN<PointType>::Ptr Tree2DP;

    LAS_TIN();
    void setInputCloud(PCP cloud);
    void setInputCloud(PCP cloud, Tree2DP tree);
    void generateTIN();
    // Get height of TIN at a given point (XY)
    template <typename TargetPointType>
    float interpolateTIN(TargetPointType point, float nodata_value=-9999) const;
    // Get TIN terrain information at a given point (XY):
    //    index 0: Slope (0 to pi)
    //    index 1: Aspect (0 to pi, Euclidean from East=0 to South=3/2*pi)
    template <typename TargetPointType>
    Eigen::Vector2f slopeAtPoint(TargetPointType point, float nodata_value=-9999) const;
    // Save to a .PLY triangles file
    void saveTIN(std::string filename, bool remean=false, Eigen::Vector3d offset=Eigen::Vector3d::Zero());

private: 
    Tree2DP tree_;
    PCP cloud_;
    las_triangulation::Delaunay TIN_;
    std::vector<las_triangulation::Delaunay::Face_handle> face_mapping_;

};

#endif // LAS_TIN_