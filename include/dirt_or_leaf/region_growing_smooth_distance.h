
#ifndef REGION_GROWING_SMOOTH_DISTANCE_
#define REGION_GROWING_SMOOTH_DISTANCE_

#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/region_growing.h>

#include <pcl/pcl_base.h>
#include <pcl/search/search.h>
#include <pcl/point_cloud.h>
#include <list>
#include <math.h>
#include <time.h>
#include <queue>

namespace pcl{

template <typename PointT, typename NormalT>
class RegionGrowingSmoothDistance : public pcl::RegionGrowing<PointT, NormalT>
{
public:
//    typedef typename pcl::KdTreeFLANN<PointType>::Ptr Tree;

    RegionGrowingSmoothDistance();

//    void setSearchMethod(Tree tree);
//    void setThresholdDistance
    void setDistanceThreshold(float distance);
    virtual bool
    validatePoint (int initial_seed, int point, int nghbr, bool& is_a_seed) const;

private: 
    float distance_threshold_;
//    Tree search_;
};

}

#endif //REGION_GROWING_SMOOTH_DISTANCE_