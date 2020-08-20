
#ifndef LAS_FILTERING_
#define LAS_FILTERING_

#include <dirt_or_leaf/las_conversions.h>

// PCL SAC Segmentation (Plane)
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/crop_box.h>

// PCL Search and Filters
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>

#include <dirt_or_leaf/timer.h>

#include <pcl/kdtree/kdtree_flann.h>

//std::vector<int> getNeighbors(pcl::Point2DIndex source, SCP target, bool keep_self=false);

#endif //LAS_FILTERING_