
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

namespace las_filtering{

// ---- Decimation ----
// Reduce cloud density, keeping only one local *minimum* point for every DECIMATION_FACTOR points
template <typename CloudType, typename Cloud2DType, typename PointType> 
void decimateToMinima(CloudType input, Cloud2DType input_flat, CloudType output, int decimation_factor);
// Reduce cloud density, keeping only one local *maximum* point for every DECIMATION_FACTOR points
template <typename CloudType, typename Cloud2DType, typename PointType> 
void decimateToMinima(CloudType input, Cloud2DType input_flat, CloudType output, int decimation_factor);


// ---- Last Return Filter ----
// Remove all points EXCEPT the last returns 
template <typename CloudType> 
void filterToLastReturn(CloudType input, CloudType output);
template <typename CloudType, typename Cloud2DType> 
void filterToLastReturn(CloudType data, Cloud2DType input, Cloud2DType output);

}

//std::vector<int> getNeighbors(pcl::Point2DIndex source, SCP target, bool keep_self=false);

#endif //LAS_FILTERING_