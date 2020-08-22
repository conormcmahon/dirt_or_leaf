
#ifndef LAS_FILTERING_
#define LAS_FILTERING_

// Dirt_Or_Leaf includes
#include <dirt_or_leaf/las_conversions.h>
#include <dirt_or_leaf/timer.h>

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

#include <pcl/kdtree/kdtree_flann.h>

namespace las_filtering{

    // ---- Decimation ----
    // Reduce cloud density, keeping only one local *minimum* point for every DECIMATION_FACTOR points
    template <typename CloudType, typename Cloud2DType, typename PointType> 
    void decimateToMinima(CloudType data, Cloud2DType input_flat, Cloud2DType output, int decimation_factor);
    // Reduce cloud density, keeping only one local *maximum* point for every DECIMATION_FACTOR points
    template <typename CloudType, typename Cloud2DType, typename PointType> 
    void decimateToMaxima(CloudType data, Cloud2DType input_flat, Cloud2DType output, int decimation_factor);

    // Estimate Cloud Normals
    template <typename CloudType, typename CloudNormalsType, typename PointType, typename NormalsType> 
    void estimateNormals(CloudType input, CloudNormalsType normals, bool radius_search, float search_parameter, bool enforce_vertical=true);

    // Estimate Roughness
    template <typename InputCloudType, typename OutputCloudType, typename InputPointType> 
    void estimateRoughness(InputCloudType input, OutputCloudType output, bool radius_search, float search_parameter);


    // ---- Last Return Filter ----
    // Remove all points EXCEPT the last returns 
    template <typename CloudType> 
    void filterToLastReturn(CloudType input, CloudType output);
    template <typename CloudType, typename Cloud2DType> 
    void filterToLastReturn(CloudType data, Cloud2DType input, Cloud2DType output);


    // De-Meaning 
    template <typename CloudType>
    Eigen::Vector3d deMeanCloud(CloudType cloud);

}

//std::vector<int> getNeighbors(pcl::Point2DIndex source, SCP target, bool keep_self=false);

#endif //LAS_FILTERING_