
#ifndef LAS_FILTERING_
#define LAS_FILTERING_

// Basic Includes
#include <boost/make_shared.hpp>

// Dirt_Or_Leaf includes
#include <dirt_or_leaf/las_conversions.h>
#include <dirt_or_leaf/timer.h>
#include <dirt_or_leaf/las_triangulation.h>

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

    // ---- Classification Filter ----
    // Filter input cloud based on existing point class information 
    //   String Class Keys (using standard LAS classification codes)
    template <typename CloudType> 
    std::vector<int> classFilter(CloudType input, CloudType output, std::string class_name, bool remove=true);
    //   Integer Class Keys
    template <typename CloudType> 
    std::vector<int> classFilter(CloudType input, CloudType output, std::vector<int> class_codes, bool remove=true);

    // ---- Decimation ----
    // Reduce cloud density, keeping only one local *minimum* point for every DECIMATION_FACTOR points
    template <typename CloudType, typename Cloud2DType, typename PointType> 
    void decimateToMinima(CloudType data, Cloud2DType input_flat, Cloud2DType output, int decimation_factor, bool remove_buildings, bool remove_vegetation);
    template <typename CloudType, typename Cloud2DType, typename PointType> 
    void decimateToMinima(CloudType data, Cloud2DType input_flat, Cloud2DType output, int decimation_factor);
    template <typename CloudType, typename PointType> 
    void decimateToMinima(CloudType input, CloudType output, int decimation_factor, bool remove_buildings, bool remove_vegetation);
    template <typename CloudType, typename PointType> 
    void decimateToMinima(CloudType input, CloudType output, int decimation_factor);
    // Reduce cloud density, keeping only one local *maximum* point for every DECIMATION_FACTOR points
    template <typename CloudType, typename Cloud2DType, typename PointType> 
    void decimateToMaxima(CloudType data, Cloud2DType input_flat, Cloud2DType output, int decimation_factor);
    template <typename CloudType, typename PointType> 
    void decimateToMaxima(CloudType input, CloudType output, int decimation_factor);

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
    // Remove all points EXCEPT the first returns 
    template <typename CloudType> 
    void filterToFirstReturn(CloudType input, CloudType output);
    template <typename CloudType, typename Cloud2DType> 
    void filterToFirstReturn(CloudType data, Cloud2DType input, Cloud2DType output);


    // De-Meaning 
    template <typename CloudType>
    Eigen::Vector3d deMeanCloud(CloudType cloud);
    template <typename CloudType>
    void reMeanCloud(CloudType cloud, Eigen::Vector3d mmean_coords);
    template <typename CloudType>
    void reMeanCloud(CloudType input, CloudType output, Eigen::Vector3d mean_coords);

    // Relative Point Height
    //   Compared to another cloud
    template <typename CloudType, typename SourcePointType, typename KdTreeType>
    float relativePointHeight(CloudType cloud, SourcePointType point, KdTreeType tree, int num_neighbors);
    //   Compared to TIN surface
    template <typename CloudType, typename SourcePointType>
    float relativePointHeight(CloudType cloud, SourcePointType point, las_triangulation::Delaunay &ground);

}

//std::vector<int> getNeighbors(pcl::Point2DIndex source, SCP target, bool keep_self=false);

#endif //LAS_FILTERING_