
#ifndef LAS_CLASSIFIER_
#define LAS_CLASSIFIER_

#include <dirt_or_leaf/las_filtering.hpp>
#include <dirt_or_leaf/las_surfacing.hpp>
#include <dirt_or_leaf/las_conversions.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/ply_io.h>

template <typename LASType, typename VegType, typename GroundType> 
class LASClassifier{

public:
  // Point Cloud Type Defines (for convenience)
    // LAS Point Type (xyz, intensity, classification, gps_time, etc.)
    typedef typename pcl::PointCloud<LASType> LC;
    typedef typename pcl::PointCloud<LASType>::Ptr LCP;
    // Vegetation Point Type (xyz, intensity, color, height, rugosity)
    typedef typename pcl::PointCloud<VegType> VC;
    typedef typename pcl::PointCloud<VegType>::Ptr VCP;
    // 2D Point Type with Ground Information (to LAS source point)
    typedef typename pcl::PointCloud<GroundType> GC;
    typedef typename pcl::PointCloud<GroundType>::Ptr GCP;
  // KdTree Defines
    typedef typename pcl::KdTreeFLANN<GroundType> tree_2D;
    typedef typename pcl::KdTreeFLANN<GroundType>::Ptr tree_2DP;

  // Constructors
    LASClassifier();
    LASClassifier(bool demean);

  // Initializing Helper Functions
    void initializeClouds();

  // Member Function
    void setInputLAS(LCP input);
    int loadLASPCD(std::string filename);
    // Output Options
    void setDebugging(bool debugging);
    void setTimekeeping(bool timekeeping);
    void setDecimationDebugging(bool decimation_debugging);
    void setGroundFilterDebugging(bool ground_filter_debugging);
    void setOutputOptions(bool save_outputs, std::string output_director="", std::string scene_name="");
    // Filter to Ground
    void decimateToMinima(int decimation_factor, bool return_information);
    void decimateToMaxima(int decimation_factor, bool return_information);
    void curvatureAnalysis(int num_neighbors, int roughness_neighbors);

private:
  // Point Clouds 
    LCP input_las_;
    GCP input_las_flattened_;
    // Initial minima search (decimation)
    GCP ground_decimated_;
    // Fine ground point filtering 
    GCP ground_filtered_;
    // Final ground cloud (after re-adding excluded points)
    GCP ground_;
  // Search Trees on Clouds
    tree_2DP input_tree_;
    tree_2DP ground_decimated_tree_;
    tree_2DP ground_filtered_tree_;
    tree_2DP ground_tree_;

  // Should cloud be de-meaned and scaled in XYZ to prevent overflow for very large coordinate systems? 
    bool demean_;

  // Output Options
    bool timekeeping_;
    bool decimation_debugging_;
    bool ground_filter_debugging_;
    bool save_outputs_;
    std::string output_directory_;
    std::string scene_name_;

    // Output in 2D
    template <typename CloudType, typename PointType>
    void outputPCD(CloudType cloud, std::string filename, bool ascii=true);
    // Output in Original Data Type 
    template <typename CloudType, typename Cloud2DType, typename PointType>
    void outputPCD(CloudType data, Cloud2DType cloud, std::string filename, bool ascii=true);

  // Timekeeping
    float loading_time_;
    float preprocessing_time_;
    float decimation_time_;
    float ground_filter_time_;
    float upsampling_time_;

};



#endif // LAS_CLASSIFIER_