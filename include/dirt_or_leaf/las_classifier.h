
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
    // 3D Search on LAS Type
    typedef typename pcl::KdTreeFLANN<LASType> Tree3D;
    typedef typename pcl::KdTreeFLANN<LASType>::Ptr Tree3DP;
    // 2D Search on Ground 
    typedef typename pcl::KdTreeFLANN<GroundType> Tree2D;
    typedef typename pcl::KdTreeFLANN<GroundType>::Ptr Tree2DP;

  // Constructors
    LASClassifier();
    LASClassifier(bool demean, bool remean);

  // Initializing Helper Functions
    void initializeClouds();

  // Member Function
    void setInputLAS(LCP input);
    int loadLASPCD(std::string filename);
    // Output Options
    void setVerbosity(bool verbosity);
    void setTimekeeping(bool timekeeping);
    void setDebugging(bool debugging);
    void setOutputOptions(bool save_outputs, std::string output_directory, std::string scene_name, bool demean=true, bool remean=true);
    // Filter to Ground
    void decimateToMinima(int decimation_factor, bool return_information);
    void decimateVegetation(int decimation_factor, bool return_information);
    void curvatureAnalysis(int num_neighbors, int roughness_neighbors);
    // Vegetation Extraction
    void extractVegetation(float min_height);


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
    // Vegetation Cloud
    VCP vegetation_;
    VCP vegetation_decimated_;
  // Search Trees on Clouds
    // 3D Search Trees
    Tree3DP input_las_tree_;
    // 2D Search Trees
    Tree2DP input_tree_;
    Tree2DP ground_decimated_tree_;
    Tree2DP ground_filtered_tree_;
    Tree2DP ground_tree_;

  // Should cloud be de-meaned and to prevent overflow for very large coordinate systems? 
    bool demean_;
  // Should the cloud be re-meaned and moved back to its original coordinates before being output?  
    bool remean_; 
    Eigen::Vector3d offset_;

  // Output Options
    bool timekeeping_;
    bool debugging_;
    bool save_outputs_;
    std::string output_directory_;
    std::string scene_name_;

    // Output in 2D
    template <typename CloudType, typename PointType>
    void outputPCD(CloudType cloud, std::string filename, bool binary=true);
    // Output in Original Data Type 
    template <typename CloudType, typename Cloud2DType, typename PointType>
    void outputPCD(CloudType data, Cloud2DType cloud, std::string filename, bool binary=true);

  // Timekeeping
    float loading_time_;
    float preprocessing_time_;
    float decimation_time_;
    float ground_filter_time_;
    float upsampling_time_;
    float veg_decimation_time_;

};



#endif // LAS_CLASSIFIER_