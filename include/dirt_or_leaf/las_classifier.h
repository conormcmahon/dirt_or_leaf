
#ifndef LAS_CLASSIFIER_
#define LAS_CLASSIFIER_

#include <dirt_or_leaf/las_filtering.hpp>
#include <dirt_or_leaf/las_surfacing.hpp>
#include <dirt_or_leaf/las_conversions.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/ply_io.h>

template <typename LASType, typename VegType> 
class LASClassifier{

public:
  // Point Cloud Type Defines (for convenience)
    // LAS Point Type (xyz, intensity, classification, gps_time, etc.)
    typedef typename pcl::PointCloud<LASType> LC;
    typedef typename pcl::PointCloud<LASType>::Ptr LCP;
    // Vegetation Point Type (xyz, intensity, color, height, rugosity)
    typedef typename pcl::PointCloud<VegType> VC;
    typedef typename pcl::PointCloud<VegType>::Ptr VCP;
    // XYZ + Intensity Point Type
    typedef typename pcl::PointCloud<pcl::PointXYZI> PC;
    typedef typename pcl::PointCloud<pcl::PointXYZI>::Ptr PCP;
    // 2D Point Type with Index (to LAS source point)
    typedef typename pcl::PointCloud<pcl::Point2DIndex> SC;
    typedef typename pcl::PointCloud<pcl::Point2DIndex>::Ptr SCP;
  // KdTree Defines
    typedef typename pcl::KdTreeFLANN<pcl::Point2DIndex>::Ptr tree_2D;

  // Constructors
    LASClassifier();

  // Member Function
    void setInputLAS(LCP input);
    int loadLASPCD(std::string filename);
    // Output Options
    void setTimekeeping(bool timekeeping);
    void setDecimationDebugging(bool decimation_debugging);
    void setGroundFilterDebugging(bool ground_filter_debugging_);
    void setOutputOptions(bool save_outputs, std::string output_director="", std::string scene_name="");
    // Filter to Ground
    void decimateToMinima(int decimation_factor);
    void decimateToMaxima(int decimation_factor);

private:
  // Point Clouds 
    LCP input_las_;
    SCP input_las_flattened_;
    // Initial minima search (decimation)
    SCP ground_decimated_;
    // Fine ground point filtering 
    SCP ground_filtered_;
    // Final ground cloud (after re-adding excluded points)
    SCP ground_;
  // Search Trees on Clouds
    tree_2D input_tree_;
    tree_2D ground_decimated_tree_;
    tree_2D ground_filtered_tree_;
    tree_2D ground_tree_;

  // Output Options
    bool timekeeping_;
    bool decimation_debugging_;
    bool ground_filter_debugging_;
    bool save_outputs_;
    std::string output_directory_;
    std::string scene_name_;

    template<typename CloudType, typename PointType> void outputPCD(CloudType cloud, std::string filename);
    template<typename CloudType, typename PointType> void outputPCDASCII(CloudType cloud, std::string filename);

  // Timekeeping
    float loading_time_;
    float preprocessing_time_;
    float decimation_minima_time_;
    float ground_filter_time_;
    float upsampling_time_;

};



#endif // LAS_CLASSIFIER_