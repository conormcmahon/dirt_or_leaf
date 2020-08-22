
#include "dirt_or_leaf/las_classifier.h"


// ------------------------------------ Constructors / Setup ------------------------------------
// Default constructor
template <typename LASType, typename VegType, typename GroundType> 
LASClassifier<LASType, VegType, GroundType>::LASClassifier(){
    timekeeping_ = true;
    decimation_debugging_ = true;
    ground_filter_debugging_ = true;
    save_outputs_ = false;
    demean_ = false;
    initializeClouds();
}

template <typename LASType, typename VegType, typename GroundType> 
LASClassifier<LASType, VegType, GroundType>::LASClassifier(bool demean){
    timekeeping_ = true;
    decimation_debugging_ = true;
    ground_filter_debugging_ = true;
    save_outputs_ = false;
    demean_ = demean;
    initializeClouds();
}

template <typename LASType, typename VegType, typename GroundType> 
void LASClassifier<LASType, VegType, GroundType>::initializeClouds(){
    // Initialize Point Cloud Smart Pointer Targets
    input_las_.reset(new LC());
    input_las_flattened_.reset(new GC());
    ground_decimated_.reset(new GC());
    ground_filtered_.reset(new GC());
    ground_.reset(new GC());
    // Initialize Point Cloud Smart Pointer Targets
    input_tree_.reset(new tree_2D());
    ground_decimated_tree_.reset(new tree_2D());
    ground_filtered_tree_.reset(new tree_2D());
    ground_tree_.reset(new tree_2D());
}


// ------------------------------------ Loading Inputs ------------------------------------
// Load Input from LAS PCL Cloud in Memory
template <typename LASType, typename VegType, typename GroundType> 
void LASClassifier<LASType, VegType, GroundType>::setInputLAS(LCP input)
{
    
}
// Load Input from .PCD file Saved on Disk
template <typename LASType, typename VegType, typename GroundType> 
int LASClassifier<LASType, VegType, GroundType>::loadLASPCD(std::string filename)
{
    if(timekeeping_)
        Timer timer("loading cloud");
    if (pcl::io::loadPCDFile<LASType> (filename, *input_las_) == -1) //* load the file
    {
        std::cout << "\nCouldn't read file data/" << filename << std::endl;
        return (-1);
    }
    std::cout << "Loaded " << input_las_->width * input_las_->height << " data points from data/" << filename << std::endl;
    if(demean_)
    {
        Timer demeaning("Demeaning");
        las_filtering::deMeanCloud<LCP>(input_las_);
        demeaning.stop();
    }
    pcl::copyPointCloud3D<LCP, GCP>(input_las_, input_las_flattened_);
    pcl::generatePointCloudIndices<GCP>(input_las_flattened_);            // allows tracking of unique point index in original cloud
    input_tree_->setInputCloud(input_las_flattened_);

    return 1;
}


// ------------------------------------ Debugging Options ------------------------------------
// Master set file which switches all debugging options on or off
template <typename LASType, typename VegType, typename GroundType> 
void LASClassifier<LASType, VegType, GroundType>::setDebugging(bool debugging)
{
    timekeeping_ = debugging;
    decimation_debugging_ = debugging;
    ground_filter_debugging_ = debugging;
    save_outputs_ = debugging;
}
// Set whether to keep track of execution time
template <typename LASType, typename VegType, typename GroundType> 
void LASClassifier<LASType, VegType, GroundType>::setTimekeeping(bool timekeeping)
{
    timekeeping_ = timekeeping;
}
// Set whether to output debugging information on decimation phase
template <typename LASType, typename VegType, typename GroundType> 
void LASClassifier<LASType, VegType, GroundType>::setDecimationDebugging(bool decimation_debugging)
{
    decimation_debugging_ = decimation_debugging;
}
// Set whether to output debugging information on decimation phase
template <typename LASType, typename VegType, typename GroundType> 
void LASClassifier<LASType, VegType, GroundType>::setGroundFilterDebugging(bool ground_filter_debugging)
{
    ground_filter_debugging_ = ground_filter_debugging; 
}
// Set whether to output saved PCD cloud files
template <typename LASType, typename VegType, typename GroundType> 
void LASClassifier<LASType, VegType, GroundType>::setOutputOptions(bool save_outputs, std::string output_directory, std::string scene_name)
{
    save_outputs_ =  save_outputs;
    if(!save_outputs_)
        return;
    output_directory_ = output_directory;
    scene_name_ = scene_name;
    if(output_directory.length() < 1)
        std::cout << "[LASClassifier] Warning - instructed to save output clouds, but no output directory string provided.";
    if(scene_name.length() < 1)
        std::cout << "[LASClassifier] Warning - instructed to save output clouds, but no output directory string provided.";
}



// ------------------------------------ Output Files ------------------------------------
// Output a Point Cloud to PCD
template <typename LASType, typename VegType, typename GroundType>           // Class Template
template <typename CloudType, typename PointType>       // Member Function Template
void LASClassifier<LASType, VegType, GroundType>::outputPCD(CloudType cloud, std::string filename, bool ascii)
{ 
    pcl::PCDWriter writer;
    writer.write<PointType>(filename, *cloud, ascii);
}
// Output a Point Cloud to PCD (using indices)
template <typename LASType, typename VegType, typename GroundType>           // Class Template
template <typename CloudType, typename Cloud2DType, typename PointType>       // Member Function Template
void LASClassifier<LASType, VegType, GroundType>::outputPCD(CloudType data, Cloud2DType cloud, std::string filename, bool ascii)
{
    // Create data cloud at given indices
    LCP output(new LC);
    for(std::size_t i=0; i<cloud->points.size(); i++)
        output->points.push_back(data->points[cloud->points[i].index]);

    pcl::PCDWriter writer;
    writer.write<PointType>(filename, *output, ascii);
}



// ------------------------------------ Ground Segmentation ------------------------------------
// Decimate point cloud, keeping only the LOWEST point within each group of DECIMATION_FACTOR points
// Optionally can also choose to filter for only the last returns, following decimation
template <typename LASType, typename VegType, typename GroundType> 
void LASClassifier<LASType, VegType, GroundType>::decimateToMinima(int decimation_factor, bool return_information)
{
    if(decimation_debugging_)
        std::cout << "Performing cloud decimation with factor " << decimation_factor << ". Initial cloud size: " << input_las_->points.size() << std::endl;
    Timer timer("decimation");
    las_filtering::decimateToMinima<LCP, GCP, GroundType>(input_las_, input_las_flattened_, ground_decimated_, decimation_factor);
    if(return_information)
    {
        GCP last_returns(new GC);
        las_filtering::filterToLastReturn<LCP, GCP>(input_las_, ground_decimated_, last_returns);
        *ground_decimated_ = *last_returns;
    }
    if(decimation_debugging_)
        std::cout << "Finished decimation. Writing " << ground_decimated_->points.size() << " to " << output_directory_ + scene_name_ + std::string("_decimated.pcd") << std::endl;
    if(save_outputs_)
        outputPCD<LCP, GCP, LASType>(input_las_, ground_decimated_, output_directory_ + scene_name_ + std::string("_decimated.pcd"), true);
    if(timekeeping_)
        decimation_time_ = timer.stop();
}

// Decimate point cloud, keeping only the HIGHEST point within each group of DECIMATION_FACTOR points
template <typename LASType, typename VegType, typename GroundType> 
void LASClassifier<LASType, VegType, GroundType>::decimateToMaxima(int decimation_factor, bool return_information)
{


}


template <typename LASType, typename VegType, typename GroundType> 
void LASClassifier<LASType, VegType, GroundType>::curvatureAnalysis(int num_neighbors, int roughness_neighbors)
{
    Timer timer("normals");
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_decimated_temp(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::copyPointCloud3D<GCP, pcl::PointCloud<pcl::PointXYZI>::Ptr>(ground_decimated_, ground_decimated_temp);
    std::cout << ground_decimated_->points[1000].z << " " << ground_decimated_->points[5000].z << " " << ground_decimated_->points[2000].z << " " << ground_decimated_->points[4000].z << std::endl;
    
    las_filtering::estimateNormals<pcl::PointCloud<pcl::PointXYZI>::Ptr, GCP, pcl::PointXYZI, GroundType>(ground_decimated_temp, ground_decimated_, true, float(num_neighbors), true);
    outputPCD<GCP, GroundType>(ground_decimated_, "/home/conor/lidar_data/normals.pcd", true);
    timer.stop();

    GCP roughness(new GC);
    las_filtering::estimateRoughness<GCP, GCP, GroundType>(ground_decimated_, roughness, false, 10);
    outputPCD<GCP, GroundType>(roughness, "/home/conor/lidar_data/roughness.pcd", true);

    GCP filtered(new GC);
    for(std::size_t i=0; i<roughness->points.size(); i++)
    {
        if(roughness->points[i].height_diff_avg < 0.5)
            if(roughness->points[i].norm_diff_avg < 0.25)
                filtered->push_back(roughness->points[i]);
    }
    outputPCD<GCP, GroundType>(filtered, "/home/conor/lidar_data/roughness_filter.pcd", true);

    GCP roughness_second(new GC);
    las_filtering::estimateRoughness<GCP, GCP, GroundType>(filtered, roughness_second, false, 10);
    outputPCD<GCP, GroundType>(roughness_second, "/home/conor/lidar_data/roughness_second.pcd", true);

    GCP filtered_again(new GC);
    for(std::size_t i=0; i<roughness_second->points.size(); i++)
    {
        if(roughness_second->points[i].height_diff_avg < 0.5)
            if(roughness_second->points[i].norm_diff_avg < 0.5)
                filtered_again->push_back(roughness_second->points[i]);
    }
    outputPCD<GCP, GroundType>(filtered_again, "/home/conor/lidar_data/ground.pcd", true);
}

