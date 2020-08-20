
#include "dirt_or_leaf/las_classifier.h"


// ------------------------------------ Constructors / Setup ------------------------------------
// Default constructor
template <typename LASType, typename VegType> 
LASClassifier<LASType, VegType>::LASClassifier(){
    timekeeping_ = true;
    decimation_debugging_ = true;
    ground_filter_debugging_ = true;
    save_outputs_ = false;
}


// ------------------------------------ Loading Inputs ------------------------------------
// Load Input from LAS PCL Cloud in Memory
template <typename LASType, typename VegType> 
void LASClassifier<LASType, VegType>::setInputLAS(LCP input)
{
    
}
// Load Input from .PCD file Saved on Disk
template <typename LASType, typename VegType> 
int LASClassifier<LASType, VegType>::loadLASPCD(std::string filename)
{
    if (pcl::io::loadPCDFile<pcl::PointLAS> (filename, *input_las_) == -1) //* load the file
    {
        std::cout << "\nCouldn't read file data/" << filename << ".pcd \n";
        return (-1);
    }
    std::cout << "Loaded " << input_las_->width * input_las_->height << " data points from data/" << filename << ".pcd with the following fields: " << std::endl;
    return 1;

    pcl::copyPointCloud2DIndex(input_las_, input_las_flattened_);
    input_tree_->setInputCloud(input_las_flattened_);
}


// ------------------------------------ Debugging Options ------------------------------------
// Set whether to keep track of execution time
template <typename LASType, typename VegType> 
void LASClassifier<LASType, VegType>::setTimekeeping(bool timekeeping)
{
    timekeeping_ = timekeeping;
}
// Set whether to output debugging information on decimation phase
template <typename LASType, typename VegType> 
void LASClassifier<LASType, VegType>::setDecimationDebugging(bool decimation_debugging)
{
    decimation_debugging_ = decimation_debugging;
}
// Set whether to output debugging information on decimation phase
template <typename LASType, typename VegType> 
void LASClassifier<LASType, VegType>::setGroundFilterDebugging(bool ground_filter_debugging)
{
    ground_filter_debugging_ = ground_filter_debugging; 
}
// Set whether to output saved PCD cloud files
template <typename LASType, typename VegType> 
void LASClassifier<LASType, VegType>::setOutputOptions(bool save_outputs, std::string output_directory, std::string scene_name)
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



// ------------------------------------ Ground Segmentation ------------------------------------
// Decimate point cloud, keeping only the LOWEST point within each group of DECIMATION_FACTOR points
template <typename LASType, typename VegType> 
void LASClassifier<LASType, VegType>::decimateToMinima(int decimation_factor)
{
    las_filtering::decimateToMinima(input_las_, input_las_flattened_, ground_decimated_, decimation_factor);
    
    if(save_outputs_)
        outputPCD(ground_decimated_, output_directory_ + scene_name_ + std::string("_decimated.pcd"));
}

// Decimate point cloud, keeping only the HIGHEST point within each group of DECIMATION_FACTOR points
template <typename LASType, typename VegType> 
void LASClassifier<LASType, VegType>::decimateToMaxima(int decimation_factor)
{


}


// ------------------------------------ Output Files ------------------------------------
// Output a Point Cloud to PCD Binary
template <typename LASType, typename VegType>           // Class Template
template <typename CloudType, typename PointType>       // Member Function Template
void LASClassifier<LASType, VegType>::outputPCD(CloudType cloud, std::string filename)
{ 
    pcl::PCDWriter writer;
    writer.write<PointType>(filename, *cloud, false);
}
// Output a Point Cloud to PCD ASCII
template <typename LASType, typename VegType>           // Class Template
template <typename CloudType, typename PointType>       // Member Function Template
void LASClassifier<LASType, VegType>::outputPCDASCII(CloudType cloud, std::string filename)
{
    pcl::PCDWriter writer;
    writer.write<PointType>(filename, *cloud, false);
}