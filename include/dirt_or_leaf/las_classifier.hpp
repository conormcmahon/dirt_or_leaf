
#include "dirt_or_leaf/las_classifier.h"


// ------------------------------------ Constructors ------------------------------------
// Default constructor
template <typename LASType, typename VegType> 
LASClassifier<LASType, VegType>::LASClassifier(){
    timekeeping_ = true;
    decimation_debugging_ = true;
    ground_filter_debugging_ = true;
}



// ------------------------------------ Loading Inputs ------------------------------------
// Load Input from LAS PCL Cloud in Memory
template <typename LASType, typename VegType> void 
LASClassifier<LASType, VegType>::setInputLAS(LCP input)
{
    
}
// Load Input from .PCD file Saved on Disk
template <typename LASType, typename VegType> int 
LASClassifier<LASType, VegType>::loadLASPCD(std::string filename)
{
    if (pcl::io::loadPCDFile<pcl::PointLAS> (filename, *input_las_) == -1) //* load the file
    {
        std::cout << "\nCouldn't read file data/" << filename << ".pcd \n";
        return (-1);
    }
    std::cout << "Loaded " << input_las_->width * input_las_->height << " data points from data/" << filename << ".pcd with the following fields: " << std::endl;
    return 1;
}
 


// ------------------------------------ Debugging Options ------------------------------------
// Set whether to keep track of execution time
template <typename LASType, typename VegType> void 
LASClassifier<LASType, VegType>::setTimekeeping(bool timekeeping)
{
    timekeeping_ = timekeeping;
}
// Set whether to output debugging information on decimation phase
template <typename LASType, typename VegType> void 
LASClassifier<LASType, VegType>::setDecimationDebugging(bool decimation_debugging)
{
    decimation_debugging_ = decimation_debugging;
}
// Set whether to output debugging information on decimation phase
template <typename LASType, typename VegType> void 
LASClassifier<LASType, VegType>::setGroundFilterDebugging(bool ground_filter_debugging)
{
    ground_filter_debugging_ = ground_filter_debugging; 
}




// ------------------------------------ Ground Segmentation ------------------------------------
// Decimate point cloud, keeping only the LOWEST point within each group of DECIMATION_FACTOR points
template <typename LASType, typename VegType> void 
LASClassifier<LASType, VegType>::decimateToMinima(int decimation_factor)
{


}

// Decimate point cloud, keeping only the HIGHEST point within each group of DECIMATION_FACTOR points
template <typename LASType, typename VegType> void 
LASClassifier<LASType, VegType>::decimateToMaxima(int decimation_factor)
{


}