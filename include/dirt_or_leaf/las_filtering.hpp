

#include <dirt_or_leaf/las_filtering.h>

namespace las_filtering{

// ---- Decimation ----
// Note that this does not guarantee that each retained point is a local minimum within its own neighborhood of DECIMATION_FACTOR points.
// For efficiency, not every point has its neighborhood checked - points which are found to be higher than a neighbor are disqualified before 
//   being evaluated themselves.
// Because point density varies across the cloud this can have unintuitive effects, wherein points on consistent slopes (which are NOT local minima)
//   can be output as decimated minima when all points lower on the slope are disqualified before the higher point is considered.
// Cloud Types:
//   INPUT is assumed to be a cloud of points containing relatively large amounts of LAS data, e.g. return number 
//   INPUT_FLAT is a restructured version of the above which contains only 2D information (to search on 2D KdTrees)
//   OUTPUT is also a flattened, simple 2D cloud but each point should contain an index to its parent point in INPUT (where data is stored)
template <typename CloudType, typename Cloud2DType, typename PointFlat> 
void decimateToMinima(CloudType input, Cloud2DType input_flat, CloudType output, int decimation_factor, bool debug)
{
    // Stores whether each input point is a minimum
    std::vector<bool> retained_after_decimation(input_flat->points.size(), true);
    pcl::KdTreeFLANN<PointFlat> search_tree;
    search_tree.setInputCloud(input_flat);
    for(std::size_t i=0; i<input_flat->points.size(); i++)
    {
        // Disqualify points which we've already found to be higher than a neighbor
        if(!retained_after_decimation[i])
            continue;

        PointFlat putative_minimum = input_flat->points[i];
        // Find all neighbors of source point in input cloud
        std::vector<int> nearest_indices;
        std::vector<float> nearest_dists;
        search_tree.nearestKSearch(putative_minimum, decimation_factor+1, nearest_indices, nearest_dists);

        // Check whether putative minimum is a minimum within its neighborhood
        for(int neighbor_ind=0; neighbor_ind<nearest_indices.size(); neighbor_ind++)
        {
            // Skip self
            if(i == nearest_indices[neighbor_ind])
                continue;
            // If Source is higher than Target, and therefore NOT a minimum
            if(input->points[i].z > input->points[nearest_indices[neighbor_ind]].z)  
            {
                retained_after_decimation[i] = false;
                break;
            }
            else // Target is higher than Source, and therefore NOT a minimum 
                retained_after_decimation[nearest_indices[neighbor_ind]] = false;
        }
        // Aggregate points if they're still a minimum after the above check 
        if(retained_after_decimation[i])
            output->points.push_back(putative_minimum);
    }
}

template <typename CloudType, typename PointType> 
void decimateToMinima(CloudType input, CloudType output, int decimation_factor)
{
    
}


}


// Assuming both input and output clouds have information on 
template <typename CloudType> 
void filterToLastReturn(CloudType input, CloudType output)
{
    for(std::size_t i=0; i<input->points.size(); i++)
        if(input->points[i].returnnumber != input->points[i].numberofreturns)
            output->points.push_back(input->points[i]);
}
// Assuming cloud to be processed is a simpler type containing only 2D information and an index which points
//   to the more complex data stored in the initial input cloud
template <typename CloudType, typename Cloud2DType> 
void filterToLastReturn(CloudType data, Cloud2DType input, Cloud2DType output)
{
    for(std::size_t i=0; i<input->points.size(); i++)
        if(data->points[input->points[i].index].returnnumber != data->points[input->points[i].index].numberofreturns)
            output->points.push_back(input->points[i]);
}      