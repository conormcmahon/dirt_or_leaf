

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
// -- For cases where input and output cloud types are 2D + Index and data (including Z) is only in a separate higher-dimension data cloud
template <typename CloudType, typename Cloud2DType, typename PointFlat> 
void decimateToMinima(CloudType data, Cloud2DType input_flat, Cloud2DType output, int decimation_factor)
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
        search_tree.nearestKSearch(putative_minimum, decimation_factor, nearest_indices, nearest_dists);

        // Check whether putative minimum is a minimum within its neighborhood
        for(int neighbor_ind=0; neighbor_ind<nearest_indices.size(); neighbor_ind++)
        {
            // Skip self
            if(i == nearest_indices[neighbor_ind])
                continue;
            int source_data_index = putative_minimum.index;                                     // index of putative minimum within the source data cloud
            int target_data_index = input_flat->points[nearest_indices[neighbor_ind]].index;    // index of neighbor point within the source data cloud
            // If Source is higher than Target, and therefore NOT a minimum
            if(data->points[source_data_index].z > data->points[target_data_index].z)  
            {
                retained_after_decimation[i] = false;
                break;
            }
            //else // Target is higher than Source, and therefore NOT a minimum 
             //   retained_after_decimation[nearest_indices[neighbor_ind]] = false;
        }
        // Aggregate points if they're still a minimum after the above check 
        if(retained_after_decimation[i])
            output->points.push_back(putative_minimum);
    }
    output->width = 1;
    output->height = output->points.size();
}
// -- For cases where input and output cloud types are the same and both contain XYZ information
template <typename CloudType, typename PointType> 
void decimateToMinima(CloudType input, CloudType output, int decimation_factor)
{
    // Stores whether each input point is a minimum
    std::vector<bool> retained_after_decimation(input->points.size(), true);
    pcl::KdTreeFLANN<PointType> search_tree;
    search_tree.setInputCloud(input);
    for(std::size_t i=0; i<input->points.size(); i++)
    {
        // Disqualify points which we've already found to be higher than a neighbor
        if(!retained_after_decimation[i])
            continue;

        PointType putative_minimum = input->points[i];
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
    output->width = 1;
    output->height = output->points.size();
}


template <typename CloudType, typename PointType> 
void decimateToMaxima(CloudType input, CloudType output, int decimation_factor)
{
     // Stores whether each input point is a minimum
    std::vector<bool> retained_after_decimation(input->points.size(), true);
    pcl::KdTreeFLANN<PointType> search_tree;
    search_tree.setInputCloud(input);
    for(std::size_t i=0; i<input->points.size(); i++)
    {
        // Disqualify points which we've already found to be higher than a neighbor
        if(!retained_after_decimation[i])
            continue;

        PointType putative_minimum = input->points[i];
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
            if(input->points[i].z < input->points[nearest_indices[neighbor_ind]].z)  
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
    output->width = 1;
    output->height = output->points.size();
}

// Assuming both input and output clouds have information on 
template <typename CloudType> 
void filterToLastReturn(CloudType input, CloudType output)
{
    for(std::size_t i=0; i<input->points.size(); i++)
        if(input->points[i].returnnumber == input->points[i].numberofreturns)
            output->points.push_back(input->points[i]);
}
// Assuming cloud to be processed is a simpler type containing only 2D information and an index which points
//   to the more complex data stored in the initial input cloud
template <typename CloudType, typename Cloud2DType> 
void filterToLastReturn(CloudType data, Cloud2DType input, Cloud2DType output)
{
    for(std::size_t i=0; i<input->points.size(); i++)
        if(data->points[input->points[i].index].returnnumber == data->points[input->points[i].index].numberofreturns)
            output->points.push_back(input->points[i]);
}      


// Estimate Cloud Normals 
template <typename CloudType, typename CloudNormalsType, typename PointType, typename NormalsType> 
void estimateNormals(CloudType input, CloudNormalsType normals, bool radius_search, float search_parameter, bool enforce_vertical)
{
    pcl::NormalEstimation<PointType, NormalsType> norm_est;
    typedef typename pcl::search::KdTree<PointType> tree;
    typedef typename pcl::search::KdTree<PointType>::Ptr tree_ptr;
    tree_ptr search_tree(new tree);
    
    norm_est.setInputCloud(input);
    norm_est.setSearchMethod(search_tree);
    // Search can be either within a given radius, or using K neighbors
    if(radius_search)
        norm_est.setRadiusSearch(search_parameter);
    else
        norm_est.setKSearch(search_parameter);
    norm_est.compute(*normals);
    
    pcl::copyPointCloud3D<CloudType, CloudNormalsType>(input, normals);

    // If requested, flip all normals to ensure they point upwards
    if(enforce_vertical)
        for(std::size_t i=0; i<normals->points.size(); i++)
            if(normals->points[i].normal_z < 0)
            {
                normals->points[i].normal_x *= -1;
                normals->points[i].normal_y *= -1;
                normals->points[i].normal_z *= -1;
            }
}


// Estimate Cloud Roughness
//   For each point, an average of how much its normal direction and height differ from its neighbors 
//   Normal difference is assessed as average absolute value angle deviation between source and neighbor points
template <typename InputCloudType, typename OutputCloudType, typename InputPointType> 
void estimateRoughness(InputCloudType input, OutputCloudType output, bool radius_search, float search_parameter)
{
    pcl::copyPointCloud3D<InputCloudType, OutputCloudType>(input, output);
    pcl::copyPointCloudNormals<InputCloudType, OutputCloudType>(input, output);

    typedef typename pcl::KdTreeFLANN<InputPointType> KDTree;
    KDTree tree;
    tree.setInputCloud(input);
    for(std::size_t i=0; i<input->points.size(); i++)
    {
        std::vector<int> indices;
        std::vector<float> dists;
        if(radius_search)
            tree.radiusSearch(output->points[i], search_parameter, indices, dists);
        else
            tree.nearestKSearch(output->points[i], search_parameter, indices, dists);

        Eigen::Vector3f source_normal; 
        source_normal << input->points[i].normal_x, input->points[i].normal_y, input->points[i].normal_z;
        float source_normal_mag = source_normal.norm();

        float avg_angle_difference = 0;
        float height_difference = 0;
        for(int j=0; j<indices.size(); j++)
        {
            Eigen::Vector3f target_normal; 
            target_normal << input->points[indices[j]].normal_x, input->points[indices[j]].normal_y, input->points[indices[j]].normal_z;
            float target_normal_mag = target_normal.norm();
            

            float angle_difference = std::acos(source_normal.dot(target_normal) / source_normal_mag / target_normal_mag);
            avg_angle_difference += angle_difference;

            height_difference += output->points[i].z - input->points[indices[j]].z;
        }
        output->points[i].height_diff_avg = height_difference / indices.size();
        output->points[i].norm_diff_avg = avg_angle_difference / indices.size();
    }
}



template <typename CloudType>
Eigen::Vector3d deMeanCloud(CloudType cloud)
{
    Eigen::Vector3d mean_coords;
    mean_coords << 0.0, 0.0, 0.0;
    for(std::size_t i; i<cloud->points.size(); i++)
    {
        mean_coords[0] += (cloud->points[i].x / cloud->points.size());
        mean_coords[1] += (cloud->points[i].y / cloud->points.size());
        mean_coords[2] += (cloud->points[i].z / cloud->points.size());
    }
    for(std::size_t i; i<cloud->points.size(); i++)
    {
        cloud->points[i].x = float(cloud->points[i].x - mean_coords[0]);
        cloud->points[i].y = float(cloud->points[i].y - mean_coords[1]);
        cloud->points[i].z = float(cloud->points[i].z - mean_coords[2]);
    }  
    return mean_coords;
}


template <typename CloudType>
void reMeanCloud(CloudType cloud, Eigen::Vector3d mean_coords)
{
    for(std::size_t i; i<cloud->points.size(); i++)
    {
        cloud->points[i].x = float(cloud->points[i].x + mean_coords[0]);
        cloud->points[i].y = float(cloud->points[i].y + mean_coords[1]);
        cloud->points[i].z = float(cloud->points[i].z + mean_coords[2]);
    }  
}
template <typename CloudType>
void reMeanCloud(CloudType input, CloudType output, Eigen::Vector3d mean_coords)
{
    *output = *input;
    for(std::size_t i; i<input->points.size(); i++)
    {
        output->points[i].x = float(input->points[i].x + mean_coords[0]);
        output->points[i].y = float(input->points[i].y + mean_coords[1]);
        output->points[i].z = float(input->points[i].z + mean_coords[2]);
    }  
}


// Get point height as inverse-square-distance-weighted difference between point z-value and z-value of NUM_NEIGHBORS points
//   Distance for weighting is evaluated in 2D (XY)
//   Nearest neighbors are in the search space of KdTreeType
//   Generally, should choose a search tree type that is 2D as well
// Is there a safer way to handle inverse distance weighting to prevent overflow for very close (or coincident) points?
//   Maybe some kind of fixed Mexican Hat-style wavelet kernel? 
template <typename CloudType, typename SourcePointType, typename KdTreeType>
float relativePointHeight(CloudType cloud, SourcePointType point, KdTreeType tree, int num_neighbors)
{
   std::vector<int> indices;
   std::vector<float> dists;
   tree->nearestKSearch(point, num_neighbors, indices, dists);

   float total_weighted_height_diff = 0;
   float total_inverse_distance = 0;

    for(std::size_t i=0; i<indices.size(); i++)
    {
        Eigen::Vector2f xy_vector_between_points;
        xy_vector_between_points << (point.x - cloud->points[indices[i]].x),
                                    (point.y - cloud->points[indices[i]].y);
        float planar_distance = xy_vector_between_points.norm();
        float height_difference = point.z - cloud->points[indices[i]].z;

        total_weighted_height_diff += height_difference / pow(planar_distance,2);
        total_inverse_distance += 1/pow(planar_distance,2);
    }
    return total_weighted_height_diff / total_inverse_distance;
}


// Get point height as simple height difference above containing TIN triangle
//   Distance for weighting is evaluated in 2D (XY)
//   Nearest neighbors are in the search space of KdTreeType
//   Generally, should choose a search tree type that is 2D as well
// Is there a safer way to handle inverse distance weighting to prevent overflow for very close (or coincident) points?
//   Maybe some kind of fixed Mexican Hat-style wavelet kernel? 
template <typename CloudType, typename SourcePointType>
float relativePointHeight(CloudType cloud, SourcePointType point, las_triangulation::Delaunay &ground)
{
    return las_triangulation::pointHeight(cloud, point, ground);
}


}