
#include <dirt_or_leaf/ground_tin.h>

template <typename GroundType> 
GroundTIN<GroundType>::GroundTIN()
{
    cloud_.reset(new GC());
    tree_.reset(new Tree2D());
}

template <typename GroundType> 
void GroundTIN<GroundType>::setInputCloud(GCP cloud)
{
    *cloud_ = *cloud;
    tree_->setInputCloud(cloud_);
    face_mapping_ = std::vector<las_triangulation::Delaunay::Face_handle>(cloud->points.size());
}
template <typename GroundType> 
void GroundTIN<GroundType>::setInputCloud(GCP cloud, Tree2DP tree)
{
    *cloud_ = *cloud;
    *tree_ = *tree;
    face_mapping_ = std::vector<las_triangulation::Delaunay::Face_handle>(cloud->points.size());
}

template <typename GroundType> 
void GroundTIN<GroundType>::generateTIN()
{
    // Generate Triangulation
    las_triangulation::delaunayTriangulation(cloud_, TIN_);
    
    // Generate mapping between TIN faces and input cloud point indices
    std::vector<bool> faces_filled(cloud_->points.size(), false);
    // Iterate over all faces in triangulation
    for (las_triangulation::Face_handle face : TIN_.finite_face_handles())
        // Iterate over 3 vertices for each face
        for(int i=0; i<3; i++)
        {
            int index = uint32_t(face->vertex(i)->info());
            if(!faces_filled[index])
                face_mapping_[index] = face;
        }
}

template <typename GroundType> 
void GroundTIN<GroundType>::saveTIN(std::string filename, bool remean, Eigen::Vector3d offset)
{
    // If cloud was centered around origin, re-apply offsets before printing
    GCP offset_cloud(new GC);
    las_filtering::reMeanCloud(cloud_, offset_cloud, offset);
    // Print .PLY triangulation
    las_triangulation::outputPly(offset_cloud, TIN_, filename);
}

template <typename GroundType> 
template <typename CloudType, typename PointType>
float GroundTIN<GroundType>::getPointHeight(CloudType source_cloud, PointType point)
{
    // Get nearest vertex within TIN to source point
    pcl::Point2DGround point_2d;
    point_2d.x = point.x;
    point_2d.y = point.y;
    point_2d.z = point.z;
    std::vector<int> nearest_indices;
    std::vector<float> nearest_dists;
    tree_->nearestKSearch(point_2d, 1, nearest_indices, nearest_dists);

    // Get point height
    return las_triangulation::pointHeight<CloudType, PointType>(source_cloud, point, TIN_, true, face_mapping_[nearest_indices[0]]);
}