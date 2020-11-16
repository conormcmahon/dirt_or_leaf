

template <typename LASType, typename VegType, typename GroundType> 
void LASClassifier<LASType, VegType, GroundType>::void extractBuildings(int normals_neighbors, float roughness_neighbors, float dist_thresh, float smoothness_thresh)
{
    // Get normals of locally decimated maxima
    pcl::PointCloud<pcl::PointXYZI>::Ptr tops_decimated_3d(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::copyPointCloud3D(tops_decimated_, tops_decimated_3d);
    las_filtering::estimateNormals<pcl::PointCloud<pcl::PointXYZI>::Ptr, GCP, pcl::PointXYZI, GroundType>(tops_decimated_3d, tops_decimated_, true, normals_neighbors, true);
    //pcl::copyPointCloudNormals(tops_decimated_3d, tops_decimated_);
    // Find points whose normals are similar to neighbors
    GCP roughness(new GC);
    las_filtering::estimateRoughness<GCP, GCP, GroundType>(tops_decimated_, roughness, false, roughness_neighbors);
    if(save_outputs_)
        outputPCD<GCP, GroundType>(roughness, output_directory_ + scene_name_ + std::string("_maxima_roughness.pcd"), true);

    // Run a smoothness filter before feeding to region growing
    pcl::PointCloud<pcl::PointXYZI>::Ptr xyz_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    GCP roughness_filtered(new GC);
    for(std::size_t i=0; i<roughness->points.size(); i++)
        if(true)//fabs(roughness->points[i].norm_diff_avg) < 0.2)
        {
            xyz_filtered->points.push_back(tops_decimated_3d->points[i]);
            roughness_filtered->points.push_back(roughness->points[i]);
        }

    std::cout << "Running region growing with " << dist_thresh << " distance threshold, " << smoothness_thresh << " smoothness threshold." << std::endl;
    pcl::RegionGrowingSmoothDistance<pcl::PointXYZI, pcl::Point2DGround> reg;

    reg.setMinClusterSize(30);
    reg.setMaxClusterSize(20000);
    pcl::search::Search<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);    
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(6);
    reg.setInputCloud(xyz_filtered);
    reg.setInputNormals(roughness_filtered);
    reg.setSmoothnessThreshold(smoothness_thresh);
    reg.setDistanceThreshold(dist_thresh);
    //reg.setDistanceThreshold(0.25);
//    reg.setResidualTestFlag(true);
    reg.setCurvatureThreshold(1.0);

    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);
    std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
    std::cout << "First cluster has " << clusters[0].indices.size () << " points." << std::endl;

    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
    if(save_outputs_)
        outputPCD<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointXYZRGB>(colored_cloud, output_directory_ + scene_name_ + std::string("_segments.pcd"), true);

    std::cout << roughness_filtered->points.size() << " " << input_las_flattened_->points.size();

    GCP buildingness_cloud(new GC);
    GCP super(new GC);
    std::vector<int> nearest_indices;
    std::vector<float> nearest_dists;
    for(int i=0; i<clusters.size(); i++)
    {
        int num_low = 0;
        int num_high = 0; 

     //  std::cout << "init " <<  i << std::endl;
     //  std::cin.get();

        // iterate over points in cluster
        Eigen::Vector2f cluster_centroid;
        cluster_centroid << 0.0, 0.0;
        for(int j=0; j<clusters[i].indices.size(); j++)
        {
            GroundType point;
            point.x = xyz_filtered->points[clusters[i].indices[j]].x;
            point.y = xyz_filtered->points[clusters[i].indices[j]].y;
            point.z = xyz_filtered->points[clusters[i].indices[j]].z;
            Eigen::Vector2f point_eig;
            point_eig << point.x, point.y;

            cluster_centroid[0] += point.x;
            cluster_centroid[1] += point.y;

           // std::cout << "in here " << j << " " << clusters[i].indices[j] << " " << point.x << " " << point.y << " " << point.z << std::endl;
           // std::cin.get();

            point.height_diff_avg = 0;

            input_tree_->radiusSearch(point, 2.0, nearest_indices, nearest_dists);

            int local_num_low = 0;
            int local_num_high = 0;
//
            //std::cout << nearest_indices.size() << std::endl;
            //if(nearest_indices.size() > 0)
           //     std::cout << nearest_indices[0] << std::endl;
            //std::cin.get();
            for(int k=0; k<nearest_indices.size(); k++)
            {
                if(fabs(roughness_filtered->points[clusters[i].indices[j]].height_diff_avg) < 3) // don't count edge points
                {
                    float height_diff = point.z - input_las_flattened_->points[nearest_indices[k]].z;

                    //std::cout << "diffs " << k << " " << height_diff << std::endl;
                    //std::cin.get();

                    if(height_diff > 5) // if neighbor is a lot lower
                    {
                        local_num_low++;
                        point.height_diff_avg += height_diff;
                    }
                    else 
                        local_num_high++;
                }
            }
            point.height_diff_avg /= nearest_indices.size();
            point.norm_diff_avg = float(local_num_low)/float(local_num_high+local_num_low);
            num_low += local_num_low;
            num_high += local_num_high;

            buildingness_cloud->points.push_back(point);
            super->points.push_back(point);
        }

        cluster_centroid /= clusters[i].indices.size();

        // Iterate over points in cluster again...
        std::vector<float> distances_to_centroid;
        int cluster_start = buildingness_cloud->points.size() - clusters[i].indices.size();
        for(int j=0; j<clusters[i].indices.size(); j++)
        {
            Eigen::Vector2f point_eig;
            point_eig << buildingness_cloud->points[cluster_start+j].x, buildingness_cloud->points[cluster_start+j].y;
            distances_to_centroid.push_back((point_eig - cluster_centroid).norm());
            buildingness_cloud->points[cluster_start+j].intensity = distances_to_centroid[j] / sqrt(clusters[i].indices.size());
        }
        std::sort(distances_to_centroid.begin(), distances_to_centroid.end());
        float high_dist = distances_to_centroid[ceil(distances_to_centroid.size()*19.0/20.0)-1];    // 95th percentile distance 
        float low_dist = distances_to_centroid[ceil(distances_to_centroid.size()*1.0/2.0)-1];       // 25th percentile distance
        // If cluster is very small, force high_dist to include at least 3 points
        if(clusters[i].indices.size() < 60)
            high_dist = distances_to_centroid[distances_to_centroid.size()-4];

        float edge_high = 0;
        float edge_low = 0;
        float center_high = 0;
        float center_low = 0;
        for(int j=0; j<clusters[i].indices.size(); j++)
        {
            // If a point has no neighbors that are much lower than it...
            if(distances_to_centroid[j] >= high_dist)
                if(buildingness_cloud->points[cluster_start+j].norm_diff_avg == 0)
                    edge_high++;
                else
                    edge_low++;
            else if(distances_to_centroid[j] <= low_dist)
                if(buildingness_cloud->points[cluster_start+j].norm_diff_avg == 0)
                    center_high++;
                else 
                    center_low++;
        }

        for(int j=0; j<clusters[i].indices.size(); j++)
        {
            super->points[cluster_start+j].norm_diff_avg = center_low/(center_low+center_high);
            super->points[cluster_start+j].height_diff_avg = edge_low/(edge_low+edge_high);
            super->points[cluster_start+j].intensity = super->points[cluster_start+j].norm_diff_avg * (1-super->points[cluster_start+j].height_diff_avg);
        }

    }
    if(save_outputs_)
    {
        outputPCD<GCP, GroundType>(buildingness_cloud, output_directory_ + scene_name_ + std::string("_buildings.pcd"), true);
        outputPCD<GCP, GroundType>(super, output_directory_ + scene_name_ + std::string("_super.pcd"), true);
    }

}