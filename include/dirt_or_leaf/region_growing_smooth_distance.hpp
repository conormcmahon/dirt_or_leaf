

#include <dirt_or_leaf/region_growing_smooth_distance.h>

template <typename PointT, typename NormalT> 
pcl::RegionGrowingSmoothDistance<PointT, NormalT>::RegionGrowingSmoothDistance()
: distance_threshold_(1)
{
    
}

template <typename PointT, typename NormalT> 
void pcl::RegionGrowingSmoothDistance<PointT, NormalT>::setDistanceThreshold (float distance)
{
    distance_threshold_ = distance;
}
/*
template <typename PointT, typename NormalT> 
int pcl::RegionGrowingSmoothDistance<PointT, NormalT>::growRegion (int initial_seed, int segment_number)
{
    1+1;
  std::queue<int> seeds;
  seeds.push (initial_seed);
  this->point_labels_[initial_seed] = segment_number;

  int num_pts_in_segment = 1;

  while (!seeds.empty ())
  {
    int curr_seed;
    curr_seed = seeds.front ();
    seeds.pop ();

    std::size_t i_nghbr = 0;
    while ( i_nghbr < this->neighbour_number_ && i_nghbr < this->point_neighbours_[curr_seed].size () )
    {
      int index = this->point_neighbours_[curr_seed][i_nghbr];
      if (this->point_labels_[index] != -1)
      {
        i_nghbr++;
        continue;
      }

      bool is_a_seed = false;
      bool belongs_to_segment = this->validatePoint (initial_seed, curr_seed, index, is_a_seed);

      if (!belongs_to_segment)
      {
        i_nghbr++;
        continue;
      }

      this->point_labels_[index] = segment_number;
      num_pts_in_segment++;

      if (is_a_seed)
      {
        seeds.push (index);
      }

      i_nghbr++;
    }// next neighbour
  }// next seed

  return (num_pts_in_segment);
}

*/

template <typename PointT, typename NormalT> bool
pcl::RegionGrowingSmoothDistance<PointT, NormalT>::validatePoint (int initial_seed, int point, int nghbr, bool& is_a_seed) const
{
  is_a_seed = true;

  float cosine_threshold = std::cos (this->theta_threshold_);
  float data[4];

  data[0] = (*this->input_)[point].data[0];
  data[1] = (*this->input_)[point].data[1];
  data[2] = (*this->input_)[point].data[2];
  data[3] = (*this->input_)[point].data[3];
  Eigen::Map<Eigen::Vector3f> initial_point (static_cast<float*> (data));
  Eigen::Map<Eigen::Vector3f> initial_normal (static_cast<float*> ((*this->normals_)[point].normal));

  //check the angle between normals
  if (this->smooth_mode_flag_ == true)
  {
    Eigen::Map<Eigen::Vector3f> nghbr_normal (static_cast<float*> ((*this->normals_)[nghbr].normal));
    float dot_product = std::abs (nghbr_normal.dot (initial_normal));
    if (dot_product < cosine_threshold)
    {
      return (false);
    }
  }
  else
  {
    Eigen::Map<Eigen::Vector3f> nghbr_normal (static_cast<float*> ((*this->normals_)[nghbr].normal));
    Eigen::Map<Eigen::Vector3f> initial_seed_normal (static_cast<float*> ((*this->normals_)[initial_seed].normal));
    float dot_product = std::abs (nghbr_normal.dot (initial_seed_normal));
    if (dot_product < cosine_threshold)
      return (false);
  }

  // Check the distance between points
  Eigen::Vector3f point_loc, nghbr_loc;
  point_loc << (*this->input_)[point].x, (*this->input_)[point].y, (*this->input_)[point].z;
  nghbr_loc << (*this->input_)[nghbr].x, (*this->input_)[nghbr].y, (*this->input_)[nghbr].z;
  if(fabs((point_loc-nghbr_loc).norm()) > this->distance_threshold_)
  {
      is_a_seed = false;
  //std::cout << point << " " << nghbr << std::endl;
  //std::cout << " got here... " << point_loc[0] << " " << point_loc[1] << " " << point_loc[2] << " || " <<  nghbr_loc[0] << " " << point_loc[1] << " " << nghbr_loc[2] << std::endl;
  //float dist = sqrt(pow(point_loc[0]*nghbr_loc[0],2) + pow(point_loc[1]*nghbr_loc[1],2) + pow(point_loc[2]*nghbr_loc[2],2));
  //std::cout << fabs((point_loc-nghbr_loc).norm()) << " " << dist << " " << fabs(point_loc[2]-nghbr_loc[2]) << " " << is_a_seed*1 << std::endl;
  }  


  // check the curvature if needed
  if (this->curvature_flag_ && (*this->normals_)[nghbr].curvature > this->curvature_threshold_)
  {
    is_a_seed = false;
  }

  // check the residual if needed
  float data_1[4];
  
  data_1[0] = (*this->input_)[nghbr].data[0];
  data_1[1] = (*this->input_)[nghbr].data[1];
  data_1[2] = (*this->input_)[nghbr].data[2];
  data_1[3] = (*this->input_)[nghbr].data[3];
  Eigen::Map<Eigen::Vector3f> nghbr_point (static_cast<float*> (data_1));
  float residual = std::abs (initial_normal.dot (initial_point - nghbr_point));
  if (this->residual_flag_ && residual > this->residual_threshold_)
    is_a_seed = false;

  return (true);
}