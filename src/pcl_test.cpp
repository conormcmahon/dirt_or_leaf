
#include "dirt_or_leaf/pcl_test.h"

// Declare point cloud types (for simplicity)
typedef typename pcl::PointCloud<pcl::PointLAS>           LC;
typedef typename pcl::PointCloud<pcl::PointLAS>::Ptr      LCP;
typedef typename pcl::PointCloud<pcl::PointVeg>           VC;
typedef typename pcl::PointCloud<pcl::PointVeg>::Ptr      VCP;
typedef typename pcl::PointCloud<pcl::PointXYZI>          PC;      
typedef typename pcl::PointCloud<pcl::PointXYZI>::Ptr     PCP;
typedef typename pcl::PointCloud<pcl::Point2DGround>      GC;
typedef typename pcl::PointCloud<pcl::Point2DGround>::Ptr GCP;

int main(int argc, char *argv[])
{
  std::cout << std::endl;
  std::cout << "Beginning LiDAR classification routine." << std::endl;

  if ( argc != 6 ) // argc should be 2 for correct execution
  {
    // We print argv[0] assuming it is the target file
    std::cout << "Didn't receive expected number of arguments. Usage: data/" << argv[0] << ".pdc <filename>\n";
    return -1;
  }

  std::string filename = argv[1];
  int decimation_factor = std::atoi(argv[2]);
  double minima_radius = std::atof(argv[3]);
  int normal_neighbors = std::atoi(argv[4]);
  int roughness_neighbors = std::atoi(argv[5]);

  // Perform Classification 
  LASClassifier<pcl::PointLAS , pcl::PointVeg, pcl::Point2DGround> classifier(true);
  classifier.loadLASPCD(filename + std::string(".pcd"));
  classifier.setOutputOptions(true, "/home/conor/lidar_data/", filename);
  classifier.decimateToMinima(decimation_factor, true);
  classifier.curvatureAnalysis(normal_neighbors, roughness_neighbors);
  
  /*

  LCP las(new LC);
  VCP veg(new VC);
  SCP index(new SC);
  PCP xyzi(new PC);

  //typename las_filtering::decimateToMinima<LCP, SCP, pcl::Point2DIndex>;
  //las_filtering::decimateToMinima<LCP, SCP, pcl::Point2DIndex>(las, index, index, 10);
*/
  std::cout << std::endl;
  return (0);
}

