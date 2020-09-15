
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

  if ( argc != 9 ) // argc should be 2 for correct execution
  {
    // We print argv[0] assuming it is the target file
    std::cout << "Didn't receive expected number of arguments. Usage: data/" << argv[0] << ".pdc <filename>\n";
    return -1;
  }

  std::string directory = argv[1];
  std::string filename = argv[2];
  int decimation_factor = std::atoi(argv[3]);
  double minima_radius = std::atof(argv[4]);
  float normals_radius = std::atoi(argv[5]);
  int roughness_neighbors = std::atoi(argv[6]);
  float min_veg_height = std::atoi(argv[7]);
  float decimation_factor_veg = std::atoi(argv[8]);

  Timer total_timer("total time");
  // Perform Classification 
  LASClassifier<pcl::PointLAS , pcl::PointVeg, pcl::Point2DGround> classifier;
  classifier.loadLASPCD(directory + filename + std::string(".pcd"));
  classifier.setOutputOptions(true, directory + "output/", filename, true, true);
  classifier.decimateToMinima(decimation_factor, true);
  classifier.curvatureAnalysis(normals_radius, roughness_neighbors);
  classifier.buildGroundTIN();
  classifier.extractVegetationTIN(min_veg_height);
  classifier.decimateVegetation(decimation_factor_veg, true);

  total_timer.stop();

  std::cout << std::endl;
  return (0);
}

