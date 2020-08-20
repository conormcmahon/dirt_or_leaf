
#include "dirt_or_leaf/pcl_test.h"

// Declare point cloud types (for simplicity)
typedef typename pcl::PointCloud<pcl::PointLAS>           LC;
typedef typename pcl::PointCloud<pcl::PointLAS>::Ptr      LCP;
typedef typename pcl::PointCloud<pcl::PointVeg>           VC;
typedef typename pcl::PointCloud<pcl::PointVeg>::Ptr      VCP;
typedef typename pcl::PointCloud<pcl::PointXYZI>          PC;      
typedef typename pcl::PointCloud<pcl::PointXYZI>::Ptr     PCP;
typedef typename pcl::PointCloud<pcl::Point2DIndex>       SC;
typedef typename pcl::PointCloud<pcl::Point2DIndex>::Ptr  SCP;

int main(int argc, char *argv[])
{
  if ( argc != 4 ) // argc should be 2 for correct execution
  {
    // We print argv[0] assuming it is the target file
    std::cout << "Didn't receive expected number of arguments. Usage: data/" << argv[0] << ".pdc <filename>\n";
    return -1;
  }
  std::cout << argc << " " << argv;
  
  for(int i=0; i<10000000; i++);

  std::string filename = argv[1];
  int decimation_factor = std::atoi(argv[2]);
  float minima_radius = std::atof(argv[3]);

  std::cout << filename << " " << decimation_factor << " " << minima_radius;

  for(int i=0; i<10000000; i++);

  // Perform Classification 
  LASClassifier< pcl::PointLAS , pcl::PointVeg > classifier;
  classifier.loadLASPCD(filename);

  pcl::PointCloud<pcl::PointXYZI>::Ptr xyzi(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointLAS>::Ptr las(new pcl::PointCloud<pcl::PointLAS>);
  //pcl::copyPointCloud3D(xyzi, las);

  std::cout << std::endl;
  return (0);
}

