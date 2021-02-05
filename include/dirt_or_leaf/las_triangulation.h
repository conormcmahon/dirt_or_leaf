
#ifndef LAS_TRIANGULATION_
#define LAS_TRIANGULATION_

#include <pcl/io/ply_io.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_hierarchy_2.h>

namespace las_triangulation
{

    typedef CGAL::Exact_predicates_inexact_constructions_kernel             K;
    typedef CGAL::Triangulation_vertex_base_with_info_2<unsigned int, K>    Vbb;
    typedef CGAL::Triangulation_hierarchy_vertex_base_2<Vbb>    Vb;
    typedef CGAL::Triangulation_data_structure_2<Vbb>                        Tds;
    typedef CGAL::Delaunay_triangulation_2<K, Tds>                          Delaunay;
    typedef CGAL::Triangulation_hierarchy_2<Delaunay>                       Delaunay_hierarchy;
    typedef Delaunay_hierarchy::Point_2                                     CGALPoint;

    typedef Delaunay::Face_handle Face_handle;

    // Create a 2.5D TIN from input PCL PointCloud Ptr
    template <typename CloudType> 
    void delaunayTriangulation(CloudType input_cloud, Delaunay& triangulation);

    template <typename CloudType>
    void outputPly(CloudType cloud, Delaunay& triangulation, std::string filename);

    // Get point height above a given TIN surface
    template <typename CloudType, typename PointType>
    float interpolateTIN(CloudType cloud, PointType point, Delaunay& triangulation, bool use_starting_face=true, Face_handle starting_face=Face_handle(), float nodata_value=-9999);
    // Get slope and aspect at a given point on TIN 
    template <typename CloudType, typename PointType>
    Eigen::Vector2f slopeAtPoint(CloudType cloud, PointType point, Delaunay& triangulation, bool use_starting_face=true, Face_handle starting_face=Face_handle(), float nodata_value=-9999);

}
 

#endif // LAS_TRIANGULATION_