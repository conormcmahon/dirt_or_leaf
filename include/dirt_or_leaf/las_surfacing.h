
#ifndef LAS_SURFACING_
#define LAS_SURFACING_

// Dirt_Or_Leaf includes
#include <dirt_or_leaf/las_filtering.h>

// PCL Surfacing
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>

// CGAL Delaunay Triangulation
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/Delaunay_triangulation_2.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel             K;
typedef CGAL::Triangulation_vertex_base_with_info_2<unsigned int, K>    Vb;
typedef CGAL::Triangulation_data_structure_2<Vb>                        Tds;
typedef CGAL::Delaunay_triangulation_2<K, Tds>                          Delaunay;
typedef K::Point_2                                                      CGALPoint;


#endif // LAS_SURFACING_