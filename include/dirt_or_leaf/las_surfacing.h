
#ifndef LAS_SURFACING_
#define LAS_SURFACING_

// PCL Surfacing
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>

// CGAL Delaunay Triangulation
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/Delaunay_triangulation_2.h>

#include <dirt_or_leaf/las_filtering.h>

#endif // LAS_SURFACING_