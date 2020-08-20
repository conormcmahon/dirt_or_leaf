
#ifndef PCL_TEST_
#define PCL_TEST_
#define PCL_NO_PRECOMPILE

#include <iostream>
#include "dirt_or_leaf/las_classifier.hpp"

typedef CGAL::Exact_predicates_inexact_constructions_kernel     K;
typedef CGAL::Triangulation_vertex_base_with_info_2<unsigned int, K> Vb;
typedef CGAL::Triangulation_data_structure_2<Vb>                        Tds;
typedef CGAL::Delaunay_triangulation_2<K, Tds>                     Delaunay;
typedef K::Point_2                                               CGALPoint;

#endif // PCL_TEST_