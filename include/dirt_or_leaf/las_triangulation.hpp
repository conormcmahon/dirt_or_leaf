
#include <dirt_or_leaf/las_triangulation.h>


namespace las_triangulation
{

    // Creates a 2.5D Delaunay Triangulation for an input PCL PointCloud Ptr
    //   Assumes input cloud has already been sampled to a simple 2.5D format for which making a Delaunay triangulation makes sense
    //   Generally used to create surface models for ground, top of canopy, etc. 
    //   Triangles are computed as Delaunay solution for 2D cloud, then reassigned height values. Height is NOT used in triangulation 
    template <typename CloudType> 
    void delaunayTriangulation(CloudType input_cloud, Delaunay& triangulation)
    { 
        std::cout << "Performing Delaunay triangulation." << std::endl;
        // Convert ground minima cloud to CGAL vector of points
        std::vector< std::pair<CGALPoint, unsigned> > minima_vec;
        for(std::size_t i=0; i<input_cloud->points.size(); i++)
        {
            minima_vec.push_back(std::make_pair(CGALPoint(input_cloud->points[i].x,input_cloud->points[i].y), i));
        }
        // Generate Delaunay Triangulation for ground minima 
        triangulation = Delaunay(minima_vec.begin(), minima_vec.end());
        std::cout << "  Number of vertices in Delaunay: " << triangulation.number_of_vertices() << std::endl;
        std::cout << "  Number of faces in Delaunay: " << triangulation.number_of_faces() << std::endl; 
    }


    // Returns the point height above a given TIN
    template <typename CloudType, typename PointType>
    float interpolateTIN(CloudType cloud, PointType point, Delaunay& triangulation, bool use_starting_face, Face_handle starting_face)
    {
        CGALPoint point_cgal(point.x, point.y);

        // Get containing triangle on Delauney Triangulation for source point - vertex indices in ground cloud
        Face_handle face;
        if(use_starting_face)
            face = triangulation.locate(point_cgal, starting_face);
        else
            face = triangulation.locate(point_cgal);
        unsigned ind_1 = face->vertex(0)->info();
        unsigned ind_2 = face->vertex(1)->info();
        unsigned ind_3 = face->vertex(2)->info();
       // Skip points which are outside the initial minima convex hull 
        if(std::max(std::max(ind_1,ind_2),ind_3) > cloud->points.size()-1)
            return -1000; 
        
        // Set up Eigen structs
        //   Vertices of Triangle
        Eigen::Vector3f vert_one, vert_two, vert_three, point_eig;
        vert_one << cloud->points[ind_1].x, cloud->points[ind_1].y, cloud->points[ind_1].z;
        vert_two << cloud->points[ind_2].x, cloud->points[ind_2].y, cloud->points[ind_2].z;
        vert_three << cloud->points[ind_3].x, cloud->points[ind_3].y, cloud->points[ind_3].z;
        point_eig << point.x, point.y, 0;
        //   Edges of Triangle
        Eigen::Vector3f side_one, side_two;
        side_one = vert_two - vert_one;
        side_two = vert_three - vert_one;

        // Plane normal for triangle
        Eigen::Vector3f plane_normal;
        plane_normal = side_one.cross(side_two);
        plane_normal /= plane_normal.norm();
        // Force normal to point downwards
        if(plane_normal[2] > 0)
        {
            plane_normal *= -1;
        }

        // Distance from Point to Plane (along vertical axis)
        Eigen::Vector3f vertical_axis;
        vertical_axis << 0, 0, 1;
        return (vert_one - point_eig).dot(plane_normal) / plane_normal.dot(vertical_axis);
    }



    // Create an output .ply triangulation file
    //   REQUIRES that the indices of points within the TRIANGULATION object match those in CLOUD (run delaunayTriangulation() to meet this)
    template <typename CloudType>
    void outputPly(CloudType cloud, Delaunay& triangulation, std::string filename)
    {
        // Convert CGAL type to output .ply file
        pcl::PolygonMesh::Ptr mesh_pcl(new pcl::PolygonMesh());
        //   Copy vertex information
        pcl::toPCLPointCloud2(*cloud, mesh_pcl->cloud);
        for (Delaunay::Finite_faces_iterator fit = triangulation.finite_faces_begin();
            fit != triangulation.finite_faces_end(); ++fit)
        {
            // Save vertices of each face into PCL structure
            pcl::Vertices face_vertices;
            face_vertices.vertices.push_back(uint32_t(fit->vertex(0)->info()));
            face_vertices.vertices.push_back(uint32_t(fit->vertex(1)->info()));
            face_vertices.vertices.push_back(uint32_t(fit->vertex(2)->info()));
            mesh_pcl->polygons.push_back(face_vertices);

            //std::cout << "\nIndices:        " << uint32_t(fit->vertex(0)->info()) << " " << uint32_t(fit->vertex(1)->info()) << " " << uint32_t(fit->vertex(2)->info());
            //std::cout << "\nCGAL Locations: " << triangulation.triangle(fit);
            //std::cout << "\nPCL Locations:  " << ground_minima->points[fit->vertex(0)->info()].x << " " << ground_minima->points[fit->vertex(0)->info()].y << " " << ground_minima->points[fit->vertex(0)->info()].scale;
            //std::cout << "\nPCL Locations:  " << ground_xyz->points[fit->vertex(0)->info()].x << " " << ground_xyz->points[fit->vertex(0)->info()].y << " " << ground_xyz->points[fit->vertex(0)->info()].z;
        }

        // last parameter is write precision 
        // could instead opt to use pcl::io::savePlyFileBinary(std::string path, const pcl::PolygonMes &mesh) for binary files
        pcl::io::savePLYFile(filename, *mesh_pcl, 8);
    }
}