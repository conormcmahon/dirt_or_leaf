# dirt_or_leaf

![Example Image](/images/dirt_and_leaf.png)

Open library for segmenting plant and ground returns from aerial LiDAR point clouds.

## Processing Overview

- Custom point types are created for LAS points, ground points, and vegetation points. 
- Ground surfaces are extracted based on decimation --> filtering based on normals, height --> TIN construction
- Vegetation is extracted based on simple height filters (points are only kept if they are at least h height higher than the local ground surface)

## Ground Preprocessing

Before segmentation is conducted, the input cloud is preprocessed. This includes the following (optional) steps:
- Offset - cloud can be offset to have its origin on (0,0) to prevent overflow errors with very large coordinates. This can be undone before the output clouds are printed.
- Decimation - cloud can be decimated by a user-specified parameter, p. Each point in the cloud is considered and is only retained if it is lower than its nearest p neighbors. 
- Return Filtering - the process can be set to keep only the last returns (we know that for cases with multiple returns, only the last return can be a ground point). 
- Normals extraction - run using Eigen analysis in PCL (I think, been a while since I've actually looked at their code)

### Ground Segmentation

Ground points are segmented by comparing their local normals and height to those of their neighbors. A user parameter n is specified which determines the number of neighbors to consider. 

For each point in the putative ground cloud generated from filtering (above), the average distance of that point ABOVE its n neighbors is computed, and so is the average magnitude of the angle between this point and its neighbor points' normals. 

This is based in the idea that ground points should generally be lower than their neighbors, or at least not a lot higher than their neighbors, and that their normal angle should be similar to their neighbors (whereas there is more variation in normals for vegetation returns). 

Points are only retained as ground if both the resulting average values are below a user-specified threshold. In our usage, values of 0.5 m and 0.5 radians seem to work well. 

### Ground Upsampling

The above filtering methods may over-aggressively remove points which are actually part of the ground surface, especially in areas where there are large, even slopes (sides of hills). Planning to add some extra functions to go back and add in ground points which are actually good fits to the ground surface and should not have been removed. May do this based on distance to the generated TIN. 

### Ground TIN Generation

Once ground points are extracted above, they are used to build a Triangulated Irregular Network (TIN). This is done using the 2D triangulation functions in CGAL. Triangulations are created using only the 2D information from the cloud, and then regenerated into a 3D surface by applying the height of each point to the vertices of the triangle. Outputs are saved in .ply format using PCL functions. 

### Vegetation Extraction

Currently, this is a simple analysis based on extracting all points which are higher than a user-specified threshold ABOVE the local ground surface. 

Plans to make more complex functions, possibly utilizing information from intensity, return number, and surface geometry.

Also want to add some functions to generate a plant surface TIN once the vegetation clouds are generated! This should be relatively simple once we have a good vegetation surface extracted. 
