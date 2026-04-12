# pointcloud-pipeline

Small C++ point cloud pipeline for class/research work. The project now has a
basic pipeline object that accepts `PointXYZ` vectors and returns the processed
cloud, which gives the next algorithms one API to plug into.
Downsampling now uses a simple voxel grid. It averages points in each occupied cell before later segmentation work.

Segmentation uses a straightforward breadth-first Euclidean clustering pass. It works for small test clouds, but the all-pairs search will need profiling.

A first benchmark target measures one synthetic 100k point cloud so I can see which part gets slow.
