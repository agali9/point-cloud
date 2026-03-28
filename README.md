# pointcloud-pipeline

Small C++ point cloud pipeline for class/research work. The project now has a
basic pipeline object that accepts `PointXYZ` vectors and returns the processed
cloud, which gives the next algorithms one API to plug into.
Downsampling now uses a simple voxel grid. It averages points in each occupied cell before later segmentation work.
