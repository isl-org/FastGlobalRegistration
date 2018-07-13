# Fast Global Registration

## Introduction

This is an open source C++ implementation based on the technique presented in
the following paper ([download](docs/fast-global-registration.pdf)):

Fast Global Registration  
Qian-Yi Zhou, Jaesik Park, and Vladlen Koltun  
ECCV 2016  

The source code and dataset are published under the MIT license. See [LICENSE](LICENSE) for details. In general, you can use them for any purpose with proper attribution. If you do something interesting with the code, we'll be happy to know about it. Feel free to contact us.

We include two external libraries ([Eigen](https://eigen.tuxfamily.org/) and [flann](http://www.cs.ubc.ca/research/flann/)) in the codebase for easy compilation. Both of them are under a BSD-style license. See [source/External/README.txt](source/External/README.txt) for details.

Current version is 1.02 ([CHANGELOG](CHANGELOG)).

## Compilation

FastGlobalRegistration is compiled using [CMake](https://cmake.org/). All dependencies are included in the codebase.

### Ubuntu

The compilation has been tested on Ubuntu 16.04 (gcc 5.4) and Ubuntu 15.10 (gcc 4.9).

```
> mkdir build
> cd build
> cmake ../source/
> make
```

### OS X

The compilation has been tested with El Capitan (Clang or Xcode) and Sierra (Clang or Xcode). Follow the instructions in the Ubuntu section to compile from console with Clang. If you want to use Xcode.
```
> mkdir build-xcode
> cd build-xcode
> cmake -G Xcode ../source/
> open FastGlobalRegistration.xcodeproj/
```

### Windows

The compilation has been tested with Windows 8 and 10, Visual Studio 2013 and 2015. You can use the CMake GUI as follows. Click **Configure** and choose the correct Visual Studio version, then click **Generate**. Open the solution file with Visual Studio, change the build type to **Release**, then **rebuild** the **ALL_BUILD** target.

![docs/windows-cmake.png](docs/windows-cmake.png)

## Running FastGlobalRegistration

The FastGlobalRegistration program takes three parameters: a file storing the features of the target point cloud, a file storing the features of the source point cloud, and an output file that contains a transformation in [.log trajectory file format](http://redwood-data.org/indoor/fileformat.html).

We have provided a synthetic dataset in the [dataset](dataset) folder. For example, you can run the program from console.
```
> FastGlobalRegistration/FastGlobalRegistration \
../dataset/pairwise_noise_xyz_level_02_01_rot_05/features_0000.bin \
../dataset/pairwise_noise_xyz_level_02_01_rot_05/features_0001.bin \
../dataset/pairwise_noise_xyz_level_02_01_rot_05/output.txt
```

### Evaluation
To evaluate the output transformation, use [evaluation program](source/FastGlobalRegistration/evaluation.cpp).
```
> FastGlobalRegistration/Evaluation \
../dataset/pairwise_noise_xyz_level_02_01_rot_05/features_0000.bin \
../dataset/pairwise_noise_xyz_level_02_01_rot_05/features_0001.bin \
../dataset/pairwise_noise_xyz_level_01_01_rot_05/gt.log \
../dataset/pairwise_noise_xyz_level_01_01_rot_05/output.txt \
../dataset/pairwise_noise_xyz_level_01_01_rot_05/output_eval.txt
```
Evaluation program will write RMSE in output_eval.txt. The error in the Table 1 shows half of average RMSE.

### Creating input

The input files are binary files storing the features of the point clouds. Each file starts with a 4-byte integer indicating the number of points, denoted as N; followed by a 4-byte integer indicating the dimensionality of the feature vector, denoted as K. Then N points are stored sequentially with each point represented as (3+K) floats. The (3+K) floats are the x,y,z coordinates and a K-vector representing the feature vector associated with the point. In the data provided in the repository, we use FPFH feature with K=33.

If you are familiar with [PCL](www.pointclouds.org), the following code creates such a binary feature file of FPFH feature.
```cpp
// Assume a point cloud with normal is given as
// pcl::PointCloud<pcl::PointNormal>::Ptr object

pcl::FPFHEstimationOMP<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> fest;
pcl::PointCloud<pcl::FPFHSignature33>::Ptr object_features(new pcl::PointCloud<pcl::FPFHSignature33>());
fest.setRadiusSearch(feature_radius_);  
fest.setInputCloud(object);
fest.setInputNormals(object);
fest.compute(*object_features);

FILE* fid = fopen("features.bin", "wb");
int nV = object->size(), nDim = 33;
fwrite(&nV, sizeof(int), 1, fid);
fwrite(&nDim, sizeof(int), 1, fid);
for (int v = 0; v < nV; v++) {
    const pcl::PointNormal &pt = object->points[v];
    float xyz[3] = {pt.x, pt.y, pt.z};
    fwrite(xyz, sizeof(float), 3, fid);
    const pcl::FPFHSignature33 &feature = object_features->points[v];
    fwrite(feature.histogram, sizeof(float), 33, fid);
}
fclose(fid);
```

### Tuning parameters

The relevant parameters are defined in [app.h](source/FastGlobalRegistration/app.h) as macros.
```cpp
#define DIV_FACTOR			1.4		// Division factor used for graduated non-convexity
#define USE_ABSOLUTE_SCALE	0		// Measure distance in absolute scale (1) or in scale relative to the diameter of the model (0)
#define MAX_CORR_DIST		0.025	// Maximum correspondence distance (also see comment of USE_ABSOLUTE_SCALE)
#define ITERATION_NUMBER	64		// Maximum number of iteration
#define TUPLE_SCALE			0.95	// Similarity measure used for tuples of feature points.
#define TUPLE_MAX_CNT		1000	// Maximum tuple numbers.
```

We measure distance relative to the diameter_of_model if **USE_ABSOLUTE_SCALE** is set to 0. It is our default setting for synthetic data. For real world data which you know the absolute scale, change **USE_ABSOLUTE_SCALE** to 1 and define **MAX_CORR_DIST** accordingly.

**MAX_CORR_DIST** determines when the optimization will stop. In general, **MAX_CORR_DIST** (USE_ABSOLUTE_SCALE=1) or **MAX_CORR_DIST * diameter_of_model** (USE_ABSOLUTE_SCALE=0) should be set close to the threshold used to determine if a point pair is a match in global space. If you don't know how to set it, start with the default value **0.025**. Decreasing this parameter sometimes results in tighter alignment.

**TUPLE_MAX_CNT** trades off between speed and accuracy. Increasing it will make the optimization slower but the result can be more accurate.

### Matlab binding

FastGlobalRegistration has a Matlab binding courtesy of Jordi Pont-Tuset. It can be used seamlessly with the compilation tool chains mentioned above. Follow instructions provided by CMake if you need to make additional configuration for MATLAB_ROOT environment variable. To use the Matlab binding, execute [fgr_demo.m](source/Matlab/fgr_demo.m) from source/Matlab.

### Using Open3D

This repository is maintained for providing standard alone FastGlobalRegistration application reproducing the results in the published paper. Open3D has end-to-end implementation used for FastGlobalRegistration - including point cloud I/O, feature extraction, and FastGlobalRegistration module in one place. For more details, please follow [this tutorial](http://open3d.org/docs/tutorial/Advanced/fast_global_registration.html).

### Troubleshooting
If you encounter issues with FGR, please check [troubleshooting.md](troubleshooting.md)
