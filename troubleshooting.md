# Troubleshooting

Please read this document before asking questions to us.

## Why FGR is not very accurate for my own dataset?

- FGR utilizes 3D features, and the accuracy depends how the feature parameters are properly set. We recommend to use 1:2:5 ratio for voxel size of point cloud downsampling, surface normal estimation, and FPFH features. You need to tune radius while keeping ratio. Before use, please visualize feature correspondences to check your parameter setting is reasonable.
- There is a special case where FPFH is not very working well. Check issue #5 and #6. Though we are recommending FPFH, FGR does not limit to certain feature. Please let us know if you find better choice.

## Why FGR is not as fast as my ICP?

There are several reasons:
- The number of points is too large. Enumerate your points before use, and downsampling it. As we reported in the paper, with point cloud pairs having 10K the procedure should be finished within few milliseconds.
- We provide single thread version for pair comparison. If you like, you can parallelize the matching procedure (a.k.a, KNN function calls). As discussed in #7, this will speed up a lot.
- Try reduce `TUPLE_MAX_CNT` to 300. We had set it conservative. However, this will sacrifice the accuracy.

## How can I use live point clouds from PCL?
- Please check a discussion in #7.
