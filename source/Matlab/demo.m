
[pt1, feat1] = read_features('../../dataset/pairwise_noise_xyz_level_02_01_rot_05/features_0000.bin');
[pt2, feat2] = read_features('../../dataset/pairwise_noise_xyz_level_02_01_rot_05/features_0001.bin');

T = fast_global_registration(pt1,feat1,pt2,feat2); 

disp('Trasformation:')
disp(T)
