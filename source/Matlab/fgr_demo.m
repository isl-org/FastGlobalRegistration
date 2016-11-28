[pt1, feat1] = read_features('../../dataset/pairwise_noise_xyz_level_02_01_rot_05/features_0000.bin');
[pt2, feat2] = read_features('../../dataset/pairwise_noise_xyz_level_02_01_rot_05/features_0001.bin');

T = fast_global_registration(pt1, feat1, pt2, feat2); 

pt2_transformed = (T(1 : 3, 1 : 3) * pt2' + repmat(T(1 : 3, 4), 1, size(pt2, 1)))';

figure();
hold on;
plot3(pt1(:, 1), pt1(:, 2), pt1(:, 3), 'b.');
plot3(pt2_transformed(:, 1), pt2_transformed(:, 2), pt2_transformed(:, 3), 'r.');
view(0, 275);
legend('Target point cloud', 'Source point cloud');
axis equal;

disp('Trasformation:')
disp(T)
