function X = poseToTask(H_e2o)

d_e2o = H_e2o(1:3, 4);
xhat_e2o = H_e2o(1:3, 1);

phi = atan2(xhat_e2o(2), xhat_e2o(1));
X = [d_e2o; phi];