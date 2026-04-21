function [H_o2c, H_c2o] = autoCalibrateCameraAndRobot(prv, ur, cameraParams, q_saved, squareSize)
    %% Automated image capture
    imBaseName = 'autoCoCal';
    calFolderName = 'RobCamCal';
    
    if ~isfolder(calFolderName)
        mkdir(calFolderName);
    end
    
    nImages = size(q_saved, 2);
    H_e2o = {}; 
    fnames = {}; 
    
    disp('Starting automated calibration capture...');
    for i = 1:nImages
        fprintf('Moving to pose %d of %d...\n', i, nImages);
        ur.Joints = q_saved(:, i);
        pause(3.0); 
        
        im = get(prv, 'CData');
        H_e2o{i} = ur.Pose;
        
        fname = sprintf('%s%03d.png', imBaseName, i);
        fnames{i} = fullfile(calFolderName, fname);
        imwrite(im, fnames{i}, 'png');
    end
    
    disp('Image capture complete. Processing extrinsics...');
    
    %% Process Checkerboards and Establish Correspondence
    [imagePoints, boardSize, tfImagesUsed] = detectCheckerboardPoints(fnames, 'PartialDetections', false);
    fnames_Used = fnames(tfImagesUsed);
    H_e2o_Used = H_e2o(tfImagesUsed);
    
    [worldPoints] = generateCheckerboardPoints(boardSize, squareSize);
    
    H_f2c_Used = {};
    for i = 1:size(imagePoints,3)
        [R_c2f, tpose_d_f2c] = extrinsics(imagePoints(:,:,i), worldPoints, cameraParams);
        H_f2c_Used{i} = [R_c2f.', tpose_d_f2c.'; 0,0,0,1];
    end
    
    %% Calculate A and B for AX = XB
    iter = 0; A_f = {}; B_e = {}; A_o = {}; B_c = {};
    n = numel(H_f2c_Used);
    for i = 1:n
        for j = 1:n
            H_fi2fj{i,j} = invSE(H_f2c_Used{j}) * H_f2c_Used{i};
            H_ei2ej{i,j} = invSE(H_e2o_Used{j}) * H_e2o_Used{i};
            H_oi2oj{i,j} = (H_e2o_Used{j}) * invSE(H_e2o_Used{i});
            H_ci2cj{i,j} = H_f2c_Used{j} * invSE(H_f2c_Used{i});
            
            if i ~= j && i < j
                iter = iter+1;
                A_f{iter} = H_fi2fj{i,j}; B_e{iter} = H_ei2ej{i,j};
                A_o{iter} = H_oi2oj{i,j}; B_c{iter} = H_ci2cj{i,j};
            end
        end
    end
    
    %% Solve AX = XB
    H_e2f = nearestSE(solveAXeqXBinSE(A_f, B_e));
    H_c2o = nearestSE(solveAXeqXBinSE(A_o, B_c));
    H_f2e = invSE(H_e2f);
    H_o2c = invSE(H_c2o);
    
    %% Validate Calibration Visually
    disp('Validating Matrices via Reprojection...');
    A_c2m = cameraParams.IntrinsicMatrix.';
    p_f = [worldPoints, zeros(size(worldPoints,1),1), ones(size(worldPoints,1),1)].'; 
    
    for i = 1:n
        im = undistortImage(imread(fnames_Used{i}), cameraParams);
        fig = figure('Name', ['Validation: ', fnames_Used{i}]);
        axs = axes('Parent', fig);
        imshow(im, 'Parent', axs); hold(axs, 'on'); axis(axs, 'tight');
        
        H_f2c_i = H_o2c * H_e2o_Used{i} * H_f2e;
        P_f2m = A_c2m * H_f2c_i(1:3,:);
        
        tilde_p_m = P_f2m * p_f; 
        p_m = tilde_p_m ./ tilde_p_m(3,:); 
        
        plot(axs, p_m(1,1), p_m(2,1), 'ys', 'LineWidth', 3, 'MarkerSize', 8);
        plot(axs, p_m(1,2:end), p_m(2,2:end), 'go', 'LineWidth', 2, 'MarkerSize', 8);
    end
    
    fnameRobotInfo = 'URcoCalInfo.mat';
    q = q_saved; 
    save(fullfile(calFolderName, fnameRobotInfo), 'q', 'H_c2o', 'H_o2c', 'cameraParams', 'squareSize');
    disp('Calibration Automated & Saved Successfully.');
end

function invH = invSE(H)
    R = H(1:3, 1:3); d = H(1:3, 4);
    invH = [R.', -R.'*d; 0, 0, 0, 1];
end