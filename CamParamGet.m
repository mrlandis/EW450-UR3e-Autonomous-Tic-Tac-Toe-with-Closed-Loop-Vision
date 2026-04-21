% CamParamGet.m
clear; clc; close all;

disp('Processing handheld calibration images...');

% Make sure this matches the folder where your perfect pictures are!
calFolderName = 'HandheldCal'; 

% =========================================================================
squareSize = 15.0; % Updated to match the 15.0mm from your screenshot
% =========================================================================

% 1. Get list of all image files
imFiles = [dir(fullfile(calFolderName, '*.png')); dir(fullfile(calFolderName, '*.jpg'))];
if isempty(imFiles)
    error('No images found in the folder. Check the folder name and file types.');
end

filePaths = fullfile(calFolderName, {imFiles.name});

% 2. Detect checkerboards in images
disp('Detecting checkerboard corners...');
[imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints(filePaths);

% 3. Validate we have enough good data
numValidImages = sum(imagesUsed);
fprintf('Successfully detected checkerboards in %d out of %d images.\n', numValidImages, length(filePaths));

if numValidImages < 3
    error('MATLAB needs at least 3 valid images to calculate the math.');
end

% 4. Generate the true world coordinates of the checkerboard
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

% 5. Estimate the camera parameters (SIMPLIFIED FOR YOUR MATLAB VERSION)
disp('Calculating camera parameters...');
[cameraParams, imagesUsed2, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints);

% 6. Save the results exactly as your setup script expects
save(fullfile('RobCamCal', 'HandCalInfo.mat'), 'cameraParams');

% 7. Show a visual confirmation of the reprojection errors
figure('Name', 'Reprojection Errors');
showReprojectionErrors(cameraParams);
title('Camera Calibration Reprojection Errors');

disp('======================================================');
disp('SUCCESS! HandCalInfo.mat has been generated safely.');
disp('You can now run your setup_and_validate.m script.');
disp('======================================================');