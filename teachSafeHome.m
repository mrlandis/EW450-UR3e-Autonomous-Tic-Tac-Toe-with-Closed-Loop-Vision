% teachSafeHome.m
clear; clc; close all;

disp('Connecting to robot...');
ur = URQt('UR3e');
ur.Initialize;

% Wait for you to physically move the arm
uiwait(msgbox('Please put the robot in Freedrive and move the arm completely out of the camera''s view. Click OK when ready.', 'Set Safe Home'));

% Read the position
safeRestPose = ur.Joints;

% Save it to your calibration folder
save(fullfile('RobCamCal', 'SafeRestPose.mat'), 'safeRestPose');

disp('===================================================');
disp('SUCCESS: Safe rest position saved to RobCamCal/SafeRestPose.mat!');
disp('Your robot will now use this instead of ur.Home.');
disp('===================================================');