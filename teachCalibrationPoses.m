% teachCalibrationPoses.m
clear; clc; close all;

%% 1. Initialization
disp('Initializing Camera and Robot...');
[cam, prv] = initCamera();
ur = URQt();
ur.Initialize
% Recover camera preview handles (The Lab 7 Method)
handles = recoverPreviewHandles(prv);

%% 2. Setup Recording Variables
nPoses = 10; % UPDATED: Now prompts for 15 calibration poses
q_saved = zeros(6, nPoses); 

calFolderName = 'RobCamCal';
if ~isfolder(calFolderName)
    mkdir(calFolderName);
end

%% 3. Teach Loop
disp('Entering Teach Mode...');
uiwait(msgbox('Please set the teach pendant to Freedrive/Local Control.', 'Teach Mode'));

for i = 1:nPoses
    % Bring camera preview to the foreground
    figure(handles.Figure); 
    
    % Prompt the user
    msg = sprintf(['Use the "Teach" button to move the arm to a new configuration.\n\n', ...
                   'Ensure the entire checkerboard is clearly visible in the camera.\n\n', ...
                   'Recording Pose %d of %d.\n\nClick OK when ready.'], i, nPoses);
    f = msgbox(msg, 'Record Pose');
    uiwait(f); 
    
    % Get the image from the preview
    im = get(prv, 'CData');
    
    % Read and store the joint angles
    q_saved(:, i) = ur.Joints;
    
    % --- THE LAB 7 MAGIC ---
    % Show checkerboard on preview to validate it was captured successfully!
    showCheckerboardOnPreview(prv, im);
    drawnow;
    
    fprintf('Successfully recorded Pose %d.\n', i);
end

% Clear the checkerboard overlay when finished
clearPreview(prv);

%% 4. Save the Data
% Save the matrix to a dedicated file
save(fullfile(calFolderName, 'SavedCalibrationPoses.mat'), 'q_saved');

disp('======================================================');
disp('SUCCESS! Joint configurations saved.');
disp('File saved as: RobCamCal/SavedCalibrationPoses.mat');
disp('You can now run the setup_and_validate.m script.');
disp('======================================================');