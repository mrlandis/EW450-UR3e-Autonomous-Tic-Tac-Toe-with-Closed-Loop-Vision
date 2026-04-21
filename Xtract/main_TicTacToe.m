% main_TicTacToe.m
clear; clc; close all;

%% 1. Initialization
disp('Waking up the UR3e arm...');
[cam, prv] = initCamera('DFK 23U618');

% Initialize Robot
ur = URQt('UR3e');
ur.Initialize; 
ur.GripPosition = 0; % Start with an open hand
disp('Success!');

load(fullfile('HandheldCal', 'HandCalInfo.mat'), 'cameraParams');
load(fullfile('RobCamCal', 'URcoCalInfo.mat'), 'H_c2o', 'H_o2c');
load(fullfile('RobCamCal', 'SavedCalibrationPoses.mat'), 'q_saved');
load(fullfile('RobCamCal', 'SafeRestPose.mat'), 'safeRestPose'); % <-- CUSTOM HOME
ur.Joints = safeRestPose;

% UR3e kinematics links
L = transpose([151.85, 243.55, 213.20, 131.05, 85.35, 92.10+170]);

%% 2. Game Constants
tagFamily = 'tag36h11';
boardTagIDs = [450, 460];     
cpuPieceIDs = [461:465];      
playerPieceIDs = [451:455];   

tagSizeBoard = 29.0;          
tagSizePiece = 25.0;          

% ========================================================
% --- EXACT PHYSICAL BOARD DIMENSIONS ---
% ========================================================
% PERFECT ALIGNMENT: 63mm grid, X shifted +5mm to perfectly bound the board edge!
squareOffsets = [
    17, -178, 0;   % 1 (Top Left)
    80, -178, 0;   % 2 (Top Mid)
    143, -178, 0;  % 3 (Top Right)
    17, -115, 0;   % 4 (Mid Left)
    80, -115, 0;   % 5 (Center)
    143, -115, 0;  % 6 (Mid Right)
    17,  -52, 0;   % 7 (Bottom Left)
    80,  -52, 0;   % 8 (Bottom Mid)
    143,  -52, 0   % 9 (Bottom Right)
];

% ========================================================
% --- TUNING & DUAL-TAG VARIABLES ---
% ========================================================
% Tag 460 is 150mm to the RIGHT (X-axis)
tag460_Offset = [150.0, 0.0]; 

pickOffsetX = 25;   
pickOffsetY = 20;  

zHoverHeight = 100.0; 
zOffsetGrasp = -20.0; 
zOffsetDrop  = -0.0; 

% SPEED TIMERS (in seconds)
tHover  = 0.8;  
tDown   = 0.6;  
tGrip   = 0.3;  
tReturn = 1.0;  
% ========================================================

%% 3. Pre-Game Setup
gameState = zeros(1, 9); 
isGameOver = false;

unplayedCpuIDs = cpuPieceIDs; 

% RECOVERY VARIABLES
recoveryMode = false;
recoveryID = 0;
recoverySquare = 0;

disp('Moving out of view...');
ur.Joints = safeRestPose;
pause(tReturn);

% ========================================================
% --- INITIAL AR PAINT ---
% Paints the live radar before asking who goes first!
% ========================================================
disp('Painting AR boundaries...');
imDist = getsnapshot(cam);
im = undistortImage(imDist, cameraParams);
[idPieces, ~, posePieces, ~] = readAprilTag(im, tagFamily, cameraParams, tagSizePiece);
[idBoard, ~, poseBoard, ~] = readAprilTag(im, tagFamily, cameraParams, tagSizeBoard);
[H_board2c, boardVisible] = getBoardOrigin(idBoard, poseBoard, tag460_Offset);

if boardVisible
    drawLiveOverlay(prv, H_board2c, squareOffsets, cameraParams, idPieces, posePieces, idBoard, poseBoard, cpuPieceIDs, playerPieceIDs);
    disp('Board locked. Boundaries drawn.');
else
    drawLiveOverlay(prv, [], squareOffsets, cameraParams, idPieces, posePieces, idBoard, poseBoard, cpuPieceIDs, playerPieceIDs);
    disp('Warning: Cannot see origin tags. Make sure the board is in view!');
end
% ========================================================

disp('   WELCOME TO TIC-TAC-TOE!');
userInput = input('Who goes first? Enter 1 for CPU, 2 for PLAYER: ');
if userInput == 1
    currentPlayer = 1;
    disp('CPU Turn.');
else
    currentPlayer = -1;
    disp('Your Turn.');
end

disp('Tracking the board...');

%% 4. Main Game Loop
while ~isGameOver
    
    % --- PLAYER'S TURN ---
    if currentPlayer == -1
        disp('Waiting for you to make a move...');
        playerMoved = false;
        
        while ~playerMoved
            imDist = getsnapshot(cam); 
            im = undistortImage(imDist, cameraParams);
            
            [idPieces, ~, posePieces, ~] = readAprilTag(im, tagFamily, cameraParams, tagSizePiece);
            [idBoard, ~, poseBoard, ~] = readAprilTag(im, tagFamily, cameraParams, tagSizeBoard);
            
            [H_board2c, boardVisible] = getBoardOrigin(idBoard, poseBoard, tag460_Offset);
            if ~boardVisible
                % Even if board is blocked, draw the pieces!
                drawLiveOverlay(prv, [], squareOffsets, cameraParams, idPieces, posePieces, idBoard, poseBoard, cpuPieceIDs, playerPieceIDs);
                pause(0.5); 
                continue;
            end
            
            % Full Live overlay
            drawLiveOverlay(prv, H_board2c, squareOffsets, cameraParams, idPieces, posePieces, idBoard, poseBoard, cpuPieceIDs, playerPieceIDs);
            
            currentState = updateBoardState(idPieces, posePieces, H_board2c, cpuPieceIDs, playerPieceIDs, squareOffsets);
            
            if sum(currentState == -1) > sum(gameState == -1)
                disp('Cool, you know how to play tic-tac-toe, shocker.');
                pause(0.1); 
                
                imDist = getsnapshot(cam); 
                im = undistortImage(imDist, cameraParams);
                
                [idPieces, ~, posePieces, ~] = readAprilTag(im, tagFamily, cameraParams, tagSizePiece);
                [idBoard, ~, poseBoard, ~] = readAprilTag(im, tagFamily, cameraParams, tagSizeBoard);
                
                [H_board2c, boardVisible] = getBoardOrigin(idBoard, poseBoard, tag460_Offset);
                
                if boardVisible
                    drawLiveOverlay(prv, H_board2c, squareOffsets, cameraParams, idPieces, posePieces, idBoard, poseBoard, cpuPieceIDs, playerPieceIDs);
                    gameState = updateBoardState(idPieces, posePieces, H_board2c, cpuPieceIDs, playerPieceIDs, squareOffsets);
                    disp('Current Board State:');
                    disp(reshape(gameState, 3, 3).');
                    playerMoved = true; 
                else
                    disp('Move your hand, bum.');
                end
            else
                pause(1.0); % Prevents camera crash
            end
        end
        
        [isGameOver, winner] = checkWinCondition(gameState);
        if isGameOver
            break;
        end
        
        currentPlayer = 1; 
    
    % --- CPU'S TURN ---
    elseif currentPlayer == 1
        
        imDist = getsnapshot(cam);
        im = undistortImage(imDist, cameraParams);
        
        [idPieces, ~, posePieces, ~] = readAprilTag(im, tagFamily, cameraParams, tagSizePiece);
        [idBoard, ~, poseBoard, ~] = readAprilTag(im, tagFamily, cameraParams, tagSizeBoard);
        
        [H_board2c, boardVisible] = getBoardOrigin(idBoard, poseBoard, tag460_Offset);
        if ~boardVisible
            disp('Origin tags not in view.');
            continue; 
        end
        H_board2o = H_c2o * H_board2c;
        
        % --- THE RECOVERY PROTOCOL ---
        if recoveryMode
            disp('RECOVERY MODE: Locating misplaced piece...');
            targetSquare = recoverySquare;
            availablePieceIdx = find(idPieces == recoveryID, 1);
            
            if isempty(availablePieceIdx)
                disp('Cannot see the misplaced piece! Falling back to a new piece from staging.');
                recoveryMode = false;
                continue; 
            end
            usedID = recoveryID; 
        else
            disp('CPU is calculating.');
            targetSquare = calculateCPUMove(gameState);
            availablePieceIdx = findAvailablePiece(idPieces, posePieces, unplayedCpuIDs);
            
            if availablePieceIdx == 0
                disp('CPU can''t see any unplayed pieces in staging.');
                break;
            end
            
            usedID = idPieces(availablePieceIdx);
            unplayedCpuIDs = unplayedCpuIDs(unplayedCpuIDs ~= usedID);
        end
        
        % --- Pick ---
        ur.GripPosition = 0; pause(tGrip);
        H_piece2c = posePieces(availablePieceIdx).T.';
        
        H_localOffset = eye(4);
        H_localOffset(1, 4) = pickOffsetX;
        H_localOffset(2, 4) = pickOffsetY;
        H_piece2c_corrected = H_piece2c * H_localOffset;
        H_piece2o = H_c2o * H_piece2c_corrected; 
        
        H_hoverPick = H_piece2o; 
        H_hoverPick(3, 4) = H_hoverPick(3, 4) + zHoverHeight;
        ur.Joints = ikinPickAndPlace(poseToTask(H_hoverPick), L); pause(tHover);
        
        H_graspPick = H_piece2o; 
        H_graspPick(3, 4) = H_graspPick(3, 4) + zOffsetGrasp;
        ur.Joints = ikinPickAndPlace(poseToTask(H_graspPick), L); pause(tDown);
        
        ur.GripPosition = 52; pause(tGrip); 
        ur.Joints = ikinPickAndPlace(poseToTask(H_hoverPick), L); pause(tDown);

        % --- Place --- 
        targetOffset = squareOffsets(targetSquare, :);
        H_target2board = eye(4);
        H_target2board(1:3, 4) = targetOffset.';
        
        H_placeOffset = eye(4);
        H_placeOffset(1, 4) = pickOffsetX;
        H_placeOffset(2, 4) = pickOffsetY;
        H_target2o = H_board2o * H_target2board * H_placeOffset; 
        
        H_hoverPlace = H_target2o; 
        H_hoverPlace(3, 4) = H_hoverPick(3, 4); 
        ur.Joints = ikinPickAndPlace(poseToTask(H_hoverPlace), L); pause(tHover);
        
        H_releasePlace = H_target2o; 
        H_releasePlace(3, 4) = H_releasePlace(3, 4) + zOffsetDrop; 
        ur.Joints = ikinPickAndPlace(poseToTask(H_releasePlace), L); pause(tDown);
        
        ur.GripPosition = 0; pause(tGrip); 
        ur.Joints = ikinPickAndPlace(poseToTask(H_hoverPlace), L); pause(tDown);
        
        disp('Moving out of view...');
        ur.Joints = safeRestPose; 
        pause(tReturn);
        
        % --- VERIFICATION SCAN ---
        disp('Verifying play...');
        
        imDist = getsnapshot(cam);
        im = undistortImage(imDist, cameraParams);
        
        [idPieces, ~, posePieces, ~] = readAprilTag(im, tagFamily, cameraParams, tagSizePiece);
        [idBoard, ~, poseBoard, ~] = readAprilTag(im, tagFamily, cameraParams, tagSizeBoard);
        
        [H_board2c, boardVisible] = getBoardOrigin(idBoard, poseBoard, tag460_Offset);
        if boardVisible
            drawLiveOverlay(prv, H_board2c, squareOffsets, cameraParams, idPieces, posePieces, idBoard, poseBoard, cpuPieceIDs, playerPieceIDs);
            gameState = updateBoardState(idPieces, posePieces, H_board2c, cpuPieceIDs, playerPieceIDs, squareOffsets);
        else
            drawLiveOverlay(prv, [], squareOffsets, cameraParams, idPieces, posePieces, idBoard, poseBoard, cpuPieceIDs, playerPieceIDs);
            disp('Warning: Board tags occluded after move.');
        end
        
        disp('TRUE board state seen by the camera:');
        disp(reshape(gameState, 3, 3).');
        
        if gameState(targetSquare) == 1
            disp('Move verified!');
            recoveryMode = false;
            
            [isGameOver, winner] = checkWinCondition(gameState);
            if isGameOver
                break;
            end
            
            currentPlayer = -1; 
        else
            disp('===============================================');
            disp('ERROR: Piece did not land in target square.');
            disp('Initiating Recovery Protocol...');
            disp('===============================================');
            
            recoveryMode = true;
            recoveryID = usedID;
            recoverySquare = targetSquare;
        end
    end
end

%% 5. Game Over Handling
disp('=================================');
if winner == 1
    disp('GAME OVER: CPU Wins!');
    ur.Home; ur.Joints = safeRestPose; ur.Home; ur.Joints = safeRestPose;
elseif winner == -1
    disp('GAME OVER: PLAYER Wins!');
else
    disp('GAME OVER: It''s a Draw!');
end
disp('=================================');
ur.GripPosition = 0; 
ur.Joints = safeRestPose; 

%% LOCAL HELPER FUNCTIONS

% --- LIVE AR RADAR OVERLAY ---
function drawLiveOverlay(prv, H_board2c, squareOffsets, cameraParams, idPieces, posesPieces, idBoard, posesBoard, cpuIDs, playerIDs)
    ax = prv.Parent;
    delete(findobj(ax, 'Tag', 'LiveRadar'));
    hold(ax, 'on');
    
    % 1. Draw Board Grid (Green)
    if ~isempty(H_board2c)
        R = H_board2c(1:3, 1:3);
        t = H_board2c(1:3, 4).';
        
        for j = 1:9
            cx = squareOffsets(j, 1);
            cy = squareOffsets(j, 2);
            
            box3D = [
                cx-31.5, cy-31.5, 0;
                cx+31.5, cy-31.5, 0;
                cx+31.5, cy+31.5, 0;
                cx-31.5, cy+31.5, 0
            ];
            
            box2D = worldToImage(cameraParams, R, t, box3D);
            plot(ax, [box2D(:,1); box2D(1,1)], [box2D(:,2); box2D(1,2)], 'g-', 'LineWidth', 2, 'Tag', 'LiveRadar');
            text(ax, box2D(1,1), box2D(1,2) - 15, sprintf('Square %d', j), 'Color', 'g', 'FontSize', 12, 'FontWeight', 'bold', 'Tag', 'LiveRadar');
        end
    end
    
    % 2. Draw Board Origin Tags (Magenta)
    for i = 1:length(idBoard)
        drawTagBox(ax, idBoard(i), posesBoard(i), cameraParams, 29.0, 'm');
    end
    
    % 3. Draw All Game Pieces
    for i = 1:length(idPieces)
        tagID = idPieces(i);
        
        % Prevent the piece scanner from overwriting the magenta board tags!
        if ismember(tagID, [450, 460])
            continue; 
        end
        
        if ismember(tagID, cpuIDs)
            color = 'r'; % CPU Pieces = Red
        elseif ismember(tagID, playerIDs)
            color = 'c'; % Player Pieces = Cyan
        else
            color = 'y'; % Unknown Tags = Yellow
        end
        drawTagBox(ax, tagID, posesPieces(i), cameraParams, 25.0, color);
    end
    
    hold(ax, 'off');
    drawnow;
end

% --- HELPER TO DRAW INDIVIDUAL TAGS ---
function drawTagBox(ax, tagID, pose, cameraParams, tagSize, color)
    H = pose.T.';
    R = H(1:3, 1:3);
    t = H(1:3, 4).';
    s = tagSize / 2.0; % Half-size from the center
    
    % The 4 corners of the AprilTag in its own local 3D coordinate frame
    box3D = [
        -s, -s, 0;
         s, -s, 0;
         s,  s, 0;
        -s,  s, 0
    ];
    
    box2D = worldToImage(cameraParams, R, t, box3D);
    plot(ax, [box2D(:,1); box2D(1,1)], [box2D(:,2); box2D(1,2)], '-', 'Color', color, 'LineWidth', 2, 'Tag', 'LiveRadar');
    text(ax, box2D(1,1), box2D(1,2) - 10, sprintf('ID %d', tagID), 'Color', color, 'FontSize', 10, 'FontWeight', 'bold', 'Tag', 'LiveRadar');
end

% --- DUAL-TAG SENSOR FUSION & VERIFICATION ---
function [H_board2c, isVisible] = getBoardOrigin(idBoard, poseBoard, offset460)
    idx450 = find(idBoard == 450, 1);
    idx460 = find(idBoard == 460, 1);
    
    isVisible = false;
    H_board2c = eye(4);
    
    if isempty(idx450) && isempty(idx460)
        return; % Neither tag is visible
    end
    
    isVisible = true;
    
    if ~isempty(idx450) && isempty(idx460)
        % Normal operation (460 is blocked)
        H_board2c = poseBoard(idx450).T.';
        
    elseif isempty(idx450) && ~isempty(idx460)
        % REDUNDANCY: 450 is blocked, fallback to 460!
        H_4602c = poseBoard(idx460).T.';
        
        % Shift the origin backward from 460 to where 450 should be
        H_shift = eye(4); 
        H_shift(1, 4) = -offset460(1); 
        H_shift(2, 4) = -offset460(2);
        
        H_board2c = H_4602c * H_shift;
        fprintf('Radar: Tag 450 blocked. Origin calculated from Tag 460.\n');
        
    else
        % FUSION: Both are visible. Average them and verify scale!
        H_4502c = poseBoard(idx450).T.';
        H_4602c = poseBoard(idx460).T.';
        
        % Check depth scaling reality
        measDist = norm(H_4602c(1:3, 4) - H_4502c(1:3, 4));
        trueDist = norm(offset460);
        
        % Only print the warning if it's drifting by more than 15mm
        if abs(measDist - trueDist) > 15.0
            fprintf('WARNING: Dual-Tag Verify failed. Measured %.1fmm but expected %.1fmm.\n', measDist, trueDist);
        end
        
        % Shift 460 back to 450, then average the translations to cut out jitter
        H_shift = eye(4); 
        H_shift(1, 4) = -offset460(1); 
        H_shift(2, 4) = -offset460(2);
        H_board2c_from_460 = H_4602c * H_shift;
        
        H_board2c = H_4502c; % Keep 450's rotation for stability
        H_board2c(1:3, 4) = (H_4502c(1:3, 4) + H_board2c_from_460(1:3, 4)) / 2.0; % Average translation
    end
end

function state = updateBoardState(ids, poses, H_board2c, cpuIDs, playerIDs, squareOffsets)
    state = zeros(1, 9);
    H_c2board = invSE(H_board2c);
    
    disp('--- CAMERA RADAR ---');
    for i = 1:length(ids)
        currentID = ids(i);
        isCPU = ismember(currentID, cpuIDs);
        isPlayer = ismember(currentID, playerIDs);
        
        if isCPU || isPlayer
            H_piece2c = poses(i).T.';
            H_piece2board = H_c2board * H_piece2c;
            piecePos = H_piece2board(1:3, 4).';
            
            minDist = inf;
            closestSquare = -1;
            for j = 1:9
                dist = norm(piecePos(1:2) - squareOffsets(j, 1:2));
                if dist < minDist
                    minDist = dist;
                    closestSquare = j;
                end
            end
            
            % TOLERANCE TIGHTENED TO 32 TO MATCH EXACT 63mm BOXES
            if minDist < 32 
                if isCPU
                    state(closestSquare) = 1;
                    fprintf('Radar: CPU Piece %d found at X: %6.1f, Y: %6.1f -> Mapped to Square %d\n', currentID, piecePos(1), piecePos(2), closestSquare);
                else
                    state(closestSquare) = -1;
                    fprintf('Radar: PLAYER Piece %d found at X: %6.1f, Y: %6.1f -> Mapped to Square %d\n', currentID, piecePos(1), piecePos(2), closestSquare);
                end
            else
                fprintf('Radar: Ignored Piece %d at X: %6.1f, Y: %6.1f (Missed Square %d by %.1f mm)\n', currentID, piecePos(1), piecePos(2), closestSquare, minDist);
            end
        end
    end
    disp('--------------------');
end

function moveIdx = calculateCPUMove(state)
    winLines = [1 2 3; 4 5 6; 7 8 9; 1 4 7; 2 5 8; 3 6 9; 1 5 9; 3 5 7];
    for i = 1:8
        line = winLines(i, :);
        if sum(state(line) == 1) == 2 && sum(state(line) == 0) == 1
            moveIdx = line(state(line) == 0); return;
        end
    end
    for i = 1:8
        line = winLines(i, :);
        if sum(state(line) == -1) == 2 && sum(state(line) == 0) == 1
            moveIdx = line(state(line) == 0); return;
        end
    end
    if state(5) == 0
        moveIdx = 5; return;
    end
    corners = [1, 3, 7, 9];
    availCorners = corners(state(corners) == 0);
    if ~isempty(availCorners)
        moveIdx = availCorners(1); return;
    end
    edges = [2, 4, 6, 8];
    availEdges = edges(state(edges) == 0);
    if ~isempty(availEdges)
        moveIdx = availEdges(1); return;
    end
    emptySquares = find(state == 0);
    moveIdx = emptySquares(1);
end

function [isOver, winner] = checkWinCondition(state)
    winLines = [1 2 3; 4 5 6; 7 8 9; 1 4 7; 2 5 8; 3 6 9; 1 5 9; 3 5 7];
    isOver = false; winner = 0;
    for i = 1:8
        lineSum = sum(state(winLines(i, :)));
        if lineSum == 3
            isOver = true; winner = 1; return;
        elseif lineSum == -3
            isOver = true; winner = -1; return;
        end
    end
    if ~any(state == 0)
        isOver = true; 
    end
end

function targetIdx = findAvailablePiece(ids, poses, activeCpuIDs)
    targetIdx = 0;
    for i = 1:length(ids)
        if ismember(ids(i), activeCpuIDs)
            targetIdx = i; return;
        end
    end
end

function invH = invSE(H)
    R = H(1:3, 1:3); d = H(1:3, 4);
    invH = [R.', -R.'*d; 0, 0, 0, 1];
end