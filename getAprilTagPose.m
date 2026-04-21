function [H_a2c] = getAprilTagPose(imDist,cameraParams,tagFamily,tagID,tagSize)

H_a2c = [];

% Undistort
im = undistortImage(imDist,cameraParams);

% Detect tags with pose
[id,loc,pose,detectedFamily] = readAprilTag(im,tagFamily,cameraParams,tagSize);

for i = 1:length(id)
    if id(i) == tagID
        H_a2c = pose(i).T.';
        return;
    end
end

end