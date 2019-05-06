function [ feature_points, blocksize ] = extract_block_features( image , blocksize )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

%% just harris corner
minQ = 0.02;
points= detectHarrisFeatures(image, 'MinQuality',minQ);
feature_points = points.Location;

while(size(points,1)<600)
    minQ = minQ/2;
    points= detectHarrisFeatures(image, 'MinQuality',minQ);
    feature_points = points.Location;
end

% %% global constant of gradient threshold
% Gth = 7;
% 
% %% get features each block
% fun = @(block_struct) select_one_feature(block_struct.data, Gth);
% processed_image = blockproc(image,blocksize,fun);
% 
% %% return points 
% [y, x] = find(processed_image > 0);
% feature_points = [x'; y'];
% feature_points =feature_points';
% 
% %% adaptive blocksize & threshold
% if (size(feature_points,1) < 700 )
%     % get features each block
%     fun = @(block_struct) select_one_feature(block_struct.data, Gth/2);
%     processed_image = blockproc(image, blocksize-10, fun);
%     
%     % return points
%     [y, x] = find(processed_image > 0);
%     feature_points = [x'; y'];
%     feature_points =feature_points';
%     
%     disp('decrease blocksize & gradient threshold');
%     
% end

end

