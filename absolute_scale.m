function [ t ] = absolute_scale( t, ground_truth, frame_id, lastframe_id )
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here

%% GT pose subject to first frame
true = [reshape(ground_truth(frame_id+1,:),[4,3])'; 0 0 0 1];
prev_true = [reshape(ground_truth(lastframe_id+1,:),[4,3])'; 0 0 0 1];

%% GT: total rotation
P2 = true\ prev_true ;
t_GT = P2(1:3,4);
scale = norm(t_GT);
if sign(t_GT(end)) ~= sign(t(end))
    t = -1*t/norm(t) * scale;
else
    t = t/norm(t) * scale;
end

end

