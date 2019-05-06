function [ pointTracker,pts1,pts2 ,blocksize,Id_matches] = track_LK_features( pointTracker , points1, img ,blocksize )
%% track to next image
[points2, point_VALIDITY]  = step(pointTracker, img);
Id_matches = find( point_VALIDITY > 0 );

%% update points & pointTracker
pts1 = points1(Id_matches,:);
pts2 = points2(Id_matches,:);
setPoints(pointTracker, pts2) ;

%% adaptive blocksize
if (size(Id_matches,1) < 500 && blocksize(1)>=7)
    blocksize = blocksize - 2;
end
if (size(Id_matches,1) > 800 || blocksize(1)<7 )
    blocksize = blocksize + 2;
end

end

