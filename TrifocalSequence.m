clc;
clear all;
close all;

%% %%%%%%%%%%%% demo for KITTI gray 00~10 image_0 %%%%%%%%%%%%%%%%
% read images and ground truth file
img_dir = 'E:\Dataset\KITTI\data_odometry_gray\dataset\sequences\06\image_0';
last_frame = 1100;
ground_truth_file = fopen('E:\Dataset\KITTI\data_odometry_poses\poses\06.txt','r');
formatSpec = '%f';
size_truth = [ 12,last_frame+1];
ground_truth = fscanf(ground_truth_file,formatSpec,size_truth)';
start_frame = 0;

%% load camera setting
% sequence 00~02
% K = [718.86 0 608.19; 0 718 185.22; 0 0 1];
% sequence 03
% K = [721.54 0 609.56;  0 721.54 172.85;  0 0 1];
% sequence 04~10
K = [707.09 0 601.89; 0 707.09 183.11; 0 0 1];
cameraParams = cameraParameters('IntrinsicMatrix',K');
CalM = [K; K; K];
        
% directory of imwrite images & trajectory file
write_img = 'E:\Project\Experiments\trifocal tensor\s06_1030';
trifocal_file = fopen([write_img '\trifocal_total.txt'], 'w');

% fisrt frame
Tr_total{1} = [eye(3) zeros(3,1); 0 0 0 1];
fprintf(trifocal_file, '% f', reshape(Tr_total{1}(1:3,:)',[1,12]));
fprintf(trifocal_file, '\n');

%% pre_definited variables (feature matching)
pointTracker = vision.PointTracker('NumPyramidLevels', 4, 'MaxBidirectionalError', 1,...
    'BlockSize', [11,11], 'MaxIterations', 50);
blocksize = [25 25];

%% %%%%%%%%%% create figure %%%%%%%%%%%
figure('Color',[1 1 1]);
ha1 = axes('Position',[0.05,0.7,0.9,0.25]);
axis off;
ha2 = axes('Position',[0.05,0.05,0.9,0.6]);
set(gca,'XTick',-500:10:500);
set(gca,'YTick',-500:10:500);
axis equal,
% grid on,
hold on;

%% %%%%%%%%% precessing time %%%%%%%%%%
% total_time = 0;

%% for each three views
for i = start_frame:2:last_frame
    %% read images
    disp(' ');
    disp(['frame ---- ' num2str(i,'%06d')]);
    img1 = imread([img_dir '/' num2str(i,'%06d') '.png']);
    img2 = imread([img_dir '/' num2str(i+1,'%06d') '.png']);
    img3 = imread([img_dir '/' num2str(i+2,'%06d') '.png']);
    
    axes(ha1); cla;
    imagesc(img3); colormap(gray);

%     %% 1. get point correspondences (harris)
%     points1 = detectHarrisFeatures(img1,'FilterSize',3, 'MinQuality',0.005);
%     points2 = detectHarrisFeatures(img2,'FilterSize',3, 'MinQuality',0.005);
%     points3 = detectHarrisFeatures(img3,'FilterSize',3, 'MinQuality',0.005);
%     
%     [f1, vpts1] = extractFeatures(img1, points1);
%     [f2, vpts2] = extractFeatures(img2, points2);
%     [f3, vpts3] = extractFeatures(img3, points3);
%     
%     pairs12 = matchFeatures(f1, f2, 'MatchThreshold', 10, 'MaxRatio', 0.6);
%     pairs23 = matchFeatures(f2, f3, 'MatchThreshold', 10, 'MaxRatio', 0.6);
%     
%     matchedPoints12_1 = vpts1(pairs12(:,1));
%     matchedPoints12_2 = vpts2(pairs12(:,2));
%     matchedPoints23_2 = vpts2(pairs23(:,1));
%     matchedPoints23_3 = vpts3(pairs23(:,2));
%     
%     matchedTriplets = []; % 3 view point correspondence
%     %count = 0;
%     for j=1:size(pairs12,1),
%         match = find(pairs23(:,1) == pairs12(j,2));
%         if(size(match,1)~= 0)
%             matchedTriplets = [matchedTriplets; pairs12(j,1), pairs12(j,2), pairs23(match, 2)];
%         end;
%     end;
%     
%     % point correspondences matrix (6*m)
%     Corresp = [vpts1(matchedTriplets(:,1)).Location';
%         vpts2(matchedTriplets(:,2)).Location';
%         vpts3(matchedTriplets(:,3)).Location'];

    %% 2. get point correspondences (LK block)
    % feature extraction
    [points1, blocksize] = extract_block_features(img1, blocksize); 
    % initialize pointTracker
    initialize(pointTracker, points1, img1);
    
    % track features
    [pointTracker,points1,points2,blocksize,~] = track_LK_features( ...
        pointTracker , points1, img2, blocksize);
    [pointTracker,points2,points3,blocksize,Id_matches] = track_LK_features( ...
        pointTracker , points2, img3, blocksize);
    points1 = points1(Id_matches,:);
    release(pointTracker);
    
    % point correspondences matrix (6*m)
    Corresp = [points1'; points2'; points3'];
    
    %% trifocal tensor estimation
    
    % 1. linear (all point correspondences)
%     [P12,P13,Reconst,T,iter]=LinearTFTPoseEstimation(Corresp,CalM);

    % 2.  ransac method 1
    
    [P12,P13,T,matcheper]=RANSAC_TFTPoseEstimation(Corresp,CalM);    
    
%     total_time = total_time + toc;
        
    text(10,400,['inlier ratio: ',num2str(matcheper)]);
    text(300,400,['3 view correspondences: ', num2str(size(Corresp,2))]);
    text(700,400,['inlier num: ', num2str(size(Corresp,2)*matcheper)]);
       
    
    %% absolute scale
    P12(1:3,4) = absolute_scale(P12(1:3,4), ground_truth, i+1, i);
    P13(1:3,4) = absolute_scale(P13(1:3,4), ground_truth, i+2, i);
    
    %% get total trajectory
    Tr_total{2} = GetTrajectory(P12(1:3,1:3), P12(1:3,4), Tr_total{1});
    Tr_total{3} = GetTrajectory(P13(1:3,1:3), P13(1:3,4), Tr_total{1});
    
    %% ground truth
    true_total{1} =[reshape(ground_truth(i+1,:),[4,3])' ; 0 0 0 1];
     true_total{2} = [reshape(ground_truth(i+2,:),[4,3])' ; 0 0 0 1];
     true_total{3} = [reshape(ground_truth(i+3,:),[4,3])' ; 0 0 0 1];
    
    %% Result display
    % draw pose of this frame
    axis off;
    axes(ha2);
    plot([Tr_total{1}(1,4) Tr_total{2}(1,4)], ...
        [Tr_total{1}(3,4) Tr_total{2}(3,4)],'.-b','LineWidth',1);
    hold on;
    plot([Tr_total{2}(1,4) Tr_total{3}(1,4)], ...
        [Tr_total{2}(3,4) Tr_total{3}(3,4)],'.-b','LineWidth',1);
    
    hold on;
    plot([true_total{1}(1,4) true_total{2}(1,4)], ...
        [true_total{1}(3,4) true_total{2}(3,4)],'.-r','LineWidth',1);
    hold on;
    plot([true_total{2}(1,4) true_total{3}(1,4)], ...
        [true_total{2}(3,4) true_total{3}(3,4)],'.-r','LineWidth',1);
    
    pause(0.05); refresh;
    
    %% save feature images & trajectory
%     saveas(gcf, [write_img '/' num2str(i+2,'%06d') '.png']);
    fprintf(trifocal_file, '% f', reshape(Tr_total{2}(1:3,:)',[1,12]));
    fprintf(trifocal_file, '\n');
    fprintf(trifocal_file, '% f', reshape(Tr_total{3}(1:3,:)',[1,12]));
    fprintf(trifocal_file, '\n');
    
    %% update total trajectory
    Tr_total{1} = Tr_total{3};
    Tr_total{2} = [];
    Tr_total{3} = [];
    
    
end

fclose(proposed_file);
fclose(ransac_file);