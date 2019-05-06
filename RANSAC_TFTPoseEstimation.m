function     [P12_best,P13_best,T_best,matcheper_best]=RANSAC_TFTPoseEstimation(Corresp,CalM)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

[~,n] = size(Corresp);
iter_num = 1000;
threshold = 0.01;

%% homogenous coordinates
% Normalization of the data
[x1,Normal1]=Normalize2Ddata(Corresp(1:2,:));
[x2,Normal2]=Normalize2Ddata(Corresp(3:4,:));
[x3,Normal3]=Normalize2Ddata(Corresp(5:6,:));

pts1 = [x1; ones(1,n)];
pts2 = [x2; ones(1,n)];
pts3 = [x3; ones(1,n)];

%% ransac iteration
matcheper = zeros(1,iter_num);
Tri = zeros(3,3,3);
Tri_all = zeros(27,iter_num);
recons_time =0;

parfor iter = 1:iter_num
%     tic
    matcheper_temp = 0;
    
    % randomly sampling
%     idx = 1:n; % test
    idx = randi(n, 1, 7);
    
    % linear trifocal tensor estimation
    [Tri_temp] = linearTFT(x1(:,idx),x2(:,idx),x3(:,idx));
%     toc
    
    Tri_all(:,iter) = reshape(Tri_temp,27,1);
    
end

parfor j=1:n
        u1=x1(1,j); v1=x1(2,j);
        u2=x2(1,j); v2=x2(2,j);
        u3=x3(1,j); v3=x3(2,j);
        
        A_temp1=[u1,0,-u1*u2, 0,0,0, -u1*u3,0,u1*u2*u3,...
            v1,0,-u2*v1,  0,0,0, -u3*v1,0,u2*u3*v1,...
            1,0,-u2, 0,0,0, -u3,0,u2*u3];
        A_temp2=[0,u1,-u1*v2, 0,0,0, 0,-u1*u3,u1*u3*v2,...
            0,v1,-v1*v2, 0,0,0, 0,-u3*v1,u3*v1*v2,...
            0,1,-v2, 0,0,0, 0,-u3,u3*v2];
        A_temp3=[0,0,0, u1,0,-u1*u2, -u1*v3,0,u1*u2*v3,...
            0,0,0, v1,0,-u2*v1, -v1*v3,0,u2*v1*v3,...
            0,0,0, 1,0,-u2, -v3,0,u2*v3];
        A_temp4=[0,0,0, 0,u1,-u1*v2, 0,-u1*v3,u1*v2*v3,...
            0,0,0, 0,v1,-v1*v2, 0,-v1*v3,v1*v2*v3,...
            0,0,0, 0,1,-v2, 0,-v3,v2*v3];
        A = abs(A_temp1*Tri_all)+abs(A_temp2*Tri_all)+abs(A_temp3*Tri_all)+abs(A_temp4*Tri_all);
        bool_in = A<0.005;
        
        matcheper = matcheper + bool_in;
    
%     toc

end

%% get the final result
max_in = max(matcheper);
max_idx = find(matcheper==max_in);
Tri = reshape(Tri_all(:,max_idx(1)),3,3,3);
matcheper_best = max_in/n

% tensor denormalization
T_best = transform_TFT(Tri,Normal1,Normal2,Normal3,1);

% get fundamental matrix
[F12,F13] = F_from_TFT(T_best);
% toc

% Find orientation using calibration and TFT
[P12_best,P13_best]=R_t_from_TFT(T_best,CalM,Corresp);

% get 3D reconstruction of all points
% Reconst=triangulation3D({CalM(1:3,:)*eye(3,4),CalM(4:6,:)*P12_best,CalM(7:9,:)*P13_best},Corresp);
% Reconst=Reconst(1:3,:)./repmat(Reconst(4,:),3,1);
% error = getErrorTrifocal(Corresp,Reconst, P12_best, P13_best,CalM);
% inlier_idx = find(error < threshold);

% %% visualizing features
% hold on ;
% plot(Corresp(5,:),Corresp(6,:),'g*','LineWidth',1.5);
% hold on ;
% plot(Corresp(5,inlier_idx),Corresp(6,inlier_idx),'ro','LineWidth',1.5);

% text(900,400,['sum distances: ', num2str(sum(error))]);

end

