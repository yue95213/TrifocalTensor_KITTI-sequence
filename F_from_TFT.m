function [F12,F13] = F_from_TFT(T)

% epipoles and fundamental matrix
[~,~,V]=svd(T(:,:,1)); v1=V(:,end);
[~,~,V]=svd(T(:,:,2)); v2=V(:,end);
[~,~,V]=svd(T(:,:,3)); v3=V(:,end);
[~,~,V]=svd([v1 v2 v3].'); epi31=V(:,end)*sign(V(end));

[~,~,V]=svd(T(:,:,1).'); v1=V(:,end);
[~,~,V]=svd(T(:,:,2).'); v2=V(:,end);
[~,~,V]=svd(T(:,:,3).'); v3=V(:,end);
[~,~,V]=svd([v1 v2 v3].'); epi21=V(:,end)*sign(V(end));

F12=crossM(epi21)*[T(:,:,1)*epi31 T(:,:,2)*epi31 T(:,:,3)*epi31];
F13=-crossM(epi31)*[T(:,:,1).'*epi21 T(:,:,2).'*epi21 T(:,:,3).'*epi21];


end

