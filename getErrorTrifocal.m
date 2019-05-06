function [ error ] = getErrorTrifocal( Corresp,Reconst, P12, P13, CalM )
%Used to obtain error between 2 coordinates

[~,count] = size(Corresp);
Reconst = [Reconst; ones(1,count)];

x1=Corresp(1:2,:);
x2=Corresp(3:4,:);
x3=Corresp(5:6,:);

% Normalization of the data
[x1,~]=Normalize2Ddata(Corresp(1:2,:));
[x2,~]=Normalize2Ddata(Corresp(3:4,:));
[x3,~]=Normalize2Ddata(Corresp(5:6,:));

% get reprojection point
x1_ = CalM(1:3,:) * [eye(3) zeros(3,1)] * Reconst;
x2_ = CalM(4:6,:) * P12 * Reconst;
x3_ = CalM(7:9,:) * P13 * Reconst;
x1_ = x1_(1:2,:)./repmat(x1_(3,:),2,1);
x2_ = x2_(1:2,:)./repmat(x2_(3,:),2,1);
x3_ = x3_(1:2,:)./repmat(x3_(3,:),2,1);

[x1_,~]=Normalize2Ddata(x1_);
[x2_,~]=Normalize2Ddata(x2_);
[x3_,~]=Normalize2Ddata(x3_);

% distance
d1 = sum((x1-x1_).^2,1);
d2 = sum((x2-x2_).^2,1);
d3 = sum((x3-x3_).^2,1);

% reprojection error
error = sqrt(d1+d2+d3);

end

