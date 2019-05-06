function [ error_ ] = getErrorTrifocal_PointTransfer( Corresp, Tri )
%Used to obtain error between 2 coordinates
    
    count = 0;
    m = struct;
    x1=Corresp(1:2,:);
    x2=Corresp(3:4,:);
    x3=Corresp(5:6,:);
    for i=1:size(Corresp,2),
        m(1).a1 = [x1(:,i); 1];
        m(1).a2 = [x2(:,i); 1];
        m(1).a3 = [x3(:,i); 1];
        p3 = pointTransfer(Tri, m(1).a1, m(1).a2);
        p3 = p3/p3(3);
        error = sqrt(sum((p3 - m(1).a3).*(p3 - m(1).a3)));
        %error
        %p3
        %m(1).a3
        if error<5,
            count = count+1;
        end;
    end;    
    error_ = count/size(Corresp,2);
end

