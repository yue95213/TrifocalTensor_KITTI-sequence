function     [Tr_total,ransac_total, true_total ] = update_total_trajectory(Tr_total, ransac_total,true_total)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here


    Tr_total{1} = Tr_total{2};
    Tr_total{2} =  [];

ransac_total{1} = ransac_total{2};
ransac_total{2} = [];

true_total{1} = true_total{2};
true_total{2} = [];


end

