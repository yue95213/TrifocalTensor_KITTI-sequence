function [ trajectory2 ] = GetTrajectory( Rotation, translation, trajectory1 )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

trajectory2 = trajectory1 / [Rotation translation ; 0 0 0 1];

end

