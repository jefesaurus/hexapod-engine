function [ legout ] = legIK(P,P0)

    % combine forward and inverse kinematics to draw everything in the
    % diagram

    a = IK(P,P0);
    legout = leg(a(1),a(2),a(3),P0);

end