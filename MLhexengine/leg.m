function [ leg ] = leg( a1,a2,a3,P0 )

    % forward kinematics

    global l1;
    global l2;
    global l3;
    global pivot;
    
    if P0(2) > pivot(2)

        P1 = P0 + l1 * [sind(a1) ; cosd(a1) ; 0];
        P2 = P1 + l2 * [sind(a1)*cosd(a2) ; cosd(a1)*cosd(a2) ; sind(a2)];
        P3 = P2 + l3 * [sind(a1)*cosd(a2-a3) ; cosd(a1)*cosd(a2-a3) ; sind(a2-a3)];
    
    else
        
        P1 = P0 + l1 * [sind(a1) ; -cosd(a1) ; 0];
        P2 = P1 + l2 * [sind(a1)*cosd(a2) ; -cosd(a1)*cosd(a2) ; sind(a2)];
        P3 = P2 + l3 * [sind(a1)*cosd(a2-a3) ; -cosd(a1)*cosd(a2-a3) ; sind(a2-a3)];
        
    end
    
    leg = [P0(1) P0(2) P0(3);...
           P1(1) P1(2) P1(3);...
           P2(1) P2(2) P2(3);...
           P3(1) P3(2) P3(3)];

end
