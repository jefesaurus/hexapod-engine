function [ x y z ] = swing(t, dt, P1, P2, h)

    % this function realizes the cycloid trajectory on swing

    % X
    ax = (P2(1) - P1(1)) / 2;
    nx = (P2(1) + P1(1)) / 2;
    
    x = nx - ax * cos(t/dt * pi);
    
    % Y
    ay = (P2(2) - P1(2)) / 2;
    ny = (P2(2) + P1(2)) / 2;
    
    y = ny - ay * cos(t/dt * pi);
    
    % Z    
    z = h/2 * (1 - cos(t/dt * 2*pi)) + (P2(3) - P1(3))*t/dt;
end