% global variables

global l1;
global l2;
global l3;
global pivot;
global half_length;
global half_width1;
global half_width2;

global t1;
global t2;
global t3;
global t4;
global t5;
global t6;

global swing1;
global swing2;
global swing3;
global swing4;
global swing5;
global swing6;

global dt;

% control vector that can be send to the hexapod by a remote control

control = [1;... % X (forward travel)
           0.5;... % Y (side travel)
           2;... % Z (pivot height)
           1];   % step size
       
% variables

l1 = 1; % coxa
l2 = 3; % femur
l3 = 4; % tibia

half_length = 4; % half body length
half_width1 = 1.5; % front and rear half width
half_width2 = 2.5; % middle half width
legdist = 4; % y leg distance from body-coxa joint in normal position

h = 1; % step height

dt = 10;
tick = 1; % dt/tick calculations per swing
stepcount = 5;

pivot = [0 ; 0 ; control(3)];

time = 0;

Npos = [0 ; legdist ; 0];

x1 = 0;
x2 = 0;
x3 = 0;
x4 = 0;
x5 = 0;
x6 = 0;

y1 = 0;
y2 = 0;
y3 = 0;
y4 = 0;
y5 = 0;
y6 = 0;

z1 = 0;
z2 = 0;
z3 = 0;
z4 = 0;
z5 = 0;
z6 = 0;

swing1 = 0;
swing2 = 0;
swing3 = 0;
swing4 = 0;
swing5 = 0;
swing6 = 0;

gait = 2;
% 1: Tripod Gait
% 2: Ripple Gait
% 3: Metachronal Wave Gait

oldgait = gait;

switch gait
    case 1
        period = 2;
        t1 = 0;
        t2 = dt;
        t3 = 0;
        t4 = dt;
        t5 = 0;
        t6 = dt;
    case 2
        period = 3;
        t1 = 0;
        t2 = 2*dt;
        t3 = dt;
        t4 = 0.5*dt;
        t5 = 2.5*dt;
        t6 = 1.5*dt;
    case 3
        period = 6;
        t1 = 0;
        t2 = 5*dt;
        t3 = 4*dt;
        t4 = 3*dt;
        t5 = 2*dt;
        t6 = dt;
end

change = 0;

while time <= period*dt*stepcount
    
    norm1 = sqrt(control(1)^2 + control(2)^2);
    X = control(1)/norm1;
    Y = control(2)/norm1;
    s = control(4);
    
    if(gait - oldgait) ~= 0
        switch gait
            case 1
                period = 2;
                t1 = 0;
                t2 = dt;
                t3 = 0;
                t4 = dt;
                t5 = 0;
                t6 = dt;
            case 2
                period = 3;
                t1 = 0;
                t2 = 2*dt;
                t3 = dt;
                t4 = 0.5*dt;
                t5 = 2.5*dt;
                t6 = 1.5*dt;
            case 3
                period = 6;
                t1 = 0;
                t2 = 5*dt;
                t3 = 4*dt;
                t4 = 3*dt;
                t5 = 2*dt;
                t6 = dt;
        end
        
        swing1 = 0;
        swing2 = 0;
        swing3 = 0;
        swing4 = 0;
        swing5 = 0;
        swing6 = 0;
    end
    
    oldgait = gait;
    
    % step calculations
    
    tickstep = tick/(dt*(period-1)); % change per tick
    
    if t1 < dt
        if(swing1 == 0)
            Pold1 = [x1 ; y1 ; z1];
            Pnew1 = s * [X ; Y ; 0];
            swing1 = 1;
        end
        [x1 y1 z1] = swing(t1, dt, Pold1, Pnew1, h);
    else
        x1 = x1 - tickstep*2*s*X;
        y1 = y1 - tickstep*2*s*Y;
        z1 = 0;
        swing1 = 0;
    end
    
    if t2 < dt
        if(swing2 == 0)
            Pold2 = [x2 ; y2 ; z2];
            Pnew2 = s * [X ; Y ; 0];
            swing2 = 1;
        end
        [x2 y2 z2] = swing(t2, dt, Pold2, Pnew2, h);
    else
        x2 = x2 - tickstep*2*s*X;
        y2 = y2 - tickstep*2*s*Y;
        z2 = 0;
        swing2 = 0;
    end
    
    if t3 < dt
        if(swing3 == 0)
            Pold3 = [x3 ; y3 ; z3];
            Pnew3 = s * [X ; Y ; 0];
            swing3 = 1;
        end
        [x3 y3 z3] = swing(t3, dt, Pold3, Pnew3, h);
    else
        x3 = x3 - tickstep*2*s*X;
        y3 = y3 - tickstep*2*s*Y;
        z3 = 0;
        swing3 = 0;
    end
    
    if t4 < dt
        if(swing4 == 0)
            Pold4 = [x4 ; y4 ; z4];
            Pnew4 = s * [X ; Y ; 0];
            swing4 = 1;
        end
        [x4 y4 z4] = swing(t4, dt, Pold4, Pnew4, h);
    else
        x4 = x4 - tickstep*2*s*X;
        y4 = y4 - tickstep*2*s*Y;
        z4 = 0;
        swing4 = 0;
    end
    
    if t5 < dt
        if(swing5 == 0)
            Pold5 = [x5 ; y5 ; z5];
            Pnew5 = s * [X ; Y ; 0];
            swing5 = 1;
        end
        [x5 y5 z5] = swing(t5, dt, Pold5, Pnew5, h);
    else
        x5 = x5 - tickstep*2*s*X;
        y5 = y5 - tickstep*2*s*Y;
        z5 = 0;
        swing5 = 0;
    end
    
    if t6 < dt
        if(swing6 == 0)
            Pold6 = [x6 ; y6 ; z6];
            Pnew6 = s * [X ; Y ; 0];
            swing6 = 1;
        end
        [x6 y6 z6] = swing(t6, dt, Pold6, Pnew6, h);
    else
        x6 = x6 - tickstep*2*s*X;
        y6 = y6 - tickstep*2*s*Y;
        z6 = 0;
        swing6 = 0;
    end
    
    pivot = pivot + [X ; Y ; 0] * tickstep*2*s;
    
    % time calculations

    t1 = t1 + tick;
    t2 = t2 + tick;
    t3 = t3 + tick;
    t4 = t4 + tick;
    t5 = t5 + tick;
    t6 = t6 + tick;
    time = time + tick;

    if(t1 >= period*dt)
        t1 = t1 - period*dt;
    end

    if(t2 >= period*dt)
        t2 = t2 - period*dt;
    end

    if(t3 >= period*dt)
        t3 = t3 - period*dt;
    end
    
    if(t4 >= period*dt)
        t4 = t4 - period*dt;
    end

    if(t5 >= period*dt)
        t5 = t5 - period*dt;
    end

    if(t6 >= period*dt)
        t6 = t6 - period*dt;
    end

    % body

    B = [ half_length + pivot(1)   half_width1 + pivot(2)  pivot(3);...
          pivot(1)                 half_width2 + pivot(2)  pivot(3);...
         -half_length + pivot(1)   half_width1 + pivot(2)  pivot(3);...
          half_length + pivot(1)  -half_width1 + pivot(2)  pivot(3);...
          pivot(1)                -half_width2 + pivot(2)  pivot(3);...
         -half_length + pivot(1)  -half_width1 + pivot(2)  pivot(3)];
      
    body = [ B(1,:);... % second body definition to draw properly in the diagram
             B(2,:);...
             B(3,:);...
             B(6,:);...
             B(5,:);...
             B(4,:);...
             B(1,:)];

    B1 = B(1,:)'; % body coxa joints
    B2 = B(2,:)';
    B3 = B(3,:)';
    B4 = B(4,:)';
    B5 = B(5,:)';
    B6 = B(6,:)';

    % leg definition
    
    P1 = [x1 ; y1 ; z1] + Npos - [0;0;pivot(3)]; % endpoints,
    P2 = [x2 ; y2 ; z2] + Npos - [0;0;pivot(3)]; % relative to each body-coxa joint
    P3 = [x3 ; y3 ; z3] + Npos - [0;0;pivot(3)];
    P4 = [x4 ; -y4 ; z4] + Npos - [0;0;pivot(3)];
    P5 = [x5 ; -y5 ; z5] + Npos - [0;0;pivot(3)];
    P6 = [x6 ; -y6 ; z6] + Npos - [0;0;pivot(3)];

    leg1 = legIK(P1+B1,B1);
    leg2 = legIK(P2+B2,B2);
    leg3 = legIK(P3+B3,B3);
    leg4 = legIK(P4+B4,B4);
    leg5 = legIK(P5+B5,B5);
    leg6 = legIK(P6+B6,B6);

    % visualization

    plot3(leg1(:,1),leg1(:,2),leg1(:,3),'- .r',...
               leg2(:,1),leg2(:,2),leg2(:,3),'- .b',...
               leg3(:,1),leg3(:,2),leg3(:,3),'- .b',...
               leg4(:,1),leg4(:,2),leg4(:,3),'- .b',...
               leg5(:,1),leg5(:,2),leg5(:,3),'- .b',...
               leg6(:,1),leg6(:,2),leg6(:,3),'- .b',...
               body(:,1),body(:,2),body(:,3),'- .k','LineWidth',3)

    axis equal
    grid on
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
    xlim([-10 30])
    ylim([-20 20])
    zlim([0 10])
    title(time)

    drawnow

end

% NOTES:
% ------
% positive acceleration:
% - reduce step length on gait change to maintain velocity
% - increase step length afterwards to accelerate
%
% negative acceleration:
% - reduce step length to reduce velocity
% - change gait and reset step length





