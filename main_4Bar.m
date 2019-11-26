%% ME 320-1001 S18 Project 1 main script (Four Bar)
% This is the solution for Project 1 assigned in the Spring Semester of
% 2018 for ME 320-1001. In this project, students were asked to complete
% the skeleton code based on this project. Four bar mechanisms were
% resolved here.

clc;clear;close all

%% Parameter Settings
% In this portion of the code, several different length combinations will
% be tested to see how your code responds. What lengths we will be using is
% the following:

% First we will not be plotting these cases. Your code should respond
% accordingly Stating that these are Change-Point or Non-Grashof

% Note all lengths are to be considered in the unit meters, while all 
% angles are to be consdered in radians.

%Len = [1 1 1 1];                      % All Lengths Equal (Change-Point)
%Len = [.1 .1 .5 .6];                  % Two Shortest (Non-Grashof)
%Len = [.1 .1 .6 .6];                  % Two Shortest and two Longest (Change-Point)
%Len = [.2 .3 .4 .5];                  % S + L = P + Q (All diff lengths Change-Point)
%Len = [.2 .3 .4 .6];                  % S + L > P + Q (All diff Lengths Non-Grashof)

% These cases are plotable, and should be resolved accordingly

%Len = [.1 .3 .6 .6];                  % Two Longest (Grashof)
%Len = [.2 .4 .5 .6];                  % S + L < P + Q (shortest L1)
Len = [.3 .1 .4 .5];                  % S + L < P + Q (shortest L2)
%Len = [.5 .2 .05 .4];                 % S + L < P + Q (shortest L3)

% We will consider th3 the ground link vector. Changing this should be
% arbitrary so we may test several different values here.

th3 = pi/12;                            % (rad) incline of the ground vector 3

% Your code should be adaptible to overlay descriptions. Often we solve a
% mechanism in terms of fundamental vector loops, but there may be points
% of interest rigidly attached to a vector. Knowing the kinematics of a
% vector loop gives you knowledge of all these points. This is just one
% overlay, but any other overlay must be possible, and overlays should be
% capable of turning off for visualization.

% I chose a simple box here, where link width is specified and percentages
% of both Len and Wid are used for box_x and box_y.

Wid = .02*ones(1,4);                    % Vector of link widths
box_x = [-.02 1.02 1.02 -.02 -.02];     % vector of x-coordinate box shape 
box_y = [0.5 0.5 -0.5 -0.5 0.5];        % vector of y-coordinate box shape

%% Simulation Settings

dt = 0.01;                              % (s) Simulation time step
N = 100;                                % (--) Number of time steps 
t = linspace(0,2*pi,N);                 % a vector for marching through a circle
time = linspace(0,dt*N,N);              % the vector for plotting time histories of data

 closure = 1;                           % Specifies closure 1
% closure = 2;                          % Specifies closure 2

%% Object Construction
% My 4 Bar mechanism requires 4 links, so to create them in my code I 
% initialize the link as Link(Link Length, initial orientation, ...
% initial x of base, initial y of base)

% The Link.m class defines all properties belonging to a link. These means
% storing instantaneous positions of the start and end of the vector,
% orientation, overlay information, velocity and acceleration data, etc.
% This will be useful for expansion of code later. Say to store inertia,
% mass, 3D kinematic and dynamic data.

L(1) = Link(Len(1),0,0,0);              % Link object 1 (Side Link left)
L(2) = Link(Len(2),0,0,0);              % Link object 2 (Coupler)
L(3) = Link(Len(3),th3,0,0);          	% Link object 3 (Ground)
L(4) = Link(Len(4),0,0,0);              % Link object 4 (Side Link right)

% Now that my links have been created, I will set overlays as visible, and
% assign the property vis_x_loc and vis_y_loc as the local frame sketch of
% the overlay. This is what will be shown in the final plot so setting
% these properties to two vectors will get you that overlay visual.
for i = 1:4
    L(i).overlay = true;                % Set to true if overlay desired  
    L(i).vis_x_loc = Len(i)*box_x;      % Scale the nondim box shape to the length
    L(i).vis_y_loc = Wid(i)*box_y;      % scale the nondim box shape to the width
end

% Sanity Grashof Criteria. Here is where you will determine the four bar
% type based on the Length vector Len. Once you have the type, you can
% either plot the motion, or error out, and quit the program.

[typeCode,type,warning] = GrashofSanity(Len);

if typeCode == 2 || typeCode == 4       % if its crank-rocker or crank-crank
    driver_index = 1;                   % set the driver index to 1 (V1)
    L(1).angd = 2*pi/(N - 1)/dt;        % set a constant velocity (V1)
    L(1).angdd = 0;                     % and 0 accel. (V1)
elseif typeCode == 3                    % if its Rocker-Rocker
    driver_index = 2;                   % set the driver index to 2 (V2)
    L(2).angd = 2*pi/(N - 1)/dt;        % set a constant velocity (V2)
    L(2).angdd = 0;                     % and 0 accel. (V2)
else
    error('This Four-Bar type was not assigned!')
end

% If the code has not errored out, initialize the mechanism based on type
% found in GrashofSanity.m. mechanisms.m should be able to instantaneously
% solve every mechanism we cover in class. So don't waste this opportunity
% to use it!

FB = mechanisms(type);                  % initialize the four bar mech

% pathPlan.m is a class which can resolve different types of path
% planning structures. There are two types of path planning structures we
% consider in this course, crank resolution and points of interest. For a
% Four Bar mechanism, if we can resolve one revolution of the crank, we
% resolve every possible mechanism closure. If its something like a 3R
% manipulator, we require a point of interest to control. (Get the point of
% interest "here" for example.

% In this case, I create the structure P which is a pathPlan type, give it
% all the links I've created, give it the mechanism, specify the closure
% (There will be two in the case of a Four Bar), and an interest point,
% which for the four bar is just initialized to 0,0 as it is not used. 

% Note - Both Closures will be tested here.

  P = pathPlan(L,FB,'FourBar',closure,0,0);     % startup path planning Closure 1

%% Solution Section
% Here is our main loop.
for i = 1:N
    cla;                % This clears the axes on the current figure
    title('\bfMotion Plot of Four-Bar')
    xlabel('X-Position (m)')
    ylabel('Y-Position (m)')

    % I will set the angle of the driving link here and update the
    % mechanism
    P.links(driver_index).ang = (i - 1)/(N - 1)*2*pi;
    P.update;

    % For all links I will do the following:
    for j = 1:length(L)
        P.links(j).store_data;                              % store all data
        plot(P.links(j).pos_x,P.links(j).pos_y,'k--')       % plot the vector loop
        hold on                                             % This keeps the current plot
        plot(P.links(j).vis_x_glo,P.links(j).vis_y_glo)     % plot the overlays (if any)
        axis equal                                          % sets the axes equal
        grid on                                             % set the grid on
    end
    plot(P.links(1).x_v,P.links(1).y_v)                     % The free joint travel of V1
    plot(P.links(4).x_v,P.links(4).y_v)                     % The free joint travel of V2
    drawnow
    pause(dt)
end

%% Data Plotting
% I will now plot the angular data from the stored vectors

figure(2)       % Angular Displacements of Links 1,2,4
plot(time,P.links(1).ang_v,time,P.links(2).ang_v,time,P.links(4).ang_v)
title('\bfTime History of Angular Displacements')
xlabel('Time (s)')
ylabel('Angular Displacement (rad)')
legend('\theta_1','\theta_2','\theta_4')
grid on

figure(3)       % Angular Velocities of Links 1,2,4
plot(time,P.links(1).angd_v,time,P.links(2).angd_v,time,P.links(4).angd_v)
title('\bfTime History of Angular Velocities')
xlabel('Time (s)')
ylabel('Angular Velocity (rad/s)')
legend('d\theta_1/dt','d\theta_2/dt','d\theta_4/dt')
grid on

figure(4)       % Angular Accelerations of Links 1,2,4
plot(time,P.links(1).angdd_v,time,P.links(2).angdd_v,time,P.links(4).angdd_v)
title('\bfTime History of Angular Accelerations')
xlabel('Time (s)')
ylabel('Angular Acceleration (rad/s^2)')
legend('d^2\theta_1/dt^2','d^2\theta_2/dt^2','d^2\theta_4/dt^2')
grid on