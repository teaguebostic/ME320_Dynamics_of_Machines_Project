%% ME 320-1001 S18 Project 1 main script (Four Bar)
% This is the solution for Project 1 assigned in the Spring Semester of
% 2018 for ME 320-1001. In this project, students were asked to complete
% the skeleton code based on this project. 3R Manipulator is resolved here

clc;clear;close all

%% Parameter Settings
% In this portion of the code, Link length definitions are prescribed,
% along with x and y overlay information (visual data)

Len = [1 .8 0.2];                       % Vector of link lengths

% I chose a simple box here, where link width is specified and percentages
% of both Len and Wid are used for box_x and box_y.

Wid = [.1 .1 .1];                       % (m) Vector of link widths
box_x = [-.02 1.02 1.02 -.02 -.02];     % (%) vector of x-coordinate box shape 
box_y = [0.5 0.5 -0.5 -0.5 0.5];        % (%) vector of y-coordinate box shape

%% Simulation Settings
% This portion of the code defines simulation settings such as run time,
% number of solutions, and path planning defs.

dt = 0.01;                              % (s) time step
N = 100;                                % (-) number of solutions
t = linspace(0,2*pi,N);                 % (-) angle step for a circle
r = 0.5;                                % (m) path planning
x0 = 0.5;                               % (m) path planning x0
y0 = 0.25;                              % (m) path planning y0
XR = r*cos(t).*sin(2*t) + x0;           % (m) Desired end effector Position X
YR = r*sin(t).*cos(4*t) + y0;           % (m) Desired end-effector Position Y

 closure = 1;                            % Closure 1 tested
% closure = 2;                          % Closure 2 tested

%% Object Construction
% My 3R mechanism requires 3 links, so create to them in my code I 
% initialize the link as Link(Link Length, initial orientation, initial x 
% of base, initial y of base)

% The Link.m class defines all properties belonging to a link. This means
% storing instantaneous positions of the start and end of the vector,
% orientation, overlay information, velocity and acceleration data, etc.
% This will be useful for expansion of code later. Say to store inertia,
% mass, 3D kinematic and dynamic data.

L(1) = Link(Len(1),0,0,0);              % Link object 1
L(2) = Link(Len(2),0,0,0);              % Link object 2
L(3) = Link(Len(3),0,0,0);              % Link object 3 (end-effector)

% Now that my links have been created, I will set overlays as visible, and
% assign the property vis_x_loc and vis_y_loc as the local frame sketch of
% the overlay. This is what will be shown in the final plot so setting
% these properties to two vectors will get you that overlay visual.

for i = 1:3
    L(i).overlay = false;
    L(i).vis_x_loc = Len(i)*box_x;
    L(i).vis_y_loc = Wid(i)*box_y;
end

% mechanisms.m should be able to instantaneously solve every mechanism we 
% cover in class. So don't waste this opportunity to use it!

% We are telling our code that we want to solve a crank-Slider mechanism
CS = mechanisms('Crank-Slider1');       

% pathPlan.m is a class which can resolve different types of path
% planning structures. There are two types of path planning structures we
% consider in this course, crank resolution and points of interest. For a
% 3R manipulator we would like to control the end effector position so this
% is a point of interest type of problem

% In this case, I create the structure P which is a pathPlan type, give it
% all the links I've created, give it the mechanism, specify the closure
% (There will be two in this case), and an interest point

% Note - Both Closures will be tested here.

P = pathPlan(L,CS,'3R',closure,0,0);    % startup path planning Closure

%% Solution Section
% Here is our main loop.
for i = 1:N
    cla;                % This clears the axes on the current figure
    title('\bfMotion Plot of Four-Bar')
    xlabel('X-Position (m)')
    ylabel('Y-Position (m)')
    
    % Since this is a 3R I cannot resolve a unique solution without setting
    % one link angle. I choose the final link 3 to spin in a circle
    P.links(3).ang = (i - 1)/(N - 1)*2*pi;
    P.xr = XR(i);               % I then set the x position of the end effector
    P.yr = YR(i);               % I then set the y position of the end effector
    P.update;                   % Solve Kinematics at this time step
    
    % I then just want to store the position of the end effector
    P.links(3).store_data;
    
    % For all links I will do the following:
    for j = 1:3
        plot(P.links(j).pos_x,P.links(j).pos_y,'k--')       % plot the vector loop
        hold on                                             % This keeps the current plot
        plot(P.links(j).vis_x_glo,P.links(j).vis_y_glo)     % plot the overlays (if any)
        axis([0 1.1 -.35 1.1])                              % sets the axes 
        axis equal
        grid on                                             % set the grid on
    end
    plot(P.links(3).x_v,P.links(3).y_v)                     % The free joint travel of the end effector
    drawnow
    pause(dt)
end