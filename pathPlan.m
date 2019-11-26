classdef pathPlan < handle
    % This class resolves path planning for various mechanisms and utilizes
    % the Link.m and mechanisms.m class structures
    properties
        xr;yr;              % (m,m) desired position of interest point 
        closure;            % (string) determines closure
        mechtype;           % (string) descriptor of mechanism type
    end
    properties (Hidden)
        links;              % link object (Links.m)
        mechanism;          % mechanism object (mechanisms.m)
    end
    methods
        function c = pathPlan(LINKS,MECHANISM,MECHTYPE,CLOSURE,XR,YR)    
            c.links = LINKS;                % instantiate links
            c.mechanism = MECHANISM;        % any mechanisms
            c.mechtype = MECHTYPE;          % mech type
            c.closure = CLOSURE;            % closure
            c.xr = XR;                      % current desired x position
            c.yr = YR;                      % current desired y position
            
            % override mech closure to pathPlan closure
            c.mechanism.closure = c.closure;
            c.update                        % update path
        end
        function update(c)
            if strcmp(c.mechtype,'FourBar')
                % This resolves Crank-Crank, Crank-Rocker, and
                % Rocker-Rocker 4-Bar Mechs. If it is Crank-Crank or
                % Crank-Rocker, set the driving link to the left hand side.
                % If its on the right invert it.
                
                % fill the necessary mechanism POSITION info 
                c.mechanism.L_k = [c.links(1).L c.links(2).L c.links(3).L c.links(4).L];
                c.mechanism.ang_k = [c.links(1).ang c.links(2).ang c.links(3).ang c.links(4).ang];
                % solve mech POSITION
                c.mechanism.solvePos;           
                % Update link POSITION info 
                typecode = GrashofSanity(c.mechanism.L_k);
                if typecode == 2 || typecode == 4
                    c.links(1).construct_pos;
                    c.links(2).x0 = c.links(1).xf;
                    c.links(2).y0 = c.links(1).yf;
                    c.links(2).ang = c.mechanism.ang_k(2);
                    c.links(2).construct_pos;
                    c.links(3).construct_pos;
                    c.links(4).x0 = c.links(3).xf;
                    c.links(4).y0 = c.links(3).yf;
                    c.links(4).ang = c.mechanism.ang_k(4);
                    c.links(4).construct_pos;
                elseif typecode == 3
                    c.links(1).construct_pos;
                    c.links(1).ang = c.mechanism.ang_k(1);
                    c.links(2).x0 = c.links(1).xf;
                    c.links(2).y0 = c.links(1).yf;
                    c.links(2).construct_pos;
                    c.links(3).construct_pos;
                    c.links(4).x0 = c.links(3).xf;
                    c.links(4).y0 = c.links(3).yf;
                    c.links(4).ang = c.mechanism.ang_k(4);
                    c.links(4).construct_pos;
                end
                % fill the necessary mechanism VELOCITY info
                c.mechanism.angd_k = [c.links(1).angd c.links(2).angd c.links(3).angd c.links(4).angd];            
                % solve mech VELOCITY
                c.mechanism.solveVel;           
                % Update link VELOCITY info
                c.links(1).angd = c.mechanism.angd_k(1); 
                c.links(2).angd = c.mechanism.angd_k(2);
                c.links(3).angd = c.mechanism.angd_k(3);
                c.links(4).angd = c.mechanism.angd_k(4);
                % fill the necessary mechanism ACCELERATION info     
                c.mechanism.angdd_k = [c.links(1).angdd c.links(2).angdd c.links(3).angdd c.links(4).angdd];
                % solve mech ACCELERATION
                c.mechanism.solveAcc;            
                % Update link ACCELERATION info
                c.links(1).angdd = c.mechanism.angdd_k(1); 
                c.links(2).angdd = c.mechanism.angdd_k(2);
                c.links(3).angdd = c.mechanism.angdd_k(3);
                c.links(4).angdd = c.mechanism.angdd_k(4); 
                % build the links
            elseif strcmp(c.mechtype,'3R')
                % determine the length and orientation of crank slider
                % based on given 3R 3rd link info. That is, in the 3R
                % config, we assume th3 and r3 are given, this
                % is because lengths are all given and orientation is the
                % most useful if assigned to end effector. So we solve the
                % 3 vector loop of links 1 and 2 with origin.
                
                % Position of link 2 end point
                x = c.xr - (c.links(3).L*cos(c.links(3).ang));
                y = c.yr - (c.links(3).L*sin(c.links(3).ang));
                R = ((x^2)+(y^2))^0.5;
                ang = atan2(y,x);
                % length and orientation of vector (orig,link 2 end)
                % then fill the necessary mechanism info
                c.mechanism.L_k = [c.links(1).L c.links(2).L R];
                c.mechanism.ang_k = [c.links(1).ang c.links(2).ang ang];              
                % solve mech
                c.mechanism.solvePos;               
                % set link info
                
                c.links(1).ang = c.mechanism.ang_k(1);
                c.links(1).construct_pos;

                c.links(2).x0 = c.links(1).xf;
                c.links(2).y0 = c.links(1).yf;
                c.links(2).ang = c.mechanism.ang_k(2);
                c.links(2).construct_pos;
                
                c.links(3).x0 = c.links(2).xf;
                c.links(3).y0 = c.links(2).yf;
                %c.links(3).ang = c.mechanism.ang_k(3);
                c.links(3).construct_pos;
                
                % build the links
             
            else
                fprintf('The MECHANISM assigned has not been built in this solver.\n')
            end
        end
    end
end