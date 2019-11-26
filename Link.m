classdef Link < handle
    % This class handles link definitions. It may be expanded for 3D links
    properties
        L;                      % (m) Length of Link
        ang;                    % (rad) Orientation of link
        angd;                   % (rad/s) Ang velocity of the link
        angdd;                  % (rad/s^2) Ang acceleration of the link
        x0; y0;                 % (m) Global (X,Y) Position of Origin
        xf; yf;                 % (m) Global (X,Y) Position of End
        overlay;                % (bool) true if visuals for links provided
    end
    properties (Hidden)
        pos_x;                  % (m) vector of instantanious [x0 xf]
        pos_y;                  % (m) vector of instantanious [y0 yf]
        vis_x_loc;              % (m) vector of link visuals x (BFF)
        vis_y_loc;              % (m) vector of link visuals y (BFF)
        vis_x_glo;              % (m) vector of link visuals x (Earth Frame)
        vis_y_glo;              % (m) vector of link visuals y (Earth Frame)
        x_v;                    % (m) vector of xf pos Time history
        y_v;                    % (m) vector of yf pos Time history
        ang_v;                  % (rad) vector of ang pos Time history
        angd_v;                 % (rad/s) vector of ang vel Time history
        angdd_v;                % (rad/s^2) vector of ang acc Time history
    end
    methods
        function c = Link(LENGTH,THETA,X0,Y0) 
            switch nargin
                case 1
                    c.L = LENGTH;           % instantiate length
                case 2
                    c.L = LENGTH;           % instantiate length
                    c.ang = THETA;          % orientation
                case 4
                    c.L = LENGTH;           % instantiate length
                    c.ang = THETA;          % orientation
                    c.x0 = X0;              % origin X
                    c.y0 = Y0;              % origin y 
                    c.update;               % find 2D End Positions 
            end
            c.angd = 0;
            c.angdd = 0;
        end
        function update(c)          % Once instantiated, update if needed
            c.xf = c.x0 + cos(c.ang)*c.L;
            c.yf = c.y0 + sin(c.ang)*c.L;
        end
        function store_data(c)      % add and store an element for each stack of data
            c.x_v = [c.x_v c.xf]; 
            c.y_v = [c.y_v c.yf];
            c.ang_v= [c.ang_v c.ang];
            c.angd_v= [c.angd_v c.angd];
            c.angdd_v= [c.angdd_v c.angdd];
        end
        function construct_pos(c)   % if plotting data, update and fill vectors
            c.update;
            if c.overlay == true    % if overlay desired find global frame of visual
                c.pos_x = [c.x0 c.xf];
                c.pos_y = [c.y0 c.yf];
                c.vis_x_glo = c.x0 + cos(c.ang)*c.vis_x_loc - sin(c.ang)*c.vis_y_loc;
                c.vis_y_glo = c.y0 + sin(c.ang)*c.vis_x_loc + cos(c.ang)*c.vis_y_loc;
                
            else                    % just find the vector start end
                c.pos_x = [c.x0 c.xf];
                c.pos_y = [c.y0 c.yf];
            end
        end
    end
    methods (Hidden)                % 2D Rotation of a vector if necessary
        function [XP,YP] = R(X,Y,ANG)
            XP = cos(ANG)*X - sin(ANG)*Y;
            YP = sin(ANG)*X + cos(ANG)*Y;            
        end
    end
end