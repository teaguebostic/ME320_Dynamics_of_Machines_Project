classdef mechanisms < handle
    % This class is a library containing all solvers for all mechanisms
    % discussed throughout ME 320
    properties
        type;           % (string) Descriptor string of mechanism type
        closure;        % (--) Specified closure
        L_k;ang_k;      % (m,rad) known vector of lengths and orientations
        Ld_k;angd_k;    % (m/s,rad/s) known vector of der. lengths and orientations
        Ldd_k;angdd_k;  % (m/s^2,rad/s^2) known vector of 2nd der. lengths and orientations
    end
    methods
        function c = mechanisms(TYPE,CLOSURE,KNOWN_L,KNOWN_A)
            switch nargin
                case 1
                    c.type = TYPE;              % instantiate type
                case 2
                    c.type = TYPE;              % instantiate type
                    c.closure = CLOSURE;        % closure 
                case 4
                    c.type = TYPE;              % instantiate type
                    c.closure = CLOSURE;        % closure        
                    c.L_k = KNOWN_L;            % vector of known lengths
                    c.ang_k = KNOWN_A;          % vector of known orientation           
            end
        end
        function solvePos(c)
            if strcmp(c.type,'Crank-Rocker') || strcmp(c.type,'Crank-Crank (Drag-Link)')
                % 4 Links, KNOWN LENGTH(S): r1,r2,r3,r4  KNOWN ANGLE(S): th1(t),th3(t)
                %        UNKNOWN LENGTH(S): -          UNKNOWN ANGLE(S): th2(t),th4(t)
                % r1*exp(i*th1) + r2*exp(i*th2) = r3*exp(i*th3) + r4*exp(i*th4)
                
                r1 = c.L_k(1);
                r2 = c.L_k(2);
                r3 = c.L_k(3);
                r4 = c.L_k(4);
                th1 = c.ang_k(1);
                th2 = c.ang_k(2);
                th3 = c.ang_k(3);
                th4 = c.ang_k(4);
                
                A2=2*r2*(r1*cos(th1)-r3*cos(th3));
                B2=2*r2*(r1*sin(th1)-r3*sin(th3));
                C2=(r4^2)-(r3^2)-(r2^2)-(r1^2) + 2*r3*r1*cos(th1-th3);
                th2_1 = 2*atan2(B2 + sqrt((B2^2) + (A2^2) - (C2^2)),A2 + C2);
                th2_2 = 2*atan2(B2 - sqrt((B2^2) + (A2^2) - (C2^2)),A2 + C2);
                
                A4=2*r4*(r3*cos(th3)-r1*cos(th1));
                B4=2*r4*(r3*sin(th3)-r1*sin(th1));
                C4=(r2^2)-(r3^2)-(r1^2)-(r4^2) + 2*r3*r1*cos(th3-th1);
                th4_1 = 2*atan2(B4 + sqrt(B4^2 + A4^2 - C4^2),A4 + C4);
                th4_2 = 2*atan2(B4 - sqrt(B4^2 + A4^2 - C4^2),A4 + C4);
                
                if c.closure == 1
                c.ang_k(4) = th4_1;c.ang_k(2) = th2_2;
                else
                c.ang_k(4) = th4_2;c.ang_k(2) = th2_1;
                end
                
            elseif strcmp(c.type,'Rocker-Rocker')
                % 4 Links, KNOWN LENGTH(S): r1,r2,r3,r4  KNOWN ANGLE(S): th2(t),th3(t)
                %        UNKNOWN LENGTH(S): -          UNKNOWN ANGLE(S): th1(t),th4(t)
                % r1*exp(i*th1) + r2*exp(i*th2) = r3*exp(i*th3) + r4*exp(i*th4)
                
                r1 = c.L_k(1);
                r2 = c.L_k(2);
                r3 = c.L_k(3);
                r4 = c.L_k(4);1
                th1 = c.ang_k(1);
                th2 = c.ang_k(2);
                th3 = c.ang_k(3);
                th4 = c.ang_k(4);
                
                A1=2*r1*(r2*cos(th2)-r3*cos(th3));
                B1=2*r1*(r2*sin(th2)-r3*sin(th3));
                C1=(r4^2)-(r3^2)-(r2^2)-(r1^2)+2*r3*r2*cos(th3-th2);
                th1_1 = 2*atan2(B1 + sqrt(B1^2 + A1^2 - C1^2),A1 + C1);
                th1_2 = 2*atan2(B1 - sqrt(B1^2 + A1^2 - C1^2),A1 + C1);
               
                
                A4=2*r4*((r3*cos(th3))-(r2*cos(th2)));
                B4=2*r4*((r3*sin(th3))-(r2*sin(th2)));
                C4=(r1^2)-(r3^2)-(r2^2)-(r4^2)+(2*r3*r2*cos(th2-th3));
                th4_1 = 2*atan2(B4 + sqrt(B4^2 + A4^2 - C4^2),A4 + C4);
                th4_2 = 2*atan2(B4 - sqrt(B4^2 + A4^2 - C4^2),A4 + C4);
                
                if c.closure == 1
                    c.ang_k(4) = th4_1;c.ang_k(1) = th1_2;
                else
                    c.ang_k(4) = th4_2;c.ang_k(1) = th1_1;
                end
                
            elseif strcmp(c.type,'Crank-Slider1')
                % 3 Links, KNOWN LENGTH(S): r1,r2,r3(t)  KNOWN ANGLE(S): th3(t)
                %        UNKNOWN LENGTH(S): -          UNKNOWN ANGLE(S): th1(t),th2(t)
                % r1*exp(i*th1) + r2*exp(i*th2) = r3*exp(i*th3)
                
                r1 = c.L_k(1);
                r2 = c.L_k(2);
                r3 = c.L_k(3); 
                th3 = c.ang_k(3);

                C1 = (r1^2)+(r3^2)-(r2^2);
                A1 = 2*r1*(r3*cos(th3));
                B1 = 2*r1*(r3*sin(th3));
                th1_1 = 2*atan2(B1 + sqrt((B1^2) + (A1^2) - (C1^2)),A1 + C1);
                th1_2 = 2*atan2(B1 - sqrt((B1^2) + (A1^2) - (C1^2)),A1 + C1);
                
                C2 = (r2^2)+(r3^2)-(r1^2);
                A2 = 2*r2*(r3*cos(th3));
                B2 = 2*r2*(r3*sin(th3));
                th2_1 = 2*atan2(B2 + sqrt((B2^2) + (A2^2) - (C2^2)),A2 + C2);
                th2_2 = 2*atan2(B2 - sqrt((B2^2) + (A2^2) - (C2^2)),A2 + C2);
                
                if c.closure == 1
                    c.ang_k(2) = th2_2;c.ang_k(1) = th1_1;
                else
                    c.ang_k(2) = th2_1;c.ang_k(1) = th1_2;
                end
            end
        end
        function solveVel(c)
            if strcmp(c.type,'Crank-Rocker') || strcmp(c.type,'Crank-Crank (Drag-Link)')
                % 4 Links, KNOWN LENGTH(S): r1,r2,r3,r4  
                %          KNOWN ANGLE(S): th1(t),th2(t),th3(t),th4(t)
                %          KNOWN DIFF ANGLE(S): thd1(t),thd3(t)
                %          UNKNOWN LENGTH(S): -          
                %          UNKNOWN ANGLE(S): -
                %          UNKNOWN DIFF ANGLE(S): thd2(t),thd4(t)
                % i*thd1*r1*exp(i*th1) + i*thd2*r2*exp(i*th2) = i*thd4*r4*exp(i*th4)
                
                r1 = c.L_k(1);
                r2 = c.L_k(2);
                r3 = c.L_k(3);
                r4 = c.L_k(4);
                th1 = c.ang_k(1);
                th2 = c.ang_k(2);
                th3 = c.ang_k(3);
                th4 = c.ang_k(4);
                thd1 = c.angd_k(1);
                thd2 = c.angd_k(1);
                thd3 = c.angd_k(3);
                thd4 = c.angd_k(4);
                
                A = [r1*sin(th1)*thd1;r1*cos(th1)*thd1];
                B = [r4*sin(th4) -r2*sin(th2);r4*cos(th4) -r2*cos(th2)];
                C = inv(B)*A;
                
                thd4 = C(1,:);
                c.angd_k(4) = thd4;
                thd2 = C(2,:);
                c.angd_k(2) = thd2;
                
            elseif strcmp(c.type,'Rocker-Rocker')
                % 4 Links, KNOWN LENGTH(S): r1,r2,r3,r4  
                %          KNOWN ANGLE(S): th1(t),th2(t),th3(t),th4(t)
                %          KNOWN DIFF ANGLE(S): thd2(t),thd3(t)
                %          UNKNOWN LENGTH(S): -          
                %          UNKNOWN ANGLE(S): -
                %          UNKNOWN DIFF ANGLE(S): thd1(t),thd4(t)
                % i*thd1*r1*exp(i*th1) + i*thd2*r2*exp(i*th2) = i*thd4*r4*exp(i*th4)
                
                r1 = c.L_k(1);
                r2 = c.L_k(2);
                r3 = c.L_k(3);
                r4 = c.L_k(4);
                th1 = c.ang_k(1);
                th2 = c.ang_k(2);
                th3 = c.ang_k(3);
                th4 = c.ang_k(4);
                thd1 = c.angd_k(1);
                thd2 = c.angd_k(2);
                thd3 = c.angd_k(3);
                thd4 = c.angd_k(4);
                
                A = [r2*sin(th2)*thd2;r2*cos(th2)*thd2];
                B = [r4*sin(th4) -r1*sin(th1);r4*cos(th4) -r1*cos(th1)];
                C = inv(B)*A;
                
                thd4 = C(1,:);
                c.angd_k(4) = thd4;
                thd1 = C(2,:);
                c.angd_k(1) = thd1;
                
            end
        end
        function solveAcc(c)
            if strcmp(c.type,'Crank-Rocker') || strcmp(c.type,'Crank-Crank (Drag-Link)')
                % 4 Links, KNOWN LENGTH(S): r1,r2,r3,r4  
                %          KNOWN ANGLE(S): th1(t),th2(t),th3(t),th4(t)
                %          KNOWN DIFF ANGLE(S): thd1(t),thd2(t),thd3(t),thd4(t)
                %          KNOWN 2nd DIFF ANGLE(S): thdd1(t),thdd3(t)
                %          UNKNOWN LENGTH(S): -          
                %          UNKNOWN ANGLE(S): -
                %          UNKNOWN DIFF ANGLE(S): -
                %          UNKNOWN 2nd DIFF ANGLE(S): thdd2(t),thdd4(t)
                % (i*thdd1 - thd1^2)*r1*exp(i*th1) + (i*thdd2 - thd2^2)*r2*exp(i*th2) = (i*thdd4 - thd4^2)*r4*exp(i*th4)
                
                r1 = c.L_k(1);
                r2 = c.L_k(2);
                r3 = c.L_k(3);
                r4 = c.L_k(4);
                th1 = c.ang_k(1);
                th2 = c.ang_k(2);
                th3 = c.ang_k(3);
                th4 = c.ang_k(4);
                thd1 = c.angd_k(1);
                thd2 = c.angd_k(2);
                thd3 = c.angd_k(3);
                thd4 = c.angd_k(4);
                thdd1 = c.angdd_k(1);
                thdd2 = c.angdd_k(2);
                thdd3 = c.angdd_k(3);
                thdd4 = c.angdd_k(4);
                
                A = [(r1*thdd1*sin(th1))+(r1*(thd1^2)*cos(th1))+(r2*(thd2^2)*cos(th2))-(r4*(thd4^2)*cos(th4));
                        (r1*thdd1*cos(th1))-(r1*(thd1^2)*sin(th1))-(r2*(thd2^2)*sin(th2))+(r4*(thd4^2)*sin(th4))];
                B = [r4*sin(th4) -r2*sin(th2);r4*cos(th4) -r2*cos(th2)] ;
                C = inv(B)*A;               
                
                thdd4 = C(1,:);
                c.angdd_k(4) = thd4;
                thdd2 = C(2,:);
                c.angdd_k(2) = thd2;
                
            elseif strcmp(c.type,'Rocker-Rocker')
                % 4 Links, KNOWN LENGTH(S): r1,r2,r3,r4  
                %          KNOWN ANGLE(S): th1(t),th2(t),th3(t),th4(t)
                %          KNOWN DIFF ANGLE(S): thd1(t),thd2(t),thd3(t),thd4(t)
                %          KNOWN 2nd DIFF ANGLE(S): thdd2(t),thdd3(t)
                %          UNKNOWN LENGTH(S): -          
                %          UNKNOWN ANGLE(S): -
                %          UNKNOWN DIFF ANGLE(S): -
                %          UNKNOWN 2nd DIFF ANGLE(S): thdd1(t),thdd4(t)
                % (i*thdd1 - thd1^2)*r1*exp(i*th1) + (i*thdd2 - thd2^2)*r2*exp(i*th2) = (i*thdd4 - thd4^2)*r4*exp(i*th4)
                
                r1 = c.L_k(1);
                r2 = c.L_k(2);
                r3 = c.L_k(3);
                r4 = c.L_k(4);
                th1 = c.ang_k(1);
                th2 = c.ang_k(2);
                th3 = c.ang_k(3);
                th4 = c.ang_k(4);
                thd1 = c.angd_k(1);
                thd2 = c.angd_k(2);
                thd3 = c.angd_k(3);
                thd4 = c.angd_k(4);
                thdd1 = c.angdd_k(1);
                thdd2 = c.angdd_k(2);
                thdd3 = c.angdd_k(3);
                thdd4 = c.angdd_k(4);
               
                A = [(r2*thdd2*sin(th2))+(r2*(thd2^2)*cos(th2))+(r1*(thd1^2)*cos(th1))-(r4*(thd4^2)*cos(th4));
                        (r2*thdd2*cos(th2))-(r2*(thd2^2)*sin(th2))-(r1*(thd1^2)*sin(th1))+(r4*(thd4^2)*sin(th4))];
                B = [r4*sin(th4) -r1*sin(th1);r4*cos(th4) -r1*cos(th1)];
                C = inv(B)*A;
                
                thdd4 = C(1,:);
                c.angdd_k(4) = thd4;
                thdd1 = C(2,:);
                c.angdd_k(1) = thd1;
                
            end
        end
    end
    methods (Static)
        function [SOL1,SOL2] = HALFTAN(A,B,C)   % half tangent law solutions
            SOL1 = 2*atan2(B + sqrt(B^2 + A^2 - C^2),A + C);
            SOL2 = 2*atan2(B - sqrt(B^2 + A^2 - C^2),A + C);
        end
        function RAD_L = LIMITRANGE(RAD)        % Keeps pos sols in [0,2*pi]
            while RAD < 0
                RAD = RAD + 2*pi;
            end
            while RAD > 2*pi
                RAD = RAD - 2*pi;
            end     
            RAD_L = RAD;
        end
    end
end