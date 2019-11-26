function [typeCode,type,warning] = GrashofSanity(Len)
% This function returns the case conclusion for Grashof and NonGrashof four
% bar mechanisms. 

S = min(Len);                       % find the shortest link length
L = max(Len);                       % find the longest link length
S_index = find(Len == S);           % find the index of the shortest link
L_index = find(Len == L);           % find the index of the longest link

% typeCode, type and warning default vals
typeCode = -1;
type = [];
warning = [];

% if it is change point by all equal length links asssign type
if length(S_index) == 4 || (length(S_index) == 2 && length(L_index) == 2)                      
    typeCode = 0; 
    type = 'Change Point'; 
    warning = 'All inversions are the same. This type is not assigned.';
    % if it is Non-Grashof by any repeated L's or S's assign type
elseif length(S_index) > 1
    typeCode = 1;
    type = 'Non-Grashof';
    warning = 'Non-predictable behavior. This type is not assigned.';
else   
    % The mechanism can be either Grashof, Non-Grashof or Changepoint but 
    % it is ruled out that any links have repeated lengths, otherwise this 
    % never gets seen..
    if length(L_index) > 1                   % find the indices of the other links
        Grashof = 1;
        GrashofCP = 0;
    else
        PQ_index = find(Len ~= S & Len ~= L);  
        P = Len(PQ_index(1));
        Q = Len(PQ_index(2));
        Grashof = S + L - (P + Q) < 0;       % Test for Grashof Criteria
        GrashofCP = S + L - (P + Q) == 0;    % Test for Grashof Criteria Change Point
    end
    if Grashof == 1
        switch S_index     
            case 1                           % crank is the shortest link
                typeCode = 2;
                type = 'Crank-Rocker';
            case 2                           % coupler is the shortest link
                typeCode = 3;
                type = 'Rocker-Rocker';
                warning = 'Actuator should be on the coupler';
            case 3                           % ground is the shortest link
                typeCode = 4;
                type = 'Crank-Crank (Drag-Link)';
        end    
    elseif GrashofCP == 1
        typeCode = 0;
        type = 'Change Point';
        warning = 'All inversions are the same. This type is not assigned.';
    else
        typeCode = 1;
        type = 'Non-Grashof';
        warning = 'Non-predictable behavior. This type is not assigned.';
    end
end

% Print to command window findings
if isempty(warning)
    fprintf(['This is a ',type,' Four Bar Mechanism.\n']);
else
    fprintf(['This is a ',type,' Four Bar Mechanism. ',warning,'\n']);
end

