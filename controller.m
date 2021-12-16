function [ui,persistentMemory] = controller(missionData,persistentMemory,currentTime)
% Decentralized controller for multi-robot search and rescue mission
%
% Inputs:
%   +missionData struct containining the following fields:
%       -Submission     : current submission ('init', 'navigate', 'search')
%       -SensorData     : 8-by-1 vector of distance to objects in sensor range in the 0,45,...,315 degree directions
%       -AgentState     : 2-by-1 position of current agent
%       -AgentID        : current agent id
%       -NeighborState  : 2-by-|Ni| matrix of visible neighbor states
%       -NeighborIDs    : 1-by-|Ni| vector of neighbor id's
%       -MissionInfo    : Struct containing mission info (e.g., target)
%       -MinSafetyDist  : Minimum safety separation
%       -MaxSensCommRng : Maximum sensing/comm distance
%   +persistentMemory   : 1-by-|Ni| vector of auxiliary flags
%   +currentTime        : current mission time
% Outputs:
%   +ui                 : 2-by-1 velocity reference for current agent
%   +persistentMemory   : 1-by-|Ni| vector of persistent memory scalars per neighbor

% Extracting variable info
currentMission = missionData.Submission;
Delta = missionData.MaxSensCommRng;
delta = missionData.MinSafetyDist;
% Agent and neighbor states and IDs
xi = missionData.AgentState;
xj = missionData.NeighborStates;
ID = missionData.AgentID;
nIDs = missionData.NeighborIDs;
numNeighbors = size(xj,2);
% Get sensor data
sensorData = missionData.SensorData;
numSensors = length(sensorData);
sensorTheta = 0:2*pi/numSensors:2*pi*(1-1/numSensors);
sensorDir = [cos(sensorTheta);sin(sensorTheta)];
numAgents = 5;

% Initialize control
%%%%%%%%%%%%%%%%%%%%%%%%%%%% Modify this block %%%%%%%%%%%%%%%%%%%%%%%%%%%%
auxFlags = persistentMemory;
ui = [0;0];
switch currentMission
    case 'init'
        %% edge-tension design
        for jj = 1:size(xj,2)
            nij = norm (xj(:,jj) - xi);
            if auxFlags(jj) == 0 && nij < 0.99*Delta
                auxFlags(jj) = 1;
            end
            wij = 1e-3*auxFlags(jj) * (nij^2 - Delta*delta)/...
                ((Delta-nij)^3*(nij-delta)^3);
            ui = ui + wij*(xj(:,jj) - xi);
        end
    case 'navigate'
        leaderAgent = ~isempty(missionData.MissionInfo);
        %% line graph adjacency matrix
        A_formation = [0 1 0 0 0; 1 0 1 0 0; 0 1 0 1 0; 0 0 1 0 1;0 0 0 1 0];
        
        % Find neighbors to maintain connectivity with
        nids_from_adj = find(A_formation(ID,:));
        indices = [];
        for nid = nids_from_adj
            indices = [indices, find(nIDs==nid)];
        end
        inpDir = [0;0];
        ui = [0;0];
        for jj = 1:size(xj,2)
            nij = norm(xj(:,jj) - xi);
            %% edge tension design for the two cases
            if ismember(jj,indices) 
                
                wij = 0.01*((2*(Delta-nij)*(nij-delta) + (2*nij-delta-Delta)*nij)/((Delta-nij)^2 * (nij-delta)^2));
                ui = ui + wij*(xj(:,jj) - xi);
            else
                
                wij =0.3*((nij-2*delta)/((nij-delta)^2));
                ui = ui + wij*(xj(:,jj) - xi);
            end
        end
        %% avoiding obstacles
        for jj=1:size(sensorData)
            d = sensorData(jj);
            if d<10
                inpDir = sensorDir(:,jj);
                    ui = ui - (1e-3/(d-delta)^2)*(inpDir);
            end
        end
        %% specifying target location for the leader by another velocity term
        if leaderAgent
            xt = missionData.MissionInfo.target;
            ui = ui+3e0*(xt-xi)/max(norm(xt-xi),delta);
        end
    case 'search'
        % Voronoi tesselation
        beacons = missionData.MissionInfo.domain;
        vcell = missionData.MissionInfo.AgentVCell;
        Lowx = min(beacons(1,:));
        Lowy = min(beacons(2,:));
        Uppx = max(beacons(1,:));
        Uppy = max(beacons(2,:));
        xgrid = linspace(Lowx,Uppx);
        ygrid = linspace(Lowy,Uppy);
        dx = xgrid(2)-xgrid(1);
        dy = ygrid(2)-ygrid(1);
        dA = dx*dy;
        [xmesh,ymesh] = meshgrid(xgrid,ygrid);
        qx = xmesh(:); qy = ymesh(:);
        %density function
        phi = exp(-((qx-2*cos(currentTime/10)).^2+(qy-2*sin(currentTime/10)).^2));
        % check the membership
        inpoly = inpolygon(qx,qy,vcell(1,[1:end,1]),vcell(2,[1:end,1]));
        int1 = qx(inpoly).*phi(inpoly);
        int2 = qy(inpoly).*phi(inpoly);
        integral1 = sum(int1)*dA;
        integral2 = sum(int2)*dA;
        m = sum(phi(inpoly))*dA; %mass
        c = [integral1;integral2]/m; %center of mass
        %% edge-tension design
        for jj = 1:size(xj,2)
            nij = norm (xj(:,jj) - xi);
            if auxFlags(jj) == 0 && nij < 0.99*Delta
                auxFlags(jj) = 1;
            end
            wij = 1e-3*auxFlags(jj) * (nij^2 - Delta*delta)/...
                ((Delta-nij)^3*(nij-delta)^3);
            ui = ui + wij*(xj(:,jj) - xi);
        end
        ui = ui+(c-xi); %Lloyds

end
%%%%%%%%%%%%%%%%%%%%%%%%%%%% Modify this block %%%%%%%%%%%%%%%%%%%%%%%%%%%%
end
