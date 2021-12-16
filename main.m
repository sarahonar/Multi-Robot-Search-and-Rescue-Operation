%% ENME808T Final Project: Multi-Robot Search and Rescue
% Main file for multi-robot search and rescue mission.
%
% Copyright (c) 2019
% Collaborative Controls and Robotics Laboratory
% University of Maryland, College Park
%
% All rights reserved.
%% Define mission parameters
% NOTE: You must run the file init.m at least once before this script!
% Set this flag to false when running the code on the Robotarium

simulate_true = true;
% Select the mission scenario
missionChoice = 'full'; %'init','navigate','search', or 'full'
switch missionChoice
    case {'init','full'}
        currentSubmission = 'init';
    case 'navigate'
        currentSubmission = 'navigate';
    case 'search'
        currentSubmission = 'search';
end
% Load the initial configuration
[numAgents,initPoses,Delta] = loadMission(currentSubmission);
% Vector of agent IDs
IDs = 1:numAgents;
%% Get Robotarium object used to communicate with the robots/simulator
r = Robotarium('NumberOfRobots', numAgents, 'ShowFigure', true, 'InitialConditions', initPoses);
% Safety distance
delta = 1.25*r.robot_diameter; % Robot diameter plus a buffer
% Domain Boundaries
domBoundary = r.boundaries;
% Max velocities
vmax = 0.5*r.max_linear_velocity;
% Max velocities
wmax = 0.5*r.max_angular_velocity;
%% Initial Connectivity
% Retrieve the most recent poses from the Robotarium.
x = r.get_poses(); r.step();
% Rename for convenience
robotPositions = x(1:2,:);
robotHeadings = x(3,:);
% Get initial connectivity
[A,pairwiseDist] = deltaDisk(robotPositions,Delta);
edgeIndices = getEdgeIndices(A);
edgeCoordinatesX = [robotPositions(1,edgeIndices(:,1));robotPositions(1,edgeIndices(:,2));nan(1,size(edgeIndices,1))];
edgeCoordinatesY = [robotPositions(2,edgeIndices(:,1));robotPositions(2,edgeIndices(:,2));nan(1,size(edgeIndices,1))];
%% Visualization Elements
% Get Robotarium figure handle
hFig = r.figure_handle;
if simulate_true
    % Get current children graphic handles
    hChildren = get(gca,'Children');
    % Get robot handles
    hRobots = hChildren(1:numAgents);
    % Get boundary handle
    hBoundary = hChildren(end);
end
% Plot connectivity edges
hEdges = plot(edgeCoordinatesX(:),edgeCoordinatesY(:),'LineWidth',2);
% Load the background image
[hIm,imX,imY,imPos] = loadBackgroundImage('missionMap.jpg',domBoundary);
% Load the target search area
[hTarget,xt] = getTargetLocation(imPos);
% Load search area beacons
[hBeacons,beacons] = getSearchAreaBeacons(imPos);
% Load obstacle data
plot_obstacles = false;
[obstacleData,hObs] = getObstacleData(plot_obstacles,imPos);
% Transform from cell to matrix form
obstacleDataMat = getObstacleDataMat(obstacleData);
if simulate_true
    % Make sure the robot patches are at the top of the graphic stack
    uistack(hRobots,'top')
end
%% Construct mission specific info
missionInfo = cell(numAgents,1);
hVor = gobjects(numAgents,1);
switch missionChoice
    case {'init','full'}
        for ii = 1:numAgents
            missionInfo{ii} = [];
        end
    case 'navigate'
        for ii = 1:numAgents
            if ii<numAgents
                missionInfo{ii} = [];
            else
                missionInfo{ii}.target = xt;
            end
        end
    case 'search'
        % Translate environment to target search area
        imPos = imPos - xt;
        beacons = beacons - xt;
        [obstacleData(1:end)] = cellfun(@(x)bsxfun(@minus,x,xt),obstacleData,'UniformOutput',false);
        xt = xt - xt;
        % Store beacon locations
        for ii = 1:numAgents
            missionInfo{ii}.domain = beacons;
        end
end
% Get random marker locations
markers = getSearchMarkers([beacons(1,[1 2]),beacons(2,[2 3])],0.5*Delta,numAgents);
activeMarkers = false(1,numAgents);
% Plot the markers
hMarkers = plot([markers(1,:);nan(size(markers(1,:)))],[markers(2,:);nan(size(markers(2,:)))],'p','MarkerSize',20,'MarkerFaceColor','y','Visible','off');
% [hMarkers(:).Visible] = deal('off');
%% Main loop
% Number of iterations
iterations = 1e6;
persMem = zeros(numAgents);
dxu = zeros(2,numAgents);
ell = 0.5*delta;
missionTimer = tic;
if simulate_true
    dt = r.time_step;
else
    previousTime = toc(missionTimer);
    dt = previousTime;
end
for k = 1:iterations
    % Retrieve the most recent poses from the Robotarium.
    x = r.get_poses();
    robotPositions = x(1:2,:);
    robotHeadings = x(3,:);
    % Get a new time reading
    currentTime = toc(missionTimer);
    % Compute the vector field for scenary motion
    switch currentSubmission
        case 'init'
            vfield = zeros(2,1);
        case 'navigate'
            vfield = getVelocityField(robotPositions,domBoundary,2*vmax);
            vfield = vfield - 0.5*vmax*xt/max(norm(xt),delta);
        case 'search'
            vfield = -0.5*vmax*xt/max(norm(xt),delta);
            [vcells,ADelaunay] = boundedVoronoi(robotPositions,beacons);
    end
    % Transition scenary and update graphics
    if ~simulate_true
        dt = currentTime - previousTime;
        previousTime = currentTime;
    end
    [xt,imPos,beacons,markers,obstacleData{:}] = ...
        transitionScenery(r.time_step,vfield,xt,imPos,beacons,markers,obstacleData{:});
    obstacleDataMat = getObstacleDataMat(obstacleData);
    set(hIm,'XData',imX+imPos(1),'YData',imY+imPos(2));
    set(hTarget,'XData',[robotPositions(1,end) xt(1)] ,'YData',[robotPositions(2,end) xt(2)]);
    set(hBeacons,'XData',beacons(1,[1:end 1]),'YData',beacons(2,[1:end 1]));
    if ~isempty(hObs)
        for kk = 1:length(hObs)
            set(hObs(kk),'XData',obstacleData{kk}(1,:),'YData',obstacleData{kk}(2,:));
        end
    end
    % Update Delta-disk graph topology
    [ADeltaDisk,pairwiseDist] = deltaDisk(robotPositions,Delta);
    % Get the sensor data for each robot along the 0,45,90,...,315 deg dir
    sensorData = rangeSensor(robotPositions,obstacleDataMat,delta,Delta);
    % Check for collisions
    collisionDetected = checkCollisions(robotPositions,obstacleDataMat,r.robot_diameter);
    if any(pairwiseDist<r.robot_diameter) || collisionDetected
        disp('Collision detected!')
        hText = text(0,0,'Collision Detected!','FontSize',28,'FontWeight','bold','HorizontalAlignment','center','Color',[1 0.25 0.25]);
        pause(5)
        break
    end
    switch currentSubmission
        case 'init'
            A = ADeltaDisk;
        case 'navigate'
            A = ADeltaDisk;
        case 'search'
            A = ADelaunay;
    end
    % Get the edges to be plotted
    edgeIndices = getEdgeIndices(A);
    edgeCoordinatesX = [robotPositions(1,edgeIndices(:,1));robotPositions(1,edgeIndices(:,2));nan(1,size(edgeIndices,1))];
    edgeCoordinatesY = [robotPositions(2,edgeIndices(:,1));robotPositions(2,edgeIndices(:,2));nan(1,size(edgeIndices,1))];
    set(hEdges,'XData',edgeCoordinatesX(:),'YData',edgeCoordinatesY(:))
    
    % Compute the control at the node-level
    u = zeros(2,numAgents);
    for ii = 1:numAgents
        switch currentSubmission
            case 'init'
                missionInfo{ii} = [];
            case 'navigate'
                if ii>=numAgents
                    missionInfo{ii}.target = xt;
                else
                    missionInfo{ii} = [];
                end
            case 'search'
                missionInfo{ii}.domain = beacons;
                missionInfo{ii}.AgentVCell = vcells{ii};
                missionInfo{ii}.NeighborVCells = vcells(ADelaunay(ii,:));
        end
        % Store pertinent data
        missionData = struct(...
            'Submission',currentSubmission,...
            'SensorData',sensorData(:,ii),...
            'AgentState',robotPositions(:,ii),...
            'AgentID',ii,...
            'NeighborStates',robotPositions(:,A(ii,:)),...
            'NeighborIDs',IDs(A(ii,:)),...
            'MissionInfo',missionInfo{ii},...
            'MinSafetyDist',delta,...
            'MaxSensCommRng',Delta);
        %%%%%%%%%%%%%%% Modify this controller %%%%%%%%%%%%%%%%%%%%%%%%%%%%
        [u(:,ii),persMem(ii,A(ii,:))] = controller(missionData,persMem(ii,A(ii,:)),currentTime);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Saturate controllers to avoid surpassing actuator limits
        if norm(u(:,ii))> vmax
            u(:,ii) = vmax*(u(:,ii)/norm(u(:,ii)));
        end
        % Compensate for scenary transitions
        u(:,ii) = u(:,ii) + vfield;
        % Compute the unicycle control
        theta_i = robotHeadings(ii);
        dxu(:,ii) = diag([1,1/ell])*[cos(theta_i),sin(theta_i);-sin(theta_i),cos(theta_i)]*u(:,ii);
        % Ensure actuator limits are met
        dxu(1,ii) = max(min(dxu(1,ii),vmax),-vmax);
        dxu(2,ii) = max(min(dxu(2,ii),wmax),-wmax);
    end
    % Check if submission is accomplished
    switch currentSubmission
        case 'init'
            if sum(A(:)) == numAgents*(numAgents-1)
                disp('Succesfully initialized!')
                if strcmp(missionChoice,'full')
                    set(hTarget,'Visible','on')
                    currentSubmission = 'navigate';
                else
                    break
                end
            end
        case 'navigate'
            if all(inpolygon(robotPositions(1,:).',robotPositions(2,:).',beacons(1,[1:end 1]).',beacons(2,[1:end 1]).'))
                disp('Succesfully navigated to search area!')
                if strcmp(missionChoice,'full')
                    set(hTarget,'Visible','off')
                    set(hBeacons,'Visible','on')
                    currentSubmission = 'search';
                else
                    break
                end
            end
        case 'search'
            % Get distance to markers
            markerDiffX = markers(1,:)-robotPositions(1,:).';
            markerDiffY = markers(2,:)-robotPositions(2,:).';
            markerDist = sqrt(markerDiffX.^2 + markerDiffY.^2);
            % Activate any marker within the activation range
            activeMarkers(any(markerDist<1.5*delta)) = true;
            for ii = 1:size(activeMarkers,2)
                if activeMarkers(ii)
                    set(hMarkers(ii),'Visible','on','XData',markers(1,ii),'YData',markers(2,ii));
                end
            end
            if all(activeMarkers)
                hText = text(0,0,'Mission Success!','FontSize',28,'FontWeight','bold','HorizontalAlignment','center','Color',[0.25 1 0.5]);
                disp('Succesfully found all markers!')
                pause(5)
                break
            end
    end
    %% Send velocities to agents
    % Set velocities of agents 1,...,N
    r.set_velocities(1:numAgents, dxu);
    % Send the velocities to the agents and update the back-end.
    r.step();
end
% Checks that the simulation passes the safety tests!
r.debug();
% Store data if not a simulation
if ~simulate_true
    file_name = unique_filename('ENME808T_FinalProject');
    save(file_name);
end
%% Auxiliary Functions
% Obtain delta-disk graph adjacency information
function [A,pairwiseDist] = deltaDisk(robotStates,Delta)
pairwiseDist = pdist(robotStates.');
A = squareform(pairwiseDist)<Delta;
A = (A - diag(diag(A)))~=0;
end
% Get edge indices for plotting
function [edgeIndices] = getEdgeIndices(A)
numAgents = size(A,1);
numEdges = sum(A(:))/2;
edgeIndices = zeros(numEdges,2);
edgeCounter = 1;
for ii = 1:numAgents-1
    for jj = ii+1:numAgents
        if A(ii,jj) == 1
            edgeIndices(edgeCounter,1) = ii;
            edgeIndices(edgeCounter,2) = jj;
            edgeCounter = edgeCounter + 1;
        end
    end
end
end
% Choose between hardcoded scenarios
function [numAgents,initPoses,Delta] = loadMission(missionChoice)
numAgents=5;
% Connectivity distance
Delta = 0.8;
initPoses = zeros(3,numAgents);
switch missionChoice
    case 'init'
        initPoses(1,:) = [-0.3918, 0.1517,-1.2231,-0.7686, 0.4381];
        initPoses(2,:) = [-0.2063, 0.2767,-0.0028, 0.4649, -0.2044];
        initPoses(3,:) = [ 2.6698, 4.6771, 5.6945, 0.5270, 2.1381];
    case 'navigate'
        initPoses(1,:) = [-0.1572,-0.2824,-0.3822,-0.4608,-0.0943];
        initPoses(2,:) = [ 0.3054,-0.0363, 0.3170, 0.1092, 0.0844];
        initPoses(3,:) = [ 0.4936,-1.9419, 2.2003,-2.7471, 0.1442];
    case 'search'
        initPoses(1,:) = [-0.4406,-0.5991,-0.3747,-0.1844, 0.0144];
        initPoses(2,:) = [-0.3795,-0.1822, 0.0430,-0.2362, 0.0059];
        initPoses(3,:) = [ 0.4045,-2.2849,-0.1914,-2.1214,-0.6349];
    otherwise
        error('Mission Choices are ''init'', ''navigate'', ''search'', or ''full''')
end
end

% Transition between scenarios
function [vfield] = getVelocityField(robotPositions,domBoundary,maxSpeed)
[~,farthestRobot] = max(vecnorm(robotPositions));
vfield = -robotPositions(:,farthestRobot)/domBoundary(4);
vfield = maxSpeed/(1.5*domBoundary(2)-norm(vfield))^6*vfield/norm(vfield);
end

function [hIm,imX,imY,imPos] = loadBackgroundImage(imName,domBoundary)
backgroundIm = imread(imName);
backgroundIm = flipud(backgroundIm);
imSize = size(backgroundIm);
imScaleRatio = 3;
backgroundIm = backgroundIm(1:imSize(1)-mod(imSize(1),imScaleRatio),1:imSize(2)-mod(imSize(2),imScaleRatio),:);
imSize = size(backgroundIm);
imX = linspace(domBoundary(1),imScaleRatio*diff(domBoundary(1:2))+domBoundary(1),imSize(2));
imY = linspace(domBoundary(3),imScaleRatio*diff(domBoundary(3:4))+domBoundary(3),imSize(1));
imPos = [imX(1)+diff(imX([1 end]))*1.5/10;imY(1)+diff(imY([1 end]))*0.5/20];
imIndx = 1:imSize(1);%/imScaleRatio;
hIm = image(imX(imIndx)+imPos(1),imY(imIndx)+imPos(2),backgroundIm,'CDataMapping', 'scaled');
uistack(hIm,'bottom')
end
% Returns centroid of search domain
function [hTar,xt] = getTargetLocation(imPos)
shiftInPos = imPos(:) + 0.64*[1;1];
% Define the target location
imPosShack = [4.177;3.275];%+[imX(1)+diff(imX([1 end]))*13/22,imY(1)+diff(imY([1 end]))*31/44];
imPosShack = imPosShack + shiftInPos;
xt = imPosShack;
hTar = plot(xt(1),xt(2),'--og','LineWidth',2,'Visible','off');
end
% Returns search domain
function [hBeacons,beacons] = getSearchAreaBeacons(imPos)
shiftInPos = imPos(:) + 0.64*[1;1];
% Define the beacons
beacons = [3.0968,5.2752,5.2752,3.0968;
    2.7254,2.7254,3.8140,3.8140];
beacons(1,:) = beacons(1,:) + shiftInPos(1);
beacons(2,:) = beacons(2,:) + shiftInPos(2);
hBeacons = plot(beacons(1,:),beacons(2,:),'-h','MarkerFaceColor',[0.8500 0.3250 0.0980],'MarkerEdgeColor',[0.8500 0.3250 0.0980],'MarkerSize',20,'LineWidth',2,'Visible','off');
end
% Return hardcoded obstacle data
function [obstacleData,varargout] = getObstacleData(varargin)
numObstacles = 10;
obstacleData = cell(numObstacles,1);
% Well
obstacleData{1} = [1.2617    1.2742    1.5925    1.9108    1.9296    1.5988; 1.2118    1.4926    1.7267    1.4965    1.2001    1.1298];
% Bottom Cliff
obstacleData{2} = [-2.2400   -1.8405   -1.6158   -1.3973   -1.3973   -0.9916   -0.6171    0.3504    0.9683    0.9746    1.1493    1.8860    2.3727    2.7785    3.5525    4.1080    4.8196    5.3876    6.3363    7.3600    7.3600   -2.2400; -1.3084   -1.3903   -0.9729   -0.7544   -0.5165   -0.4306   -0.5399   -0.3877   -0.3760   -0.2005    0.1623    0.2755    -0.2083    0.2911    0.7787    0.9699    1.3444    1.4341    1.1181    1.7579   -1.6400   -1.6400];
% Bottom Boulder
obstacleData{3} = [1.7236    1.7298    1.8047    2.2354    2.2729    1.9982; 0.3418    0.4744    0.6305    0.7475    0.4510    0.2872];
% Bottom Stump
obstacleData{4} = [0.6562    0.9121    1.0245    0.8435; 0.0726    0.1935    0.0648   -0.0561];
% Bottom Tree
obstacleData{5} = [2.3228    2.8097    3.3215    3.4776    3.2404    2.4539; 1.0011    1.3054    1.2001    1.0284    0.5017    0.5720];
% Top Forrest
obstacleData{6} = [-2.2400   -2.0091   -1.7656   -1.3474   -0.9854   -0.5984   -0.5422   -0.2738    0.0196    0.5251    0.7623    0.8123    0.9870    1.3990    1.3865    1.9171    2.0981    2.8783    2.9907    2.7223    2.7223   -2.2400; 1.0011    1.0518    0.8372    0.8372    1.1844    1.4224    1.5317    1.5980    1.4263    1.5082    1.6292    1.8867    1.9413    1.9491    2.2027    2.2261    1.9218    1.9803    2.2183    2.4991    4.3600    4.3600];
% Top Stump
obstacleData{7} = [3.0406    3.3278    3.3590    3.1717    2.9657; 1.5785    1.5395    1.6994    1.7891    1.6682];
% Top Tree
obstacleData{8} = [3.9519    4.4700    4.7072    4.8633    4.8820    4.7509    4.5761    4.3701    4.1142    3.9270; 1.4458    1.3093    1.3990    1.5551    1.7657    1.8711    1.9569    1.9764    1.9062    1.7657];
% Top Rock
obstacleData{9} = [4.6885    4.7384    4.8633    5.1192    5.2565    5.2690    5.0567    4.7197; 1.9686    1.8945    1.8515    1.8320    1.9530    2.0154    2.1129    2.0427];
% Shack
obstacleData{10} = [3.0968    3.5525    3.5525    3.4027    2.7285    2.7285    5.4687    5.6685    5.6685    5.6248    5.6248    4.0893    4.0893    4.4014    5.2752    5.2752    3.0968; 2.7254    2.7254    2.6630    2.4991    2.4991    4.0596    4.0596    3.9270    2.6279    2.5967    2.1753    2.1753    2.5225    2.7254    2.7254    3.8140    3.8140];
% Plank
% obstacleData{11} = [3.4776    3.5899    3.7147    3.6086; 2.3860    2.1480    2.2222    2.4133];
if nargin>0
    plot_obstacles = varargin{1};
    if nargin>1
        imPos = varargin{2};
    else
        imPos = zeros(2,1);
    end
else
    plot_obstacles = false;
    imPos = zeros(2,1);
end
shiftInPos = imPos(:) + 0.64*[1;1];
hObstacles = [];
if plot_obstacles
    hObstacles = gobjects(numObstacles,1);
end
for ii = 1:numObstacles
    obstacleData{ii}(1,:) = obstacleData{ii}(1,:) + shiftInPos(1);
    obstacleData{ii}(2,:) = obstacleData{ii}(2,:) + shiftInPos(2);
    if plot_obstacles
        hObstacles(ii) = patch('XData',obstacleData{ii}(1,:),'YData',obstacleData{ii}(2,:),'FaceColor','r','EdgeColor','r','FaceAlpha',0.333,'LineWidth',2);
    end
end
if nargout>1
    varargout{1} = hObstacles;
end
end
% Get obstacle data in stacked matrix form separated by nan's
function obstacleDataMat = getObstacleDataMat(obstacleData)
numObstacles = length(obstacleData);
obstacleDataMat = [obstacleData{1}.';obstacleData{1}(:,1).'];
for kk = 2:numObstacles
    obstacleDataMat = [obstacleDataMat;nan(1,2);obstacleData{kk}.';obstacleData{kk}(:,1).'];
end
end
% Update the input arguments position given a velocity field
function [varargout] = transitionScenery(dt,v,varargin)
assert((nargin-2)==nargout,'Outputs must match extra inputs.')
varargout = cell(nargout,1);
for ii = 1:nargout
    varargout{ii} = varargin{ii} + dt*v;
end
end
% Get range sensor information
function [sensorData] = rangeSensor(robotPositions,obstacleData,delta,Delta)
numRobots = size(robotPositions,2);
numSensors = 8;
minSenRng = 0*delta;
sensorData  = Inf(numSensors,numRobots);
sensorDir = 0:2*pi/(numSensors):2*pi*(numSensors-1)/numSensors;
sensorRaysX = [minSenRng*cos(sensorDir);Delta*cos(sensorDir)];
sensorRaysY = [minSenRng*sin(sensorDir);Delta*sin(sensorDir)];
robotTheta = 0:2*pi/50:2*pi;
robotBody = 0.5*delta*[cos(robotTheta);sin(robotTheta)];
% Construct neighboring robots as stacked polygons separated by nan's
robotPolyX = [robotBody(1,:).'+robotPositions(1,:);nan(1,numRobots)];
robotPolyY = [robotBody(2,:).'+robotPositions(2,:);nan(1,numRobots)];
for ii = 1:numRobots
    for kk = 1:numSensors
        % Construct the current sensor ray
        sensorRay = [sensorRaysX(:,kk)+robotPositions(1,ii),sensorRaysY(:,kk)+robotPositions(2,ii)];
        % Get neighbor agent polygons
        neighborsPolyX = robotPolyX(:,(1:numRobots)~=ii);
        objectPolyX = [neighborsPolyX(:);obstacleData(:,1)];
        neighborsPolyY = robotPolyY(:,(1:numRobots)~=ii);
        objectPolyY = [neighborsPolyY(:);obstacleData(:,2)];
        % Check for intersections
        [tempX,tempY] = ...
            polyxpoly(sensorRay(:,1),sensorRay(:,2),objectPolyX,objectPolyY);
        % If there are any intersections, select the closest to the robot
        if ~isempty(tempX) && ~isempty(tempY)
            sensorData(kk,ii) = min(vecnorm([tempX.'-robotPositions(1,ii);tempY.'-robotPositions(2,ii)]));
        end
    end
end
end
% Check for collisions
function [collisionDetected] = checkCollisions(robotPositions,obstacleData,delta)
collisionDetected = false;
numRobots = size(robotPositions,2);
robotTheta = 0:2*pi/50:2*pi;
robotBody = 0.5*delta*[cos(robotTheta);sin(robotTheta)];
% Construct neighboring robots as stacked polygons separated by nan's
robotPolyX = [robotBody(1,:).'+robotPositions(1,:);nan(1,numRobots)];
robotPolyY = [robotBody(2,:).'+robotPositions(2,:);nan(1,numRobots)];
[tempX,tempY] = polyxpoly(robotPolyX,robotPolyY,obstacleData(:,1),obstacleData(:,2));
% If there are any intersections, collision detected
if ~isempty(tempX) && ~isempty(tempY)
    collisionDetected = true;
end
end
% Get bounded Voronoi tessellation
function [vcells,ADelaunay] = boundedVoronoi(generatorSeeds,domBounds)
numSeeds = size(generatorSeeds,2);
% Find the domain boundary hyperplanes tangents
t = zeros(size(domBounds));
numBoundaries = size(t,2);
% First boundary
t(:,1) = diff(domBounds(:,[end 1]),1,2);
% Remaining boundaries
for ii = 2:numBoundaries
    t(:,ii) = diff(domBounds(:,[ii-1 ii]),1,2);
end
paddedSeeds = zeros(2,numSeeds*(numBoundaries+1));
% Interior points
paddedSeeds(:,1:numSeeds) = generatorSeeds;
% Reflected points
for ii = 1:4
    vecNormal2bnd = bsxfun(@minus,generatorSeeds,t(:,ii));
    paddedSeeds(:,(1:numSeeds)+ii*numSeeds) = vecNormal2bnd-2*t(:,ii)*(t(:,ii).'*vecNormal2bnd)/dot(t(:,ii),t(:,ii));
end
% Find the tessellation using domain interior and exterior points
[VVert,VIndx] = voronoin(paddedSeeds.');
% Keep only the cells for the interior (real) agents
vcells = cell(numSeeds,1);
for ii = 1:numSeeds
    vcells{ii} = [VVert(VIndx{ii},1),VVert(VIndx{ii},2)].';
end
% Get Delaunay neighbors
dTri = delaunayTriangulation(paddedSeeds.');
ADelaunay = zeros(numSeeds);
for ii = 1:size(dTri.ConnectivityList,1)
    % No neighbors outside domain
    if all(dTri.ConnectivityList(ii,[1 2]) <= numSeeds) 
        ADelaunay(dTri.ConnectivityList(ii,1),dTri.ConnectivityList(ii,2)) = 1;
    end
    if all(dTri.ConnectivityList(ii,[1 3]) <= numSeeds)
        ADelaunay(dTri.ConnectivityList(ii,1),dTri.ConnectivityList(ii,3)) = 1;
    end
    if all(dTri.ConnectivityList(ii,[2 3]) <= numSeeds)
        ADelaunay(dTri.ConnectivityList(ii,2),dTri.ConnectivityList(ii,3)) = 1;
    end
end
ADelaunay = (ADelaunay + ADelaunay.')~=0;
end
% Generate spaced-out random points
function [markers] = getSearchMarkers(domainBoundary,minimumSeparation,numPoints)
markers = Inf(2,numPoints);
rng('shuffle')
x = domainBoundary(1) + 0.1*diff(domainBoundary(1:2)) + 0.8*diff(domainBoundary(1:2))*rand(1, 10000);
y = domainBoundary(3) + 0.1*diff(domainBoundary(3:4)) + 0.8*diff(domainBoundary(3:4))*rand(1, 10000);
% Initialize first point.
markers(:,1) = [x(1);y(1)];
counter = 2;
while counter<=numPoints
    % Try dropping down more points.
    for k = 1:size(x,2)
        % Get a trial point.
        thisX = x(k);
        thisY = y(k);
        % See how far is is away from existing keeper points.
        distances = sqrt((thisX-markers(1,:)).^2 + (thisY - markers(2,:)).^2);
        minDistance = min(distances);
        if minDistance >= minimumSeparation
            markers(:,counter) = [thisX;thisY];
            counter = counter + 1;
            if counter > numPoints
                break
            end
        end
    end
    rng('shuffle')
    x = domainBoundary(1) + 0.1*diff(domainBoundary(1:2)) + 0.8*diff(domainBoundary(1:2))*rand(1, 10000);
    y = domainBoundary(3) + 0.1*diff(domainBoundary(3:4)) + 0.8*diff(domainBoundary(3:4))*rand(1, 10000);
end
end