function [ patch_data ] = gritsbot_patch( varargin )
%KHEPERAMASK Returns the information required to create patch object.
%   Accepts an optional color triplet argument.

if nargin==2
    BodyColor = varargin{2};
    robotDiameter = varargin{1};
else if nargin==1
        BodyColor = [1 1 1];
        robotDiameter = varargin{1};
    else
        BodyColor = 0.9*[1 1 1];
        robotDiameter = 0.11;
        robotRadius = 0.5*robotDiameter;
    end
    
    % Define custom patch variables
    thetaBase = (0:10:360).';
    k4Base =  [cosd(thetaBase),sind(thetaBase),ones(size(thetaBase))];
    %k4BaseCentroid = sum(k4Base(:,1:2))/size(k4Base,1);
    %k4Base(:,1:2) = bsxfun(@minus,k4Base(:,1:2),k4BaseCentroid(1:2));
    %k3NormalizingFactor = max(max(abs(k4Base(:,1:2))));
    thetaTop = [-60:10:60 120:10:240].';
    k4TopRadX = 0.55;
    k4TopRadY = 0.7;
    k4Top = [k4TopRadX*cosd(thetaTop),k4TopRadY*sind(thetaTop),ones(size(thetaTop));
        k4TopRadX*cosd(thetaTop(1)),k4TopRadY*sind(thetaTop(1)),1];
    thetaLEDPos = [0; 150; 210];
    thetaLEDDia = linspace(0,360,8).';
    LED_Color = [0 0 1;0 1 0;1 0 0];
    led_diam = 0.05;
    k4LED1 = [led_diam*cosd(thetaLEDDia)+2/3*cosd(thetaLEDPos(1)),led_diam*sind(thetaLEDDia)+2/3*sind(thetaLEDPos(1)),ones(size(thetaLEDDia))];
    k4LED2 = [led_diam*cosd(thetaLEDDia)+2/3*cosd(thetaLEDPos(2)),led_diam*sind(thetaLEDDia)+2/3*sind(thetaLEDPos(2)),ones(size(thetaLEDDia))];
    k4LED3 = [led_diam*cosd(thetaLEDDia)+2/3*cosd(thetaLEDPos(3)),led_diam*sind(thetaLEDDia)+2/3*sind(thetaLEDPos(3)),ones(size(thetaLEDDia))];
    k4TopGrooveR = [...
        [k4TopRadX*cosd(-60),k4TopRadY*sind(-60)] + [-0.1, 0.01], 1;
        [k4TopRadX*cosd(-60),k4TopRadY*sind(-60)] + [-0.1, 0.11], 1;
        [k4TopRadX*cosd(240),k4TopRadY*sind(240)] + [ 0.1, 0.11], 1;
        [k4TopRadX*cosd(240),k4TopRadY*sind(240)] + [ 0.1, 0.01], 1];
    k4TopGrooveL = [...
        [k4TopRadX*cosd(60), k4TopRadY*sind(60)]  + [-0.1, -0.01], 1;
        [k4TopRadX*cosd(60), k4TopRadY*sind(60)]  + [-0.1, -0.11], 1;
        [k4TopRadX*cosd(120),k4TopRadY*sind(120)]+ [ 0.1, -0.11], 1;
        [k4TopRadX*cosd(120),k4TopRadY*sind(120)]+ [ 0.1, -0.01], 1];
    %k4Top(:,1:2) = bsxfun(@minus,k4Top(:,1:2),k4BaseCentroid(1:2));
    k4Base(:,1:2) = k4Base(:,1:2)*robotRadius;
    k4Top(:,1:2) = k4Top(:,1:2)*robotRadius;
    k4LED1(:,1:2) = k4LED1(:,1:2)*robotRadius;
    k4LED2(:,1:2) = k4LED2(:,1:2)*robotRadius;
    k4LED3(:,1:2) = k4LED3(:,1:2)*robotRadius;
    k4TopGrooveR(:,1:2) = k4TopGrooveR(:,1:2)*robotRadius;
    k4TopGrooveL(:,1:2) = k4TopGrooveL(:,1:2)*robotRadius;
    k4BaseColor = BodyColor;
    k4TopColor = 0.9*BodyColor;
    
    k4TopGrooveColor = [0 0 0];
    % Define common patch variables
    robotVertices = [k4Base;k4Top;k4LED1;k4LED2;k4LED3;k4TopGrooveR;k4TopGrooveL];
    robotVertices(:,1:2) = robotVertices(:,1:2)*[0 1;-1 0];
    robotColor = [k4BaseColor;k4TopColor;LED_Color;k4TopGrooveColor;k4TopGrooveColor];
    maxVert = max([size(k4Base,1),size(k4Top,1),size(k4LED1,1),size(k4LED2,1),size(k4LED3,1),size(k4TopGrooveR,1),size(k4TopGrooveL,1)]);
    robotFaces = nan(7,maxVert);
    robotFaces(1,1:size(k4Base,1)) =  1:size(k4Base,1);
    robotFaces(2,1:size(k4Top ,1)) = (1:size(k4Top,1))+max(robotFaces(1,:));
    robotFaces(3,1:size(k4LED1,1)) = (1:size(k4LED1,1))+max(robotFaces(2,:));
    robotFaces(4,1:size(k4LED2,1)) = (1:size(k4LED2,1))+max(robotFaces(3,:));
    robotFaces(5,1:size(k4LED3,1)) = (1:size(k4LED3,1))+max(robotFaces(4,:));
    robotFaces(6,1:size(k4TopGrooveR,1)) = (1:size(k4TopGrooveR,1))+max(robotFaces(5,:));
    robotFaces(7,1:size(k4TopGrooveL,1)) = (1:size(k4TopGrooveL,1))+max(robotFaces(6,:));
    
    
    
end


patch_data = [];
patch_data.vertices = robotVertices;
patch_data.colors = robotColor;
patch_data.faces = robotFaces;
end

