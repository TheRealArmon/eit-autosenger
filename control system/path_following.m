path = [5     10;
        5     5;
        8     5;
        8     3;
        12    3];
    
robotInitialLocation = path(1,:);
robotGoal = path(end,:);
initialOrientation = 0;

robotCurrentPose = [robotInitialLocation initialOrientation]';
robot = differentialDriveKinematics("TrackWidth", 1, "VehicleInputs", "VehicleSpeedHeadingRate");

%plotting path
% figure
% plot(path(:,1), path(:,2),'k--d')
% xlim([0.9*min(path(:,1)) 1.1*max(path(:,1))])
% ylim([0.9*min(path(:,2)) 1.1*max(path(:,2))])

controller = controllerPurePursuit;
controller.Waypoints = path;
controller.DesiredLinearVelocity = 0.5;
controller.MaxAngularVelocity = 2;
controller.LookaheadDistance = 0.3;

goalRadius = 0.1;
distanceToGoal = norm(robotInitialLocation - robotGoal);

% Initialize the simulation loop
sampleTime = 0.1;
vizRate = rateControl(10);

% Initialize the figure
figure

% Determine vehicle frame size to most closely represent vehicle with plotTransforms
frameSize = robot.TrackWidth/0.8;
nextWayPoint = 2;
prevWayPoint = 1;

while( distanceToGoal > goalRadius )
    
    % Compute the controller outputs, i.e., the inputs to the robot
    [v, omega] = controller(robotCurrentPose);
    
    % Get the robot's velocity using controller inputs
    vel = derivative(robot, robotCurrentPose, [v omega]);
    
    % Update the current pose
    robotCurrentPose = robotCurrentPose + vel*sampleTime; 
    
    % Re-compute the distance to the goal
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));  
    
    
    % Compute distance to next waypoint
    distanceToNextWaypoint = norm(robotCurrentPose(1:2) - path(nextWayPoint,:)');
    % Slow down as approaching a waypoint
    if distanceToNextWaypoint < 0.5 && nextWayPoint < size(path,1)
        nextWayPoint = nextWayPoint + 1;
        prevWayPoint = prevWayPoint + 1;
        controller.DesiredLinearVelocity = 0.5;
        controller.MaxAngularVelocity = 2;
    end
    
    % Compute distance to the previous waypoint
    distanceToPrevWaypoint = norm(robotCurrentPose(1:2) - path(prevWayPoint,:)');
    % Speed up as moving away from a waypoint
    if distanceToPrevWaypoint > 0.5 && distanceToNextWaypoint > 1
        controller.DesiredLinearVelocity = 1;
        controller.MaxAngularVelocity = 1;
    end
    
    
    
    
    % Update the plot
    hold off
    
    % Plot path each instance so that it stays persistent while robot mesh
    % moves
    plot(path(:,1), path(:,2),"k--d")
    hold all
    
    % Plot the path of the robot as a set of transforms
    plotTrVec = [robotCurrentPose(1:2); 0];
    plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
    plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize);
    light;
    title('Bane'); xlabel('x koordinat [m]');ylabel('y koordinat [m]');
    xlim([min(path(:,1))-2 max(path(:,1))+2])
    ylim([min(path(:,2))-2 max(path(:,2))+2])
    
    waitfor(vizRate);
end