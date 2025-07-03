function [final_position, total_time] = simulatePathFollowing(waypoints, start_position, sampleTime)

    % Defaults
    if nargin < 2
        start_position = waypoints(1, :);
    end
    if nargin < 3
        sampleTime = 0.05;
    end

    tVec = 0:sampleTime:40;

    % Initial pose: [x; y; theta]
    initPose = [start_position(1); start_position(2); 0];

    % Differential drive robot
    diffDrive = differentialDriveKinematics(VehicleInputs="VehicleSpeedHeadingRate");
    diffDrive.WheelSpeedRange = [-10 10]*2*pi;

    % Pure Pursuit controller
    controller = controllerPurePursuit(Waypoints=waypoints, ...
        DesiredLinearVelocity=3, MaxAngularVelocity=3*pi);

    % Goal
    goalPoint = waypoints(end,:)';
    goalRadius = 1;

    % Simulate with ode45
    [~, diffDrivePose] = ode45(@(t,y)derivative(diffDrive, y, ...
        myMobileRobotController(controller, y, goalPoint, goalRadius)), ...
        tVec, initPose);

    % Visualization (every 10th frame)
    indices = 1:10:size(diffDrivePose,1);
    diffDriveTranslations = [diffDrivePose(indices, 1:2), zeros(length(indices),1)];
    diffDriveRot = axang2quat([repmat([0 0 1], length(indices), 1), diffDrivePose(indices, 3)]);

    plot(waypoints(:,1), waypoints(:,2), "kx-", MarkerSize=20);
    hold on;
    plotTransforms(diffDriveTranslations, diffDriveRot, ...
        MeshFilePath="groundvehicle.stl", MeshColor="g");

    axis equal
    view(0,90)
    title('Trajectory - Differential Drive Only');

    % Return updated position & time (only 4% of the path forward)
    step_index = round(length(diffDrivePose) * 0.1);
    step_index = max(2, min(step_index, length(diffDrivePose)));  % ensure within bounds
    final_position = diffDrivePose(step_index, 1:2);
    total_time = (step_index - 1) * sampleTime;
end
