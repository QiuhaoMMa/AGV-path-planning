function simulatePathFollowing(waypoints)

    sampleTime = 0.05;
    tVec = 0:sampleTime:40;
    initPose = [waypoints(1,:)'; 0];  % Initial pose [x; y; theta]

    %{
    % Initialize Unicycle model
    unicycle = unicycleKinematics(VehicleInputs="VehicleSpeedHeadingRate");

    % Initialize Bicycle model
    bicycle = bicycleKinematics(VehicleInputs="VehicleSpeedHeadingRate", ...
                                 MaxSteeringAngle=pi/8);
    %}

    % Initialize Differential Drive model
    diffDrive = differentialDriveKinematics(VehicleInputs="VehicleSpeedHeadingRate");
    diffDrive.WheelSpeedRange = [-10 10]*2*pi;

    % Pure Pursuit Controllers
    %{
    controller1 = controllerPurePursuit(Waypoints=waypoints, ...
        DesiredLinearVelocity=3, MaxAngularVelocity=3*pi);

    controller2 = controllerPurePursuit(Waypoints=waypoints, ...
        DesiredLinearVelocity=3, MaxAngularVelocity=3*pi);
    %}

    controller3 = controllerPurePursuit(Waypoints=waypoints, ...
        DesiredLinearVelocity=3, MaxAngularVelocity=3*pi);

    % Goal configuration
    goalPoint = waypoints(end,:)';
    goalRadius = 1;

    % ODE Simulation
    %{
    [~, unicyclePose] = ode45(@(t,y)derivative(unicycle, y, ...
        myMobileRobotController(controller1, y, goalPoint, goalRadius)), ...
        tVec, initPose);

    [~, bicyclePose] = ode45(@(t,y)derivative(bicycle, y, ...
        myMobileRobotController(controller2, y, goalPoint, goalRadius)), ...
        tVec, initPose);
    %}

    [~, diffDrivePose] = ode45(@(t,y)derivative(diffDrive, y, ...
        myMobileRobotController(controller3, y, goalPoint, goalRadius)), ...
        tVec, initPose);

    % Prepare visualization
    %{
    unicycleTranslations = [unicyclePose(:,1:2) zeros(length(unicyclePose),1)];
    unicycleRot = axang2quat([repmat([0 0 1], length(unicyclePose), 1), unicyclePose(:,3)]);

    bicycleTranslations = [bicyclePose(:,1:2) zeros(length(bicyclePose),1)];
    bicycleRot = axang2quat([repmat([0 0 1], length(bicyclePose), 1), bicyclePose(:,3)]);
    %}

    diffDriveTranslations = [diffDrivePose(:,1:2) zeros(length(diffDrivePose),1)];
    diffDriveRot = axang2quat([repmat([0 0 1], length(diffDrivePose), 1), diffDrivePose(:,3)]);

    % Plot result
    figure
    plot(waypoints(:,1), waypoints(:,2), "kx-", MarkerSize=20);  % Waypoints
    hold on

    %{
    plotTransforms(unicycleTranslations(1:10:end,:), unicycleRot(1:10:end,:), ...
        MeshFilePath="groundvehicle.stl", MeshColor="r");

    plotTransforms(bicycleTranslations(1:10:end,:), bicycleRot(1:10:end,:), ...
        MeshFilePath="groundvehicle.stl", MeshColor="b");
    %}

    plotTransforms(diffDriveTranslations(1:10:end,:), diffDriveRot(1:10:end,:), ...
        MeshFilePath="groundvehicle.stl", MeshColor="g");

    axis equal
    view(0,90)
    title('Trajectory - Differential Drive Only')
    disp('Using latest simulatePathFollowing...');

end
