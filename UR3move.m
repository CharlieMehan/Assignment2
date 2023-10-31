function [ ] = UR3move( )
clf;

robot = UR3;

% Rotate the base around the Y axis 
% robot.model.base = troty(pi);


workspace = [-1 1 -1 1 -1 1]; % Set the size of the workspace when drawing the robot
scale = 1;
q = zeros(1,6); % Create a vector of initial joint angles
robot.model.plot(q,'workspace',workspace,'scale',scale); % Plot the robot

R_down = trotx(pi/2);  % This rotates z-axis to point downwards

% Simulate reaching behind to a target pose
liftBehindPose = transl(-0.5,0.5, 0.3) * R_down;
pickBehindPose = transl(-0.5, 0.5, 0) * R_down;
liftForwardPose = transl(-0.5, 0.5, 0.3) * R_down; 
frontPose = transl(-0.5, -0.4, 0.2) * R_down; 
placePose = transl(-0.5, -0.4, 0) * R_down;
posesSequence = {liftBehindPose, pickBehindPose, liftForwardPose, frontPose, placePose};

q_previous = q;

% Animate each step in sequence
for k = 1:length(posesSequence)
    targetPose = posesSequence{k};
    initialPose = robot.model.fkine(q_previous).T;

    % Interpolate in Cartesian space
    numSteps = 75;
    s = lspb(0,1,numSteps); 

    intermediatePoses = zeros(4, 4, numSteps);

    for i = 1:numSteps
        intermediatePoses(:,:,i) = initialPose + s(i) * (targetPose - initialPose);
    end

    % Solve for joint configurations and animate
    for i = 1:numSteps
        q_now = robot.model.ikcon(intermediatePoses(:,:,i), q_previous);
        robot.model.animate(q_now);
        drawnow();
        pause(0.01);
        q_previous = q_now;
    end
end

disp('Robot movement complete. Press enter to end.');
pause;

end



