function [ ] = UR32( )
r = UR3;



% Define a target pose for the robot's end effector
targetPose = transl(-0.5, 0.5, 0);


% Compute the initial and final joint configurations using inverse kinematics
q0 = q;
q_final = robot.model.ikcon(targetPose, q0); % Use the current configuration as an initial guess

% Generate a joint trajectory between the initial and final configurations
steps = 150;
qtraj = jtraj(q0, q_final, steps);

% Animate the robot's movement along the trajectory
for i = 1:size(qtraj, 1)
    robot.model.animate(qtraj(i, :));
    drawnow();
    pause(0.01);  % Add a small pause to visualize the movement
end

disp('Robot movement complete. Press enter to end.');
pause;


end