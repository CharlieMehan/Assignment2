function [] = UR3collision()
clf;

%#ok<*NASGU>

robot = UR3;

% Define workspace and plot settings
workspace = [-1 1 -1 1 -1 1];
scale = 1;
q = zeros(1,6);
robot.model.plot(q,'workspace',workspace,'scale',scale);

% Rotation matrix to align the tool with the world frame
R_down = trotx(pi/2);

% Define poses for the robot's trajectory
posesSequence = {
    transl(-0.5, 0.5, 0.5) * R_down,
    transl(-0.5, 0.5, 0) * R_down,
    transl(-0.5, 0.5, 0.3) * R_down,
    transl(-0.5, -0.4, 0.2) * R_down,
    transl(-0.5, -0.4, 0) * R_down,
    robot.model.fkine(q).T
};

% Initialization
qMatrix = q;
deltaT = 0.05; 
epsilon = 0.1; 
W = diag([1 1 1 0.1 0.1 0.1]); 
interpolation_steps = 20;

% Define the plane for collision 
plane_normal = [0, 0, 1];
plane_point = [0, 0, 0.2]; %  point on the plane, 0.2m above the base
plane_d = -dot(plane_normal, plane_point);

% Collision avoidance
for k = 1:length(posesSequence)-1
    T_start = robot.model.fkine(qMatrix(end,:)).T;

    for interp_step = 1:interpolation_steps
        alpha = interp_step / interpolation_steps;
        T_intermediate = trinterp(T_start, posesSequence{k+1}, alpha);

             % Use the LinePlaneIntersection function

            [intersectionPoint, check] = LinePlaneIntersection(plane_normal, plane_point, T_start(1:3,4)', T_intermediate(1:3,4)');
            
            % If check is 1, a collision is imminent
            if check == 1
                
            % Move the target point 0.1m along the plane normal away from the plane
            adjusted_target = T_intermediate(1:3, 4) - 0.05 * plane_normal';
            T_intermediate(1:3, 4) = adjusted_target;
        end

        % Continue with RMRC calculations as normal
        T = robot.model.fkine(qMatrix(end,:)).T;
        Rd = T_intermediate(1:3,1:3);
        Ra = T(1:3,1:3);
        Rdot = (1/deltaT)*(Rd - Ra);
        S = Rdot*Ra';

        deltaX = T_intermediate(1:3,4) - T(1:3,4);
        linear_velocity = (1/deltaT)*deltaX;
        angular_velocity = [S(3,2);S(1,3);S(2,1)]; 

        xdot = W*[linear_velocity;angular_velocity];
        J = robot.model.jacob0(qMatrix(end,:));

        m = sqrt(det(J*J'));
        if m < epsilon 
            lambda = (1 - m/epsilon)*5E-2;
        else
            lambda = 0;
        end

        invJ = inv(J'*J + lambda *eye(6))*J'; 
        qdot = (invJ*xdot)';

        for j = 1:6 
            if qMatrix(end,j) + deltaT*qdot(j) < robot.model.qlim(j,1) 
                qdot(j) = 0; 
            elseif qMatrix(end,j) + deltaT*qdot(j) > robot.model.qlim(j,2) 
                qdot(j) = 0; 
            end
        end

        % Append the new configuration to the qMatrix
        qMatrix = [qMatrix; qMatrix(end,:) + deltaT*qdot];
        robot.model.animate(qMatrix(end,:));
        drawnow();
        pause(0.01);
    end
end
end