%% Part A: Location of the Marker Tip
% Location of the marker tip with respect to the world origin in the
% all-zeros configuration.

% Origin to back right corner of mount location. [m]
v1 = [0.75, 0.5, 1.0];

% Back right corner of mount location to base centered below the end plate.
% [m]
v2 = [-140e-3, 220e-3, 0];

% Base centered below the end plate to end plate. [m]
v3 = [0, 0, 910e-3+346e-3];

% End plate to tip of vertical marker. [m]
v4 = [0, 0, 12e-3];

fin = v1+v2+v3+v4

%% Forward Kinematics of the 7-DOF Barrett WAM arm.
init = [v1'; 1]; % 1 in the 4th row because translations matter for the physical location of the translated point.
pos = zeros(length(JointData), 3);
for i = 1:length(JointData)
    % Save all joint angles at current step into a column vector.
    J = JointData(i, :);
    
    % Figure out end effector position using homogeneous transformations.
    % J1 to J2. t1 about the +z axis. 0 translation.
    H1 = [cos(J(1)), -sin(J(1)), 0, 0;
          sin(J(1)), cos(J(1)),  0, 0;
          0, 0, 1, 0;
          0, 0, 0, 1];
    
    % J2 to J3. -t2 about +x axis. 0 translation.
    H2 = [1, 0, 0, 0;
        0 cos(-J(2)) -sin(-J(2)) 0;
        0 sin(-J(2)) cos(-J(2)) 0;
        0, 0, 0, 1];
    
    % J3 to J4. t3 about +z axis. [0, 45, 550] translation in mm.
    H3 = [cos(J(3)), -sin(J(3)), 0, 0;
          sin(J(3)), cos(J(3)),  0, 45e-3;
          0, 0, 1, 550e-3;
          0, 0, 0, 1];
    
    % J4 to J5. -t4 about +x axis. 0 translation.
    H4 = [1, 0, 0, 0;
        0 cos(-J(4)) -sin(-J(4)) 0;
        0 sin(-J(4)) cos(-J(4)) 0;
        0, 0, 0, 1];
    
    % J5 to J6. t5 about +z axis. [0, 0, 300] translation in mm.
    H5 = [cos(J(5)), -sin(J(5)), 0, 0;
          sin(J(5)), cos(J(5)),  0, 0;
          0, 0, 1, 300e-3;
          0, 0, 0, 1];
    
    % J6 to J7. -t6 about +x axis. [0,0,60] translation in mm.
    H6 = [1, 0, 0, 0;
        0 cos(-J(6)) -sin(-J(6)) 0;
        0 sin(-J(6)) cos(-J(6)) 60e-3;
        0, 0, 0, 1];
    
    % J7 to marker tip.
    H7 = [cos(J(7)), -sin(J(7)), 0, 0;
          sin(J(7)), cos(J(7)),  0, 0;
          0, 0, 1, 12e-2;
          0, 0, 0, 1];
    
    % Relative frames means multiply subsequent frames on the right side.
    % (RelativeRight. AbsoluteLeft)
    H = H1*H2*H3*H4*H5*H6*H7;
    currpos = H*init;
    pos(i, 1:3) = currpos(1:3);
    
end

scatter3(pos(:, 1), pos(:, 2), pos(:,3));