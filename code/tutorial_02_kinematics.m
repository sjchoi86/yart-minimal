ccc
%% Show and control of some robots
ccc
robot_name = 'coman'; % coman / iiwa7
urdf_path = sprintf('../urdf/%s/%s_urdf.xml',robot_name,robot_name);
cache_folder = '../cache/';
chain = get_chain_from_urdf_with_caching(robot_name,'RE',0,'SKIP_CAPSULE',0,...
    'urdf_path',urdf_path,'cache_folder',cache_folder);
plot_chain_graph(chain,'fig_idx',2,'fig_pos',[0.5,0.0,0.5,0.2],'NO_MARGIN',1);
animate_chain_with_joint_control_using_sliders(chain,...
    'fig_pos_robot',[0.5,0.5,0.4,0.5],'fig_pos_slider',[0.5,0.2,0.4,0.2],...
    'PLOT_MESH',0,'PLOT_LINK',1,'PLOT_JOINT_AXIS',1,'PLOT_CAPSULE',0,'PLOT_JOINT_NAME',0,...
    'PLOT_COM',0,'PRINT_JOINT_POS',0,'AXIS_OFF',1,'NO_MARGIN',1,'PLOT_GRAPH',0);

%% Basic position, rotation, Coordinates
ccc

% Define a local coordinate {A}
p = cv([0.5,1,1]); % position as a column vector
R = rpy2r(360*rand(1,3)*D2R); % rotation matrix from Euler angle
T_A = pr2t(p,R);

% Define the World coordinate
T_W = pr2t('','');

% Plot
fig_idx = 1;
set_fig(figure(fig_idx),'pos',[0.0,0.5,0.5,1.0],...
    'view_info',[80,26],'axis_info',2*[-1,+1,-1,+1,-1,+1],'AXIS_EQUAL',1,'GRID_ON',1,...
    'REMOVE_MENUBAR',1,'USE_DRAGZOOM',1,...
    'SET_CAMLIGHT',1,'SET_MATERIAL','METAL','SET_AXISLABEL',1,'afs',18);
plot_T(T_W,'fig_idx',fig_idx,'subfig_idx',1,...
    'all',1.0,'alw',3,'text_str','~$\Sigma_W$','text_interp','latex','text_fs',25);
plot_T(T_A,'fig_idx',fig_idx,'subfig_idx',2,...
    'all',0.5,'alw',2,'text_str','~$\Sigma_A$','text_interp','latex','text_fs',25);

%% Coordinate Transformations
ccc
%
% T_W -> T_A -> T_B
%

% Initialize World Coordinate {W} and local coordinates, {A}, {B}, and {C}
T_W = pr2t('',''); % world coordinates {W}
T_A_in_W = pr2t(cv([0,1,0]),rpy2r([30,0,0]*D2R)); % Local coordinates {A} in {W}
T_B_in_A = pr2t(cv([0,0,1]),rpy2r([0,30,0]*D2R)); % Local coordinates {B} in {A}

% Coordinate transfrom: Pre-multiply
%
%       T_{BinW} = T_{W} * T_{AinW} * T_{BinA}
%
% Coordinates in {W}
T_A_in_W = T_W*T_A_in_W; % {A} in {W}
T_B_in_W = T_W*T_A_in_W*T_B_in_A; % {B} in {W}
% Coordinates in {A}
T_W_in_A = inv_T(T_A_in_W);
T_A_in_A = pr2t('','');
% Coordinates in {B}
T_W_in_B = inv_T(T_B_in_W);
T_A_in_B = inv_T(T_B_in_A);
T_B_in_B = pr2t('','');

% Point transform: Pre-multiply
%
%       p_X_in_{A} = T_{B}in{A} * p_X_in{B}
%       p_X_in_{W} = T_{A}in{W} * T_{B}in{A} * p_X_in{B}
%
% Point X in {B}
p_X_in_B = cv([0.3,0.3,0.3]); % point in {B}
T_X_in_B = pr2t(p_X_in_B,'');
% Point X in {A}
T_X_in_A = T_B_in_A*T_X_in_B;
% Point X in {W}
T_X_in_W = T_A_in_W*T_B_in_A*T_X_in_B;

% Plot things in {W}
axis_info = 1.5*[-1,+1,-1,+1,-1,+1];
fig_idx = 1;
set_fig(figure(fig_idx),'pos',[0.0,0.5,0.3,0.5],...
    'view_info',[80,26],'axis_info',axis_info,'AXIS_EQUAL',1,'GRID_ON',1,...
    'REMOVE_MENUBAR',1,'USE_DRAGZOOM',1,...
    'SET_CAMLIGHT',1,'SET_MATERIAL','METAL','SET_AXISLABEL',1,'afs',18);
plot_T(T_W,'fig_idx',fig_idx,'subfig_idx',1,...
    'all',0.5,'alw',3,'text_str','~$\Sigma_W$','text_interp','latex','text_fs',25);
plot_line(t2p(T_W),t2p(T_A_in_W),'fig_idx',fig_idx,'subfig_idx',1,...
    'lc','k','lw',1,'ls','--');
plot_T(T_A_in_W,'fig_idx',fig_idx,'subfig_idx',2,...
    'all',0.5,'alw',3,'text_str','~$\Sigma_A$','text_interp','latex','text_fs',25);
plot_line(t2p(T_A_in_W),t2p(T_B_in_W),'fig_idx',fig_idx,'subfig_idx',2,...
    'lc','k','lw',1,'ls','--');
plot_T(T_B_in_W,'fig_idx',fig_idx,'subfig_idx',3,...
    'all',0.5,'alw',3,'text_str','~$\Sigma_B$','text_interp','latex','text_fs',25);
plot_line(t2p(T_B_in_W),t2p(T_X_in_W),'fig_idx',fig_idx,'subfig_idx',3,...
    'lc','r','lw',1,'ls','--');
plot_T(T_X_in_W,'fig_idx',fig_idx,'subfig_idx',4,...
    'PLOT_AXIS',0,'PLOT_SPHERE',1,'sr',0.05,'sfc','r','sfa',0.5);
plot_title('In the World Coordinate System','fig_idx',fig_idx,'interpreter','latex','tfs',30);

% Plot things in {A}
fig_idx = 2;
set_fig(figure(fig_idx),'pos',[0.3,0.5,0.3,0.5],...
    'view_info',[80,26],'axis_info',axis_info,'AXIS_EQUAL',1,'GRID_ON',1,...
    'REMOVE_MENUBAR',1,'USE_DRAGZOOM',1,...
    'SET_CAMLIGHT',1,'SET_MATERIAL','METAL','SET_AXISLABEL',1,'afs',18);
plot_T(T_W_in_A,'fig_idx',fig_idx,'subfig_idx',1,...
    'all',0.5,'alw',3,'text_str','~$^A\Sigma_A$','text_interp','latex','text_fs',25);
plot_line(t2p(T_W_in_A),t2p(T_A_in_A),'fig_idx',fig_idx,'subfig_idx',1,...
    'lc','k','lw',1,'ls','--');
plot_T(T_A_in_A,'fig_idx',fig_idx,'subfig_idx',2,...
    'all',0.5,'alw',3,'text_str','~$^A\Sigma_A$','text_interp','latex','text_fs',25);
plot_line(t2p(T_A_in_A),t2p(T_B_in_A),'fig_idx',fig_idx,'subfig_idx',2,...
    'lc','k','lw',1,'ls','--');
plot_T(T_B_in_A,'fig_idx',fig_idx,'subfig_idx',3,...
    'all',0.5,'alw',3,'text_str','~$^A\Sigma_B$','text_interp','latex','text_fs',25);
plot_line(t2p(T_B_in_A),t2p(T_X_in_A),'fig_idx',fig_idx,'subfig_idx',3,...
    'lc','r','lw',1,'ls','--');
plot_T(T_X_in_A,'fig_idx',fig_idx,'subfig_idx',4,...
    'PLOT_AXIS',0,'PLOT_SPHERE',1,'sr',0.05,'sfc','r','sfa',0.5);
plot_title('In the Local Coordinate System A','fig_idx',fig_idx,'interpreter','latex','tfs',30);

% Plot things in {B}
fig_idx = 3;
set_fig(figure(fig_idx),'pos',[0.6,0.5,0.3,0.5],...
    'view_info',[80,26],'axis_info',axis_info,'AXIS_EQUAL',1,'GRID_ON',1,...
    'REMOVE_MENUBAR',1,'USE_DRAGZOOM',1,...
    'SET_CAMLIGHT',1,'SET_MATERIAL','METAL','SET_AXISLABEL',1,'afs',18);
plot_T(T_W_in_B,'fig_idx',fig_idx,'subfig_idx',1,...
    'all',0.5,'alw',3,'text_str','~$^A\Sigma_A$','text_interp','latex','text_fs',25);
plot_line(t2p(T_W_in_B),t2p(T_A_in_B),'fig_idx',fig_idx,'subfig_idx',1,...
    'lc','k','lw',1,'ls','--');
plot_T(T_A_in_B,'fig_idx',fig_idx,'subfig_idx',2,...
    'all',0.5,'alw',3,'text_str','~$^A\Sigma_A$','text_interp','latex','text_fs',25);
plot_line(t2p(T_A_in_B),t2p(T_B_in_B),'fig_idx',fig_idx,'subfig_idx',2,...
    'lc','k','lw',1,'ls','--');
plot_T(T_B_in_B,'fig_idx',fig_idx,'subfig_idx',3,...
    'all',0.5,'alw',3,'text_str','~$^A\Sigma_B$','text_interp','latex','text_fs',25);
plot_line(t2p(T_B_in_B),t2p(T_X_in_B),'fig_idx',fig_idx,'subfig_idx',3,...
    'lc','r','lw',1,'ls','--');
plot_T(T_X_in_B,'fig_idx',fig_idx,'subfig_idx',4,...
    'PLOT_AXIS',0,'PLOT_SPHERE',1,'sr',0.05,'sfc','r','sfa',0.5);
plot_title('In the Local Coordinate System B','fig_idx',fig_idx,'interpreter','latex','tfs',30);

%% Coordinate Transformations (again)
%
% Post-multiplication : local transformation
% Pre-multiplication  : global transformation
%
ccc

% Configuration
transform_type = 'Local'; % 'Global', 'Local'
rotation_axis  = 'Z'; % 'X', 'Y', 'Z'

% Random homogeneous transformation matrix
T_w = pr2t(cv(-1+2*rand(1,3)),rpy2r(360*rand(1,3)*D2R));
root_traj = []; x_traj = []; y_traj = []; z_traj = [];
tick = 0; max_tick = 360;
while 1
    if tick < max_tick
        tick = tick + 1;
    end
    % Coordinate transform
    switch rotation_axis
        case 'X'
            T_w2a = pr2t(cv([0,0,0]),rpy2r(tick*[1,0,0]*D2R)); % rotate w.r.t. x-axis
        case 'Y'
            T_w2a = pr2t(cv([0,0,0]),rpy2r(tick*[0,1,0]*D2R)); % rotate w.r.t. y-axis
        case 'Z'
            T_w2a = pr2t(cv([0,0,0]),rpy2r(tick*[0,0,1]*D2R)); % rotate w.r.t. z-axis
    end
    switch lower(transform_type)
        case 'global'
            T_a = T_w2a*T_w; % pre-multiplication (global)
        case 'local'
            T_a = T_w*T_w2a; % post-multiplication (local)
    end
    
    R_t = t2r(T_a); % get the rotation matrix
    % Append the axes trajectories
    root_traj   = [root_traj; rv(t2p(T_a))];
    x_traj      = [x_traj; rv(t2p(T_a)) + 0.5*rv(R_t(:,1))];
    y_traj      = [y_traj; rv(t2p(T_a)) + 0.5*rv(R_t(:,2))];
    z_traj      = [z_traj; rv(t2p(T_a)) + 0.5*rv(R_t(:,3))];
    % Animate
    if mod(tick,5) == 0 % animate
        fig = set_fig(figure(1),'pos',[0.6,0.4,0.3,0.5],...
            'view_info',[80,26],'axis_info',1.6*[-1,+1,-1,+1,-1,+1],'AXIS_EQUAL',1,'GRID_ON',1,...
            'REMOVE_MENUBAR',1,'USE_DRAGZOOM',1,'SET_CAMLIGHT',1,'SET_MATERIAL','METAL',...
            'SET_AXISLABEL',1,'afs',18,'interpreter','latex','NO_MARGIN',0);
        plot_T(pr2t(cv([0,0,0]),eye(3,3)),'fig_idx',1,'subfig_idx',1,...
            'PLOT_AXIS',1,'all',1.0,'alw',3,'PLOT_SPHERE',0,...
            'text_str','World','text_fs',20,'text_interp','latex'); % world coordinate
        plot_T(T_w,'fig_idx',1,'subfig_idx',2,...
            'PLOT_AXIS',1,'all',0.5,'alw',2,'PLOT_SPHERE',0); % initial coordinate
        plot_T(T_a,'fig_idx',1,'subfig_idx',3,...
            'PLOT_AXIS',1,'all',0.5,'alw',2,'PLOT_AXIS_TIP',1,'atr',0.05,...
            'PLOT_SPHERE',1,'sr',0.03,'sfc','k',...
            'text_str','T','text_fs',20,'text_interp','latex'); % transformed coordinate
        plot_traj(root_traj,'fig_idx',1,'subfig_idx',1,'tlc','k','tlw',3);
        plot_traj(x_traj,'fig_idx',1,'subfig_idx',2,'tlc','r','tlw',1);
        plot_traj(y_traj,'fig_idx',1,'subfig_idx',3,'tlc','g','tlw',1);
        plot_traj(z_traj,'fig_idx',1,'subfig_idx',4,'tlc','b','tlw',1);
        title_str = sprintf('[%d/%d] %s Transform w.r.t. %s axis',tick,max_tick,transform_type,rotation_axis);
        plot_title(title_str,'tfs',20,'interpreter','latex');
        drawnow;
        if ~ishandle(fig), break; end
    end % if mod(tick,5) == 0 % animate
end % for tick = 1:360 % for each tick
fprintf('Done.\n');

%% Angular velocity vector \omega
ccc

% Angular velocity vector
w = 0.5 + 0.5*rand(3,1);

% Plot the original angular velocity vector and directional velocity at point
fig_idx = 1; axis_info = 1.5*[-1,+1,-1,+1,-1,+1];
set_fig(figure(fig_idx),'pos',[0.0,0.5,0.3,0.5],...
    'view_info',[80,26],'axis_info',axis_info,'AXIS_EQUAL',1,'GRID_ON',1,...
    'REMOVE_MENUBAR',1,'USE_DRAGZOOM',1,...
    'SET_CAMLIGHT',1,'SET_MATERIAL','METAL','SET_AXISLABEL',1,'afs',18);
plot_T(pr2t(cv([0,0,0]),eye(3,3)),'fig_idx',fig_idx,'subfig_idx',1,...
    'PLOT_AXIS',1,'all',1.0,'alw',3,'PLOT_SPHERE',0,...
    'text_str','~$\Sigma_W$','text_interp','latex','text_fs',25); % world coordinate
sw = 0.05; tw = 0.1; % stem width and tip width
plot_arrow_3d(zeros(3,1),w,'fig_idx',fig_idx,'subfig_idx',1,'alpha',0.7,'color','m',...
    'sw',sw,'tw',tw); % angular velocity vector
ps = cell(1,20); vs = cell(1,20);
for i_idx = 1:20
    p = rand(3,1);          % random point
    v = cross(w,p);         % the directional velocity of the point
    ps{i_idx} = p; vs{i_idx} = v; % append point and directional velocity
    sw = 0.01; tw = 0.04;
    plot_T(p2t(p),'fig_idx',fig_idx,'subfig_idx',1+i_idx,...
        'PLOT_AXIS',0,'PLOT_SPHERE',1,'sr',0.02,'sfc','k');
    plot_arrow_3d(p,p+v,'fig_idx',fig_idx,'subfig_idx',1+i_idx,...
        'alpha',0.5,'color','b','sw',sw,'tw',tw); % directional velocity vector
end
plot_title('Angular Velocity Vector $\omega$',...
    'fig_idx',1,'tfs',25,'interpreter','latex');

% Get a random rotation matrix to rotate the angular velocity vector
R = rpy2r(360*rand(3,1));

% Plot the rotated angular velocity vector and directional velocity at point
fig_idx = 2;
set_fig(figure(fig_idx),'pos',[0.3,0.5,0.3,0.5],...
    'view_info',[80,26],'axis_info',axis_info,'AXIS_EQUAL',1,'GRID_ON',1,...
    'REMOVE_MENUBAR',1,'USE_DRAGZOOM',1,...
    'SET_CAMLIGHT',1,'SET_MATERIAL','METAL','SET_AXISLABEL',1,'afs',18);
plot_T(pr2t(cv([0,0,0]),eye(3,3)),'fig_idx',fig_idx,'subfig_idx',1,...
    'PLOT_AXIS',1,'all',1.0,'alw',3,'PLOT_SPHERE',0,...
    'text_str','~$\Sigma_W$','text_interp','latex','text_fs',25); % world coordinate
sw = 0.05; tw = 0.1; % stem width and tip width
plot_arrow_3d(zeros(3,1),R*w,'fig_idx',fig_idx,'subfig_idx',1,'alpha',0.7,'color','m',...
    'sw',sw,'tw',tw); % rotated angular velocity vector
for i_idx = 1:20
    p = R*ps{i_idx}; % rotate position
    v = R*vs{i_idx}; % rotate velocity
    sw = 0.01; tw = 0.04;
    plot_T(p2t(p),'fig_idx',fig_idx,'subfig_idx',1+i_idx,...
        'PLOT_AXIS',0,'PLOT_SPHERE',1,'sr',0.02,'sfc','k');
    plot_arrow_3d(p,p+v,'fig_idx',fig_idx,'subfig_idx',1+i_idx,...
        'alpha',0.5,'color','b','sw',sw,'tw',tw); % directional velocity vector
end
plot_title('Rotated Angular Velocity Vector $R\omega$',...
    'fig_idx',fig_idx,'tfs',25,'interpreter','latex');

%% Convert R <=> w using Rodrigues' formula
ccc

R = rpy2r(360*D2R*rand(3,1)); % random rotation matrix
w = r2w(R); % log map: w = wedge(ln(R))
R2 = rodrigues(uv(w),norm(w)); % exp map

% Printout
fprintf('R:\n'); disp(R);
fprintf('R2:\n'); disp(R2);

%% Interpolate rotation matrics
ccc

% Get two random rotational matrices
p1 = cv([0,0,0]);
p2 = cv([0,3,0]);
R1 = rpy2r(360*rand(3,1)*D2R);
R2 = rpy2r(360*rand(3,1)*D2R);
R_link = R1'*R2; % rotation matrix which links R1 and R2
w_link = r2w(R_link); % equivalent velocity vector fron the rotation matrix

x_traj = []; y_traj = []; z_traj = [];
all = 1.0; % axis line length
axis_info = [-1,+1,-1,+4,-1,+1];
ts = linspace(0,1,100); % t:[0,1]
for tick = 1:length(ts)
    
    % Interpolate R1 and R2
    t = ts(tick);
    p_t = (1-t)*p1 + t*p2;
    R_t = R1*rodrigues(w_link/norm(w_link),norm(w_link)*t); % interpolate
    
    % Append the trajectory
    x_traj = cat(1,x_traj, rv(p_t)+all*rv(R_t(:,1)));
    y_traj = cat(1,y_traj, rv(p_t)+all*rv(R_t(:,2)));
    z_traj = cat(1,z_traj, rv(p_t)+all*rv(R_t(:,3)));
    
    % Animate
    if (tick==1) || (mod(tick,2)==0) || (tick==length(ts))
        view_info = [80,16];
        fig = set_fig(figure(1),'pos',[0.0,0.5,0.5,0.4],...
            'view_info',view_info,'axis_info',axis_info,...
            'AXIS_EQUAL',1,'GRID_ON',1,'REMOVE_MENUBAR',1,'USE_DRAGZOOM',1,...
            'SET_CAMLIGHT',1,'SET_MATERIAL','METAL','SET_AXISLABEL',1,'afs',18); % make figure
        plot_T(pr2t(p1,R1),'fig_idx',1,'subfig_idx',1,...
            'PLOT_AXIS',1,'all',all,'alw',3,'alc','','PLOT_SPHERE',0,'text_str','R1','TEXT_AT_ZTIP',1,...
            'PLOT_AXIS_TIP',1); % R1
        plot_T(pr2t(p2,R2),'fig_idx',1,'subfig_idx',2,...
            'PLOT_AXIS',1,'all',all,'alw',3,'alc','','PLOT_SPHERE',0,'text_str','R2','TEXT_AT_ZTIP',1,...
            'PLOT_AXIS_TIP',1); % R2
        plot_T(pr2t(p_t,R_t),'fig_idx',1,'subfig_idx',3,...
            'PLOT_AXIS',1,'all',all,'alw',2,'alc','','PLOT_SPHERE',0,'PLOT_AXIS_TIP',1); % R_t
        plot_traj(x_traj,'fig_idx',1,'subfig_idx',2,'tlc','r','tlw',1,'tls','--');
        plot_traj(y_traj,'fig_idx',1,'subfig_idx',3,'tlc','g','tlw',1,'tls','--');
        plot_traj(z_traj,'fig_idx',1,'subfig_idx',4,'tlc','b','tlw',1,'tls','--');
        title_str = sprintf('[%d/%d] t:[%.3f]',tick,length(ts),t);
        plot_title(title_str,'tfs',25,'interpreter','latex');
        drawnow; if ~ishandle(fig), break; end
    end
end

%% Translate and rotate objects in 3D space
ccc
warning('off','MATLAB:hg:DiceyTransformMatrix'); % remove rendering warnings

% Local coordinates {A} in {W}
T_A_in_W = pr2t(rand(3,1),rpy2r(10*randn(3,1)*D2R));

% Point X in {A}
p_X_in_A = cv([1.0,0.5,0.2]);           % position in local coordinate system {A}

% Spatial velocity of {A}
omega_A_in_W    = 1*D2R;                % constant angular velocity w.r.t z-axis of T_A_in_W
v_A_in_W        = cv(-1/360*rand(1,3)); % directional velocity of {A} in {W}

% Loop
tick = 0; max_tick = 720; dt = 1; p_X_in_W_traj = []; arrow_cnt = 0;
while true
    tick = tick + 1;
    if tick <= max_tick
        % Get position and orientation
        [p_A_in_W,R_A_in_W] = t2pr(T_A_in_W);
        % Update position
        p_A_in_W = p_A_in_W + v_A_in_W*dt;
        % Update rotaton
        a_in_W = R_A_in_W(:,3); % z-axis to be the axis of rotation
        R_rot = rodrigues(a_in_W,omega_A_in_W*dt); % rotate w.r.t z-axis of {A}
        R_A_in_W = R_rot*R_A_in_W;
        % Update coordinate transform
        T_A_in_W = pr2t(p_A_in_W,R_A_in_W);
        
        % Point in the world coordinates {W}
        p_X_in_W = t2p(T_A_in_W*pr2t(p_X_in_A,''));
        % Compute the linear velocity of p_X in {W}
        w_A_in_W = r2w(rodrigues(a_in_W,omega_A_in_W)); % angular velocity vector of {A} (2.43)
        p_X_dot_in_W = v_A_in_W + cross(w_A_in_W,p_X_in_W-p_A_in_W); % (2.44)
        
        % Concat 'p_X_in_W'
        p_X_in_W_traj = cat(1,p_X_in_W_traj,rv(p_X_in_W));
    end
    
    % Animate
    if (tick==1) || (mod(tick,10)==0)
        view_info = [80,16];
        fig = set_fig(figure(1),'pos',[0.0,0.5,0.3,0.5],...
            'view_info',view_info,'axis_info',2.0*[-1,+1,-1,+1,-1,+1],...
            'AXIS_EQUAL',1,'GRID_ON',1,'REMOVE_MENUBAR',1,'USE_DRAGZOOM',1,...
            'SET_CAMLIGHT',1,'SET_MATERIAL','METAL','SET_AXISLABEL',1,'afs',18); % make figure
        plot_T(pr2t(cv([0,0,0]),eye(3,3)),'fig_idx',1,'subfig_idx',1,...
            'PLOT_AXIS',1,'all',1.0,'alw',2,'alc','','PLOT_SPHERE',0,...
            'text_str','~$\Sigma_W$','text_interp','latex','text_fs',25); % {W}
        plot_T(T_A_in_W,'fig_idx',1,'subfig_idx',2,...
            'PLOT_AXIS',1,'all',0.5,'alw',2,'alc','','PLOT_SPHERE',0,...
            'text_str','~$\Sigma_A$','text_interp','latex','text_fs',25); % local coordinates {A}
        plot_T(pr2t(p_X_in_W,''),'fig_idx',1,'subfig_idx',3,...
            'PLOT_AXIS',0,'PLOT_SPHERE',1,'sr',0.1,'sfc','r','sfa',0.8); % p_X in {W}
        plot_cube(T_A_in_W,cv([0,0,0]),p_X_in_A,'fig_idx',1,'subfig_idx',1,...
            'bfc','g','bfa',0.3,'bec','k'); % box at {A} illustraing p_X in {A}
        % Plot an arrow from {W} to {A}
        sw = 0.01; tw = 0.03; % stem width and tip width
        plot_arrow_3d('',t2p(T_A_in_W),'fig_idx',1,'subfig_idx',1,'color',[0,0,0],...
            'sw',sw,'tw',tw,'text_str','~$p_A$','text_fs',20,'text_color','k','interpreter','latex');
        % Plot the axis of rotation and directional velocity of {A}
        sw = 0.02; tw = 0.05;
        plot_arrow_3d(p_A_in_W,p_A_in_W+0.5*a_in_W,'fig_idx',1,'subfig_idx',2,'color','b',...
            'sw',sw,'tw',tw,'text_str','~Axis of Rotation','text_fs',15,'text_color','b','interpreter','latex');
        plot_arrow_3d(p_A_in_W,p_A_in_W+0.3*v_A_in_W/norm(v_A_in_W),'fig_idx',1,'subfig_idx',3,...
            'color','m','sw',sw,'tw',tw,'text_str','~Dir. Velocity','text_fs',15,'text_color','m','interpreter','latex');
        % Plot the velocity of the point in {A} w.r.t. {W}
        if mod(tick,30) == 0
            sw = 0.02; tw = 0.04;
            diff_W = 0.3*uv(p_X_dot_in_W);
            arrow_cnt = arrow_cnt + 1; % increase arrow connter
            plot_arrow_3d(p_X_in_W,p_X_in_W+diff_W,'fig_idx',1,'subfig_idx',3+arrow_cnt,...
                'color','r','sw',sw,'tw',tw);
        end
        % Plot the trajectory of 'p_X_in_W'
        plot_traj(p_X_in_W_traj,'fig_idx',1,'subfig_idx',1,'tlc','r','tlw',1,'tls','--'); % traj
        plot_title(sprintf('[%d]',tick),'tfs',25,'interpreter','latex');
        drawnow; if ~ishandle(fig), break; end
    end
    
    if tick == 1
        pause;
    end
end

%% Construct the kinematic chain
%
% Link parameters are specified in p.47.
% Here, we will use the followings:
%
% chain =
%   struct with fields:
%                name: 'kinematic_chain'
%                  dt: 0.0100
%               joint: [1×10 struct]
%         joint_names: {'world'  'J1'  'J2'  'J3'  'J4'  'J5'  'J6'  'EE'  'EE_R'  'EE_L'}
%             n_joint: 10
%     rev_joint_names: {'J1'  'J2'  'J3'  'J4'  'J5'  'J6'}
%         n_rev_joint: 6
%                link: [1×4 struct]
%          link_names: {'base_link'  'EE_link'  'EE_R_link'  'EE_L_link'}
%              n_link: 4
%
% chain.joint
% ans =
%   1×10 struct array with fields:
%     name
%     p
%     R
%     a
%     type
%     p_offset
%     R_offset
%     q
%     dq
%     ddq
%     q_diff
%     q_prev
%     v
%     vo
%     w
%     dvo
%     dw
%     u
%     ext_f
%     parent
%     childs
%     link_idx
%
% chain.link
% ans =
%   1×4 struct array with fields:
%     name
%     joint_idx
%     fv
%     box
%     bcube
%     capsule
%     box_added
%     v
%     vo
%     w
%     m
%     I_bar
%     com_bar
%
ccc

% Initialize a kinematic chain
chain = init_chain('name','kinematic_chain');

% Add joint to the chain
chain = add_joint_to_chain(chain,'name','world');
chain = add_joint_to_chain(chain,'name','J1','parent_name','world',...
    'p_offset',cv([0,0,0.5]),'a',cv([0,0,1]));
chain = add_joint_to_chain(chain,'name','J2','parent_name','J1',...
    'p_offset',cv([0,0,0.5]),'a',cv([1,0,0]));
chain = add_joint_to_chain(chain,'name','J3','parent_name','J2',...
    'p_offset',cv([0,0,0.5]),'a',cv([0,1,0]));
chain = add_joint_to_chain(chain,'name','J4','parent_name','J3',...
    'p_offset',cv([0,0,0.5]),'a',cv([0,0,1]));
chain = add_joint_to_chain(chain,'name','J5','parent_name','J4',...
    'p_offset',cv([0,0,0.5]),'a',cv([1,0,0]));
chain = add_joint_to_chain(chain,'name','J6','parent_name','J5',...
    'p_offset',cv([0,0,0.5]),'a',cv([0,1,0]));
chain = add_joint_to_chain(chain,'name','EE','parent_name','J6',...
    'p_offset',cv([0,0,0.2]),'a',cv([0,0,0]));
chain = add_joint_to_chain(chain,'name','EE_R','parent_name','EE',...
    'p_offset',cv([0,0.2,0]),'a',cv([0,0,0]));
chain = add_joint_to_chain(chain,'name','EE_L','parent_name','EE',...
    'p_offset',cv([0,-0.2,0]),'a',cv([0,0,0]));

% Add link to the chain
box_added = struct('xyz_min',[-2,-2,0],'xyz_len',[4,4,0.1],...
    'p_offset',cv([0,0,0]),'R_offset',rpy2r([0,0,0]*D2R),...
    'color',0.3*[1,1,1],'alpha',0.5,'ec','k');
chain = add_link_to_chain(chain,'name','base_link','joint_name','world','box_added',box_added);
box_added = struct('xyz_min',[-0.15,-0.3,-0.1],'xyz_len',[0.3,0.6,0.1],...
    'p_offset',cv([0,0,0]),'R_offset',rpy2r([0,0,0]*D2R),...
    'color','m','alpha',0.5,'ec','k');
chain = add_link_to_chain(chain,'name','EE_link','joint_name','EE','box_added',box_added);
box_added = struct('xyz_min',[-0.15,0,-0.1],'xyz_len',[0.3,0.1,0.4],...
    'p_offset',cv([0,0,0]),'R_offset',rpy2r([0,0,0]*D2R),...
    'color','m','alpha',0.5,'ec','k');
chain = add_link_to_chain(chain,'name','EE_R_link','joint_name','EE_R','box_added',box_added);
box_added = struct('xyz_min',[-0.15,-0.1,-0.1],'xyz_len',[0.3,0.1,0.4],...
    'p_offset',cv([0,0,0]),'R_offset',rpy2r([0,0,0]*D2R),...
    'color','m','alpha',0.5,'ec','k');
chain = add_link_to_chain(chain,'name','EE_L_link','joint_name','EE_L','box_added',box_added);

% Update chain mass, inertia, and com
chain = update_chain_mass_inertia_com(chain,'density',500);

tick = 0; ee_traj = [];
while tick < 1e4 % loop
    
    % Update
    tick = tick + 1;
    if tick <= 360
        q = 90*sin(2*pi*tick/360)*D2R;
        chain = update_chain_q(chain,chain.rev_joint_names,q*ones(1,chain.n_rev_joint));
        % Append end-effector trajectory
        % ee_traj = [ee_traj; rv(chain.joint(idx_cell(chain.joint_names,'EE')).p)];
    end
    
    % Animate
    if mod(tick,10) == 0
        fig = plot_chain(chain,'fig_idx',1,'subfig_idx',1,'fig_pos',[0.5,0.25,0.4,0.5],...
            'view_info',[68,16],'axis_info',2.5*[-1,+1,-1,+1,0,+1.5],'USE_ZOOMRATE',1,...
            'PLOT_LINK',1,'llc','k','llw',1,'lls','-',...
            'PLOT_JOINT_AXIS',1,'jal',0.3,'jalw',3,'jals','-',...
            'PLOT_JOINT_SPHERE',1,'jsr',0.05,'jsfc','k','jsfa',0.75,...
            'PLOT_ROTATE_AXIS',1,'ral',0.5,'rac','','raa',0.75,...
            'PLOT_JOINT_NAME',1 ...
            );
        plot_traj(ee_traj,'fig_idx',1,'subfig_idx',1,'tlc','m','USE_ZOOMRATE',1);
        plot_title(sprintf('[%d] Kinematic Chain',tick),'fig_idx',1,'tfs',26,'interpreter','latex');
        drawnow; if ~ishandle(fig), break; end
    end
end % while 1 % loop
fprintf('Done.\n');

%% Inverse Kinematics with Interactive Marker
%
% ik_info = init_ik_info(chain,...);
%
% ik_info = add_ik_info(ik_info,...);
% ik_info = add_ik_info(ik_info,...);
% ik_info = add_ik_info(ik_info,...);
% ...
% [dq,joint_names_to_ctrl] = one_step_ik(chain,ik_info);
% ...
% q = get_q_chain(chain,joint_names_to_ctrl);
% q = q + dq;
% chain = update_chain_q(chain,joint_names_to_ctrl,q);
%
ccc

% IK configuration
CONSIDER_JOINT_LIMIT = 1; % consider joint limit while solving IK

% Initialize robot
robot_name = 'iiwa7'; % 'coman', 'iiwa7'
chain = get_chain_from_urdf_with_caching(robot_name,'RE',0,'SKIP_CAPSULE',0);
chain = add_joi_to_robot(chain);
chain.joi = get_joi_chain(chain);
chain.sc_checks = get_sc_checks(chain,'collision_margin',max(chain.sz.xyz_len)/100);
T_joi = get_t_joi(chain,chain.joi);

switch robot_name
    case 'coman'
        joi_ik_trgts = {'rh','lh','re','le'}; % specify IK targets in terms of JOI
        joi_ik_types = {'IK_P','IK_P','IK_P','IK_P'};
        ik_weights   = {1,1,1,1};
    case 'iiwa7'
        joi_ik_trgts = {'rh'};
        joi_ik_types = {'IK_PR'};
        ik_weights   = {1};
    otherwise
        joi_ik_trgts = {};
        joi_ik_types = {};
        ik_weights   = {};
end

% Initialize IK targets
ik_info = init_ik_info(chain,...
    'joint_names_to_ctrl',chain.rev_joint_names,...
    'ik_err_th',1.0,...
    'dq_th',50*D2R,...
    'step_size',0.2,...
    'lambda_rate',0.001,...
    'lambda_min',1e-6,...
    'lambda_max',0.1 ...
    );
for ik_idx = 1:length(joi_ik_trgts) % append IK targets
    joi_ik_trgt = joi_ik_trgts{ik_idx};
    joi_ik_type = joi_ik_types{ik_idx};
    joint_idx = chain.joi.idxs(idx_cell(chain.joi.types,joi_ik_trgt));
    joint_name = chain.joint_names{joint_idx};
    % Add information information
    ik_info = add_ik_info(ik_info,...
        'joint_name',joint_name,...
        'type',joi_ik_type,...
        'weight',ik_weights{ik_idx},...
        'coord',getfield(T_joi,joi_ik_trgt)...
        );
end

% Animate
plot_chain(chain,'fig_idx',1,'fig_pos',[0.5,0.4,0.5,0.6],'mfc','','axis_info',chain.axis_info);
axis off;
for ik_idx = 1:ik_info.n_trgt
    plot_interactive_marker('fig_idx',1,'subfig_idx',ik_idx,...
        'T',ik_info.trgt_coords{ik_idx},'clen',0.2,'sr',0.02);
end
tick = 0;
while 1 % loop
    
    % Update IK target coordinates from interactive markers
    tick = tick + 1;
    for ik_idx = 1:ik_info.n_trgt
        T_i = g_im{ik_idx}.T;
        ik_info.trgt_coords{ik_idx} = T_i;
    end
    
    % Compute dq with one-step IK
    [dq,joint_names_to_ctrl,J_use,ik_err,det_J] = one_step_ik(chain,ik_info,...
        'CONSIDER_JOINT_LIMIT',CONSIDER_JOINT_LIMIT,...
        'UNIT_DQ_HEURISTIC',0,'unit_dq_rad',2*D2R);
    
    % Update chain with dq computed from IK
    q = get_q_chain(chain,joint_names_to_ctrl);
    q = q + dq;
    chain = update_chain_q(chain,joint_names_to_ctrl,q,'IGNORE_LIMIT',0);
    
    % Animate robot with IK targets using interactive markers
    plot_every = 5;
    if mod(tick,plot_every) == 0
        fig = plot_chain(chain,'fig_idx',1,'cfc','');
        ik_plot_info = get_ik_plot_info_from_ik_info(ik_info);
        plot_ik_targets('chain_robot',chain,'ik_plot_info',ik_plot_info,'sr',0.02,...
            'PLOT_AXIS',0,'all',0.2,...
            'PLOT_ARROW',0,'adl',0.2,'adsw',0.02,'adtw',0.04);
        if CONSIDER_JOINT_LIMIT
            title_str = sprintf(['[Consider Joint Limit] Tick:[%d] IK error:[%.3f]',...
                '\n(Press [t]:toggle [q]:quit)'],...
                tick,norm(ik_err));
            plot_title_with_text(title_str,'fig_idx',1,'tfs',20,'tfc','b','interpreter','latex');
        else
            title_str = sprintf(['[Naive IK] Tick:[%d] IK error:[%.3f]',...
                '\n(Press [t]:toggle [q]:quit)'],...
                tick,norm(ik_err));
            plot_title_with_text(title_str,'fig_idx',1,'tfs',20,'tfc','k','interpreter','latex');
        end
        drawnow; if ~ishandle(fig), break; end
    end
    
    % Toggle 'CONSIDER_JOINT_LIMIT' with keyboard inputs
    % Keyboard handler
    if ~isempty(g_key) % if key pressed
        switch g_key
            case 'q'       % press 'q' to quit
                break;
            case 't'       % press 't' to toggle 'CONSIDER_JOINT_LIMIT'
                CONSIDER_JOINT_LIMIT = ~CONSIDER_JOINT_LIMIT;
        end
        g_key = ''; % reset key pressed
    end % if ~isempty(g_key) % if key pressed
    
end % while 1 % loop
if ishandle(fig)
    plot_title_with_text('Terminated','fig_idx',1,'tfs',15,'tfc','r');
end
fprintf('Done.\n');

%%















