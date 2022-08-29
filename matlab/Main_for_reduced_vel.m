% simulation example of ACC 2020
% multiple clusters tracking multiple targets using ADP and approximate
% control

rng(622); % Sets a fixed seed for the random number generation. Code works fine for different seeds
close all;
clear all;

global U ;

%% Initialze colors for the plots
colors = ['r', 'k', 'g', 'b', 'c']; % For plotting purpose only
shapes = ['o', 'x', 'd', '*','s']; % For plotting purpose only

%% The next section sets up lat,lon coordinates for an example scenarions. You do NOT need to have them set up correctly to generate the correct gain matrix
% Havlock base
%base_lat = 40.8522108472926 ;
%base_lon = -96.60822380671102 ;

% % % % QP new base  
base_lat = 40.8328148 ;
base_lon = -96.6957025 ;

%QP example target locations
target_lat_ag1 = 40.8327215 ;
target_lon_ag1 = -96.6958784 ;

target_lat_ag2 = 40.8328707;
target_lon_ag2 = -96.6959139 ;

% Distance between the base and the targets
target_xm_2 = lon_to_meters(base_lat,base_lon,base_lat,target_lon_ag2);
target_ym_2 = lat_to_meters(base_lat,base_lon,target_lat_ag2,base_lon);

target_xm = lon_to_meters(base_lat,base_lon,base_lat,target_lon_ag1);
target_ym = lat_to_meters(base_lat,base_lon,target_lat_ag1,base_lon);

%% Initialize the swarm
learning = 1; % This is to enable the learning component 1: enable 0: disable
Nc = 2; % number of clusters. Important: this variable is heavily used in the code.

p(:,1) = [3 1]';

N = sum(p); % total number of agents
Nt = Nc; % number of targets
ndim = 2; % 2D/3D agents 
r = 2; % The radius of the formation circle for each cluster

tg_loc = [target_xm target_ym 5 ; target_xm_2 target_ym_2 5]; % Set the distances to the targets
tg_loc = [10 10 5 ; 10 -10 5]; % This is for test purposes


%% The commented section creates a random graph. We do not need that. I changed it. We assume full connectivity.
% Network design
if Nc ~= 1
%     chk = 0;
%     while chk==0
%         Adj = random_graph(Nc,0.1); % This randomly creates a graph with a 0.1 probability of two agents being connected.
%         chk = isconnected(Adj); % Check if the random graph is connected. Run the loop util the the previous step generates a connected graph
%     end
    Adj = ones(Nc); % creates a fully connected network instead. Agents are connected to themselves
    Deg = diag(Adj*ones(Nc,1)); % Creates the degree matrix ( along the diagnal it provides the number connections one node has )
    Lap = Deg - Adj;% laplacian matrix between the clusters
else
    Lap = 1;
end


%% create the transformation matrics for each cluster
T = [];
for j = 1:Nc % For each cluster
    Ncj = p(j); % number of agents in each cluster/group. Both the terms are used to mean the same thing
    trans = [-ones(Ncj-1,1) eye(Ncj-1);1/Ncj*ones(1,Ncj)];% last row is the average and all the other rows are differences. Right now it's not obvious why it is created like that. But it is used to calculate the Kroneker product, AND to create the transformed states later in the code
    TT{j} = kron(trans, eye(2*ndim)); %2*ndim: position + velocity
    % TT{j} = blkdiag(TT, eye(ndim*Ncj)); % added the integral control
    T = blkdiag(T, TT{j}, eye(ndim*Ncj)); % not transforming the integral state
%     T = blkdiag(T, TT{j}, blkdiag(eye(Ncj),ones(Ncj)1)); % not transforming the integral state
    
end
   
% create the desired states within each cluster 
ZD = []; % To store some future value 
S = []; % used later
for j = 1:Nc %For each cluster / group
    Ncj = p(j); % number of agents in each cluster
    zd{j} = zeros(2*ndim*Ncj,1); % 2 : state and its derivative. ndim = 2 : x-y dimensions only
    ang = 2*pi/Ncj; % calculate the angle so that each agent has an equal 
    for i = 1:Ncj-1 % Why n-1 agents ? 
        if ndim == 3 
            zd{j} = [zd{j}; [r*cos(ang*i) r*sin(ang*i) 0 ]' - [r 0 5]']; % desired relative positions
        else
            zd{j} = [zd{j}; [r*cos(ang*i) r*sin(ang*i)]' - [r 0]']; % desired relative positions
        end
            
    end
%     zd{j} = [zd{j}; [tg_loc(j,1:ndim)]']; % add the target location as the desired location for the average state
    % This is to take out global target from the integral state.
    zd{j} = [zd{j}; zeros(1,ndim)']; % add the target location as the desired location for the average state

    ZD = [ZD; zd{j}]; %ZD is the offset signal in the system dynamics (see agent_dynamics.m)
    Sj = [zeros(ndim,2*ndim*(Ncj-1)) eye(ndim) zeros(ndim,ndim) zeros(ndim,ndim*Ncj)]; % select the average state if multiplied with the transformed states
    S = blkdiag(S, Sj); % record everthing in a block diagnol matrix, why not... :-/ 
end

%% crea\te the optimal control objectives for each cluster
R = [];
Q = []; % This Q is in the transformed space. 

% Transformed space for a group with 2 agents is below. Assume the agnets
% are two dimensional and x is position and \dot(x) is velocity
%      x2 - x1                         -|
%      y2 - y1                         -| Formation at the beginning
% \dot(x2) - \dot(x1)                  -|
% \dot(y2) - \dot(y1)                  -| Velocity control at the beginning
%    (x2 + x1)/2                       -|
%    (y2 + y1)/2                       -| Track the target
% ( \dot(x2) + \dot(x1) )/2            -| 
% ( \dot(y2) + \dot(y1) )/2            -| Make the velocity at the taregt 0
% x2 - x1 - x_inter_agent_distance     -|
% y2 - y1 - y_inter_agent_distance     -| Integrals for formation 
% (x2 + x1)/2 - x_target               -| 
% (y2 + y1)/2 - y_taregt               -| Integrals for tracking

% below three were the original values
% q_formation = 0.1; % prioratize getting into the formation
% q_target = .1; % prioratize getting to the target
% q_integral = 0.1; % prioratize the integral term on position

 q_formation = 10; % prioratize getting into the formation. Corresponds to the first 4 transformed states
 q_target_track_position = 0.01 ; % Corresponds to the 5th and 6th transformed states
 q_target_track_velocity = 0 ; % Corresponds to the 7th and 8th transformed states

 q_integral_for_formation = 10 ; % Corresponds to the 9th and 10th transformed states
 q_integral_for_tracking = 0.01 ; % Corresponds to the 11thth and 12th transformed states

for j = 1:Nc
    Ncj = p(j); % number of agents in each group / cluster
    
    % Gains are arranged to a proper matrix formation for multiplication
    q_target = [ q_target_track_position*ones(length(blkdiag(eye(ndim), zeros(ndim,ndim)))/2,1) ; q_target_track_velocity*ones(length(blkdiag(eye(ndim), zeros(ndim,ndim)))/2,1) ] ;
    q_integral = [ q_integral_for_formation*ones(ndim*Ncj/2,1) ; q_integral_for_tracking*ones(ndim*Ncj/2,1)];

    nstate = Ncj * 3 * ndim; % 2*ndim*Ncj: position + velocity for Ncj agents; ndim*Ncj: integral states for the position
    nu = Ncj * ndim; % each agent has ndim controls
    Rj{j} = eye(nu); % This is an identity matrix prioratizing the control input. pay attention to the dimensions
    Qj{j} = blkdiag(q_formation*(kron(eye(Ncj-1),blkdiag(eye(ndim), zeros(ndim,ndim)))), blkdiag(eye(ndim), ones(ndim,ndim)).*q_target, eye(ndim*Ncj).*q_integral); % Generate a part of the Q matrix
    Q = blkdiag(Q, Qj{j}); % Store in block diagnol matrix
    R = blkdiag(R, Rj{j});
end
% add coupling between clusters in Q
tildeQ = S'*kron(Lap, eye(ndim))*S * 1;
Qbar = Q;
Q = Qbar + tildeQ; % Need to understand bit more what's happening here

%% generate agent dynamics A_i, B_i and the rotated dynamics
A = [];
B = [];
xo =[]; %zeros(ndim*N*2,1);
% s = cumsum(p);
% group = zeros(N,1);
TA = [];
TB = [];
Txo = [];
TG = [];
epsilon = .1; % small integral feedback gain for initial stabilizing control
%TG  = [];
for i = 1:Nc % go through all the clusters
    Ac{i} = [];
    Bc{i} = [];
    xoc{i} = [];
    G{i} = [];
    Ncj = p(i);
    center = randn(ndim,1);
    for j = 1:Ncj % go through each agent in each cluster (Same agent dynamics within each cluster)
        mi = 1.4; %mass
        ci = 1.25; % damping coefficient to model the drag and other resisiting forces.
%         ci = 0; % damping coefficient, may be to model the drag and other resisiting forces. Why not model the same, but changes with i? may be to say each system i non-homogenic
        Ai = [zeros(ndim,ndim) eye(ndim);zeros(ndim,ndim) -ci/mi*eye(ndim)]; % including the integral state
        Bi = [zeros(ndim,ndim);1/mi*eye(ndim)];
        Gi = [-5*eye(ndim) -5*eye(ndim)];% initial stabilization control
        xoi = [3*randn(ndim,1); 0.5*randn(ndim,1)]; % starting location for each is randomly selected, but why 4 elements. do they initialize \dot_x \dot_y as well ?
        Ac{i} = blkdiag(Ac{i}, Ai);
        Bc{i} = blkdiag(Bc{i}, Bi);
        xoc{i} = [xoc{i};xoi];
        G{i} = blkdiag(G{i}, Gi);
    end
    [~,nui] = size(Bc{i});
    nsi = length(Ac{i});
    
    % The next section calulates the transformed states (eq 31 in the paper)
    TAi{i} = TT{i} * Ac{i} * inv(TT{i});
    TBi{i} = TT{i} * Bc{i};
    TGi{i} = G{i} * inv(TT{i});
    Mi = pinv(TBi{i})*kron(eye(Ncj), [zeros(ndim,ndim); eye(ndim)]);% feedback gain matrix for integral control to ensure the correct feedback form
    TAi{i} = [TAi{i} zeros(2*ndim*Ncj, ndim*Ncj);kron(eye(Ncj), [eye(ndim) zeros(ndim)]) zeros(ndim*Ncj, Ncj*ndim)]; % add integral state to the transformed dynamics

    TBi{i} = [TBi{i}; zeros(Ncj*ndim,Ncj*ndim)];
    Txoi{i} = [TT{i}*xoc{i};zeros(Ncj*ndim,1)];
    TGi{i} = [TGi{i} -epsilon*Mi];
    TA = blkdiag(TA, TAi{i});
    TB = blkdiag(TB, TBi{i});
    Txo = [Txo; Txoi{i}];
    TG = blkdiag(TG, TGi{i});

end
nu = ndim*N;
ns = ndim*3*N;
disp('--------------');
disp('check the closed-loop eigenvalues with the initial stabilizing gain G');
disp('--------------');
eig(TA+TB*TG)% check to see if the feedback gain TG is stabilizing
pause(0.1);


%% formulate the optimal control. We don't need this
%[K,S,E] = lqr(TA,TB,Q,R);
Po = care(TA, TB, Q, R);
K = inv(R)*TB'*Po;
%% form the approximate control. We don't need this
Pa = care(TA,TB,Qbar,R);
a = Pa*TB;
b = TB'*Pa;
tildeR = pinv(a)*tildeQ*pinv(b);
Ka = (inv(R)+tildeR)*TB'*Pa; % approximate control
% % Ka = inv(R)*TB'*P;

%% learning control (non-working version) (This non working comment is from the original matlab code)
xfinal = [];
% figure;
if learning
    dT = 0.01;
    tf = [30 15 15 6];
    PL = [];
    KL = [];
    for i = 1:Nc % learn through each cluster
        tL = [0:dT:tf(i)];
        A = TAi{i};
        B = TBi{i};
        G = TGi{i};
        xn = length(A);
        [~, un] = size(B);
        Ncj = p(i); % number of agents
        pvo = [];
        for j = 1:Ncj
            pvo = [pvo;10*randn(ndim,1);3*randn(ndim,1)];
        end
        io = zeros(ndim*Ncj,1);
        
        %% TODO : What if we set the interatcions only on x and y axis. 
        xo = [randn(3*ndim*Ncj,1); zeros(xn^2 + xn * un,1)]';
        expl_noise_freq = (rand(un,100)-.5)*1000;
        D = xo;
        for k = 1:length(tL)
            [t, xo] = ode45(@agent_dynamics_learning_reduced_vel, [(k-1)*dT, k*dT], xo(end,:),'',A,B,G,expl_noise_freq);
            D = [D;xo(end,:)];
        end

        %   K0 = lqr(A,B,0.01*eye(xn), eye(un)); % initial control has to be stabilizing, not quite satisfied with how this is generated!!
        [Ki,Pi]=adp_sim1(D,xn,un,Qj{i},Rj{i},-G,0);
        Koi = lqr(A,B,Qj{i},Rj{i});
        KL = blkdiag(KL, Ki);
        PL = blkdiag(PL, Pi);
        disp('--------------');
        disp('check the difference between the learned controller and the optimal controller');
        disp('--------------');
        norm(Koi-Ki)
%         pause(0.1);
        xf{i} = xo(end,1:length(A))';
        xfinal = [xfinal; xf{i}];
    end
    KL = KL + (tildeR)*R*KL;
end

%% Set up the vector to change the centroids to errors here
aug_state = [];
for i = 1:Nc
 
    Ncj = p(i);
    vector = zeros(1, (Ncj-1)*ndim*2 + ndim*2 + (Ncj-1)*ndim+ ndim);
    vector((Ncj-1)*ndim*2 + 1) = tg_loc(i,1);
    vector((Ncj-1)*ndim*2 + 2) = tg_loc(i,2);
    
    aug_state = [ aug_state vector ];
    
end

%% Set up initial conditions here
xStart = zeros(N*6,1);
disp(xStart);

% The next line is import. This transforms the real worl coordinates to
% trannsformed coordinates. The algorithm works on the transformed
% coordinates.
xStart = T*xStart ;
heights = [10] ;

%% solve the ode
delat_t = 0.25 ;
tspan = [0:delat_t:20];
int_size = ndim*N; %state dim for the integral state

% simulate with the optimal control in the transformed coordinates
if learning
% %     [t_ell,x_ell] = ode45(@agent_dynamics_reorg, tspan, xStart,'',ZD, TA, TB, -KL,int_size,aug_state);
% %     XL = (inv(T) * x_ell')';
    x_current = xStart ;
    X_collector = [] ;
    for tk = 1:length(tspan)
        
        X_collector = [ X_collector ; x_current' ] ; % records states

        u = -1*KL*(x_current-aug_state') ; % calculate the control input
          
        % The following section adds bounds to the maximum thrust the the
        % control input can have. Same limitation is applied to the
        % training. 
        
        sum_of_forces = sqrt(abs(u(1))^2 + abs(u(2))^2) ;
        max_force = 6.5 ;
        
        if(sum_of_forces) > max_force 
%             disp("forces exceed at simulation");
            theta = atan(abs(u(2))/abs(u(1))) ;   % Angel of the resultant force  

            u(1) = max_force * cos(theta) * sign(u(1)); % Set the maximum bounds based on the angel
            u(2) = max_force * sin(theta) * sign(u(2));    

        end
        
        % Calculate the discrete state propergation
        x_k_plus_one = ( TA*delat_t + eye(length(TA)) )*(x_current-aug_state') + TB*delat_t*u  - ZD*delat_t + aug_state';
        
        x_current = x_k_plus_one ;       
        
    end 
    % Record the states
    XL = (inv(T)*X_collector')' ;
    
end
%% plotting

%% Plot trajectories
linestyles = {'-','-.','--'};
figure(3);
title('Trajectories of the agents');
% agents' positions
if ndim == 3
    for j = 1:Nc
        Ncj = p(j);
        if j == 1
            pos_s = 0;
        else
            pos_s = sum(p(1:j-1))*3*ndim;
        end
        for i = 1:Ncj
            % plot trajectory
            if learning
                plot3(XL(1:end,pos_s + 2*ndim*(i-1)+1), XL(1:end,pos_s + 2*ndim*(i-1)+2), XL(1:end,pos_s + 2*ndim*(i-1)+3),[colors(j) linestyles{2}],'linewidth',2);
            end
            hold on ;
            % plot final position
            if learning
                plot3(XL(end,pos_s + 2*ndim*(i-1)+1), XL(end,pos_s + 2*ndim*(i-1)+2), XL(end,pos_s + 2*ndim*(i-1)+3),[colors(j) shapes(2)],'linewidth',1);
            end
            hold on ;
            % plot initial position
            plot3(XL(1,pos_s + 2*ndim*(i-1)+1), XL(1,pos_s + 2*ndim*(i-1)+2), XL(1,pos_s + 2*ndim*(i-1)+3) , [colors(j) shapes(2)],'linewidth',1);

        end
    end
elseif ndim == 2
    
    for j = 1:Nc
        Ncj = p(j);
        if j == 1
            pos_s = 0;
        else
            pos_s = sum(p(1:j-1))*3*ndim;
        end
        for i = 1:Ncj
            % plot trajectory
            if learning
                plot(XL(1:end,pos_s + 2*ndim*(i-1)+1), XL(1:end,pos_s + 2*ndim*(i-1)+2) ,[colors(j) linestyles{1}],'linewidth',2);
            end
            hold on ;
            % plot final position
            if learning
                plot(XL(end,pos_s + 2*ndim*(i-1)+1), XL(end,pos_s + 2*ndim*(i-1)+2),[colors(j) shapes(2)],'linewidth',1);
            end
            hold on ;
            % plot initial position
            plot(XL(1,pos_s + 2*ndim*(i-1)+1), XL(1,pos_s + 2*ndim*(i-1)+2) , [colors(j) shapes(2)],'linewidth',1);

        end
    end
    
end
hold on ;
% target locations
if ndim == 3
    for i = 1:Nt
        plot3(tg_loc(i,1), tg_loc(i,2), tg_loc(i,3) , [colors(i) '+'], 'MarkerSize',15,'Linewidth',5);
    end

elseif ndim == 2
     for i = 1:Nt
        plot(tg_loc(i,1), tg_loc(i,2) , [colors(i) '+'], 'MarkerSize',15,'Linewidth',5);
     end
end
% axis equal;
grid on;
xlabel('x (meters)','FontSize',20);
ylabel('y (meters)','FontSize',20);
zlabel('z (meters)','FontSize',20);
ax = gca;

%% Save XL to be simulated in Frejya sim
% save('XL.mat','XL')

% Save sys matrix  to be simulated in Frejya sim
% save('SYS.mat','TA','TB','TG','T');

%% SAVE sys mats on a json object
strct_for_mats.TA = TA ;
strct_for_mats.TB = TB ;
strct_for_mats.TG = KL ;
strct_for_mats.T = T ; 
strct_for_mats.AUG = aug_state' ;
strct_for_mats.TGT = ZD ;

% convert to a json object
strct_to_jason = jsonencode(strct_for_mats);

%save to file 
fid = fopen('../swarm-nodes/config/sys_mats_4ag_QP_31.json','wt');
fprintf(fid,strct_to_jason);
fclose(fid);


figure;
plot(1:length(U),U);

%% plot control inputs
linestyles = {'-','-.','--'};
% U = (-K*X')';
UL = (-KL*XL')';
figure;
hold on;
% agents' positions
for j = 1:Nc
    subplot(2,2,j);
    Ncj = p(j);
    if j == 1
        pos_s = 0;
    else
        pos_s = ndim*Ncj;
    end
    
    for i = j
        % plot trajectory
%         plot(t, U(1:end,pos_s + ndim*(i-1)+1), [colors(1) linestyles{1}],'linewidth',2);
%         hold on;
%         plot(t, U(1:end,pos_s + ndim*(i-1)+2), [colors(2) linestyles{1}],'linewidth',1);
%         
        %plot(Xa(1:end,pos_s + 2*ndim*(i-1)+1), Xa(1:end,pos_s + 2*ndim*(i-1)+2), [colors(j) linestyles(2,:)],'linewidth',2);
        if learning
            plot(t_ell, UL(1:end,pos_s + ndim*(i-1)+1), [colors(1) linestyles{2}],'linewidth',2);
            plot(t_ell, UL(1:end,pos_s + ndim*(i-1)+2), [colors(2) linestyles{2}],'linewidth',1);
        end
    end
    axis auto;
    grid on;
    xlabel('t (seconds)','FontSize',20);
    ylabel(['u_' num2str(i) '^' num2str(j)],'FontSize',20);
    ax = gca;
    ax.FontSize=20;
%   legend(['optimal control: u_' num2str(i) '^' num2str(j)], ['optimal control: u_' num2str(i) '^' num2str(j)], ...
%        ['learned approximate control: u_' num2str(i) '^' num2str(j)], ['learned approximate control: u_' num2str(i) '^' num2str(j)],'location','best');
%     legend(['optimal control: 1st component'], ['optimal control: 2nd component'], ...
%      ['learned approximate control: 1st component'], ['learned approximate control: 2nd component'],'location','best');

%     pbaspect([1.62 1 1]);
end
% BaseLat, BaseLon = 40.851622360702855, -96.60921604026855 ;


function d = gps_to_meters(lat1,lon1,lat2,lon2)

    r = 6378137 ; % radius of the earth in meters
    
    dLat = lat2*pi/180 - lat1*pi/180 ;
    dLon = lon2*pi/180 - lon1*pi/180 ;
    
    a = sin(dLat/2)*sin(dLat/2) + cos(lat1*pi/180)*cos(lat2*pi/180)*sin(dLon/2)*sin(dLon/2) ;
    c = 2*atan2(sqrt(a),sqrt(1-a));
    d = r*c ;

end

function [t_lat,t_lon] = meters_to_lat(start_lat,start_lon,distance)

    r = 6378137 ; % radius of the earth in meters

    d = distance ;
    
    base_lat_rads = start_lat*pi/180 ;
    base_lon_rads = start_lon*pi/180 ;
    
    bearing = 0*pi/180 ;
    
    t_lat = asin(sin(base_lat_rads)*cos(d/r) + cos(base_lat_rads)*sin(d/r)*cos(bearing));
    t_lon = base_lon_rads + atan2(sin(bearing)*sin(d/r)*cos(base_lat_rads),cos(d/r)-sin(base_lat_rads)*sin(t_lat));
 
    t_lat = t_lat*(180/pi);
    t_lon = t_lon*(180/pi);
end

function [t_lat,t_lon] = meters_to_lon(start_lat,start_lon,distance)
    
    r = 6378137 ; % radius of the earth in meters

    d = distance ;
    
    base_lat_rads = start_lat*pi/180 ;
    base_lon_rads = start_lon*pi/180 ;
    
    bearing = 90*pi/180 ;
    
    t_lat = asin(sin(base_lat_rads)*cos(d/r) + cos(base_lat_rads)*sin(d/r)*cos(bearing));
    t_lon = base_lon_rads + atan2(sin(bearing)*sin(d/r)*cos(base_lat_rads),cos(d/r)-sin(base_lat_rads)*sin(t_lat));
 
    t_lat = t_lat*(180/pi);
    t_lon = t_lon*(180/pi);
    
    

end

function d = lon_to_meters(lat1,lon1,lat2,lon2)

    r = 6378137 ; % radius of the earth in meters
    
    dLat = lat2*pi/180 - lat1*pi/180 ;
    dLon = lon2*pi/180 - lon1*pi/180 ;
    
    a = sin(dLat/2)*sin(dLat/2) + cos(lat1*pi/180)*cos(lat2*pi/180)*sin(dLon/2)*sin(dLon/2) ;
    c = 2*atan2(sqrt(a),sqrt(1-a));
    d = r*c ;
    
    % check for the sign
    if(lon2 < lon1)
       d = d*-1 ; 
    end
    
end

function d = lat_to_meters(lat1,lon1,lat2,lon2)

    r = 6378137 ; % radius of the earth in meters
    
    dLat = lat2*pi/180 - lat1*pi/180 ;
    dLon = lon2*pi/180 - lon1*pi/180 ;
    
    a = sin(dLat/2)*sin(dLat/2) + cos(lat1*pi/180)*cos(lat2*pi/180)*sin(dLon/2)*sin(dLon/2) ;
    c = 2*atan2(sqrt(a),sqrt(1-a));
    d = r*c ;
    
    % check for the sign 
    
    if(lat2 < lat1)
        d = d*-1 ;
    end
    
end

