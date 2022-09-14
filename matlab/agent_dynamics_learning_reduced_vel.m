function dot = agent_dynamics_learning_reduced_vel(t,x,A,B,G,expl_noise_freq)
ns = length(A);
[~,nu] = size(B);
X = x(1:ns,1);
u = G*X + (1/4)*sum(sin(expl_noise_freq*t),2);

%% following checks if the sum of the forces are less than 11.5N. if not limits to 11.5N
max_force = 6.5 ;
sum_of_forces = sqrt(abs(u(1))^2 + abs(u(2))^2) ;

if(sum_of_forces) > max_force 

    % This calculates the maximum 
%     disp("forces exceed at training");
    theta = atan(abs(u(2))/abs(u(1))) ;    
    
    u(1) = max_force * cos(theta) * sign(u(1));
    u(2) = max_force * sin(theta) * sign(u(2));    

end

dot(1:ns,1) = A*X + B*u ;
dot(ns+1:ns + ns^2,1) = kron(X,X);
dot(ns + ns^2 + 1: ns + ns^2 + ns*nu,1) = kron(X,u);
