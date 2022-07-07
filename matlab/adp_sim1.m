
function [K,P]=adp_sim1(X,xn,un,Q,R,K0,display)
% Generic ADP to run with simulator - Sayak Mukherjee
% from the simulator store x(t), integral of kron(x,x) and integral of kron(x,u) in X
% rows of X are time series and columns correspond to [x,int(kron(x,x)),int(kron(x,u))] 
% xn and un are dimensions of state vector and control inputs

Dxx=[];
Ixx=[];
Ixu=[];
% xn=20; % dim of x
% un=5;  % dim of u
Kinit=K0; % if this doesn't work then use a stabilizing initial K

% X is from workspace
X1=X(1:end,:);
n=size(X1,1);
for i=2:n
    j=i-1;
Dxx=[Dxx;kron(X1(i,1:xn),X1(i,1:xn))-kron(X1(j,1:xn),X1(j,1:xn))];
Ixx=[Ixx;X1(i,xn+1:xn+xn^2)-X1(j,xn+1:xn+xn^2)];
Ixu=[Ixu;X1(i,xn+xn^2+1:end)-X1(j,xn+xn^2+1:end)];
end
% rank([Ixx Ixu])
% xn*(xn+1)/2+xn*un
% pause
P_old = zeros(xn);
P = eye(xn);    % Initialize the previous cost matrix
it = 0;            % Counter for iterations
p_save = [];       % Track the cost matrices in all the iterations
k_save = [];       % Track the feedback gain matrix in each iterations
MaxIteration = 20; 
% Qdelta=[4,-1,-1,-1,-1;-1,4,-1,-1,-1;-1,-1,4,-1,-1;-1,-1,-1,4,-1;-1,-1,-1,-1,4]
% Q=1*blkdiag(5*Qdelta,2*eye(10),.1*eye(5)); R=1*eye(un);
K=Kinit;
%% Off-policy learning using the collected online data
while norm(P-P_old)>1e-8 & it<MaxIteration*10
    it = it+1;                        % Update and display the # of iters
    P_old = P;                        % Update the previous cost matrix
    QK = Q+K'*R*K;                    % Update the Qk matrix
    
    Theta = [Dxx,-Ixx*kron(eye(xn),K')-Ixu];  % Left-hand side of
    % the key equation
    Xi = -Ixx*QK(:);                 % Right-hand side of the key equation
    pp = pinv(Theta)*Xi;             % Solve the equations in the LS sense
    P = reshape(pp(1:xn*xn), [xn, xn]);  % Reconstruct the symmetric matrix
    P = (P + P')/2;
    
    BPv = pp(end-(xn*un-1):end);
    K = inv(R)*reshape(BPv,un,xn)/2;% Get the improved gain matrix
%     p_save = [p_save,norm(P-Popt)];   % Keep track of the cost matrix
%     k_save = [k_save,norm(K-Kopt)];    % Keep track of the control gains
    if display
        disp(['K_', num2str(it), '=']);
        disp(K);
    end
end
if display
    if it>=MaxIteration*5 && norm(P-P_old)>1e-8
        warning('not converging');
    end
end
