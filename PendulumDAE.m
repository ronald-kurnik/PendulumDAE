%% ----------------------------------------------------------------
% 2. Simple index-1 DAE: pendulum in Cartesian coordinates
%    x'' = -λ x ,  y'' = -λ y - g
%    algebraic: x^2 + y^2 = L^2
% ----------------------------------------------------------------
fprintf('=== Index-1 DAE (pendulum) ===\n');

g = 9.81; L = 1.0;
% State: [x, y, vx, vy, λ]
% We use a simple stabilized index-1 formulation + explicit Euler on differential
% part with projection (demo only; production uses proper DAE solvers)

dt = 1e-3;
T  = 5;
t  = 0:dt:T;
N  = length(t);
X  = zeros(5,N);

% IC: hanging almost horizontal
theta0 = pi/2 - 0.1;
X(:,1) = [L*sin(theta0); -L*cos(theta0); 0; 0; 0];

for n = 1:N-1
    x = X(1,n); y = X(2,n); vx = X(3,n); vy = X(4,n);
    % Lagrange multiplier from constraint acceleration
    % 2(x vx + y vy) = 0  (velocity level)
    % differentiate again → λ expression
    lam = (vx^2 + vy^2 - g*y) / L^2;   % approx for length constraint
    ax  = -lam * x;
    ay  = -lam * y - g;

    % Explicit Euler
    X(1,n+1) = x  + dt*vx;
    X(2,n+1) = y  + dt*vy;
    X(3,n+1) = vx + dt*ax;
    X(4,n+1) = vy + dt*ay;
    X(5,n+1) = lam;

    % Simple projection back onto constraint
    r = sqrt(X(1,n+1)^2 + X(2,n+1)^2);
    X(1:2,n+1) = X(1:2,n+1) * (L/r);
end

figure('Name','Pendulum DAE','Position',[100 100 800 400]);
subplot(1,2,1);
plot(X(1,:), X(2,:), 'b-'); axis equal; grid on;
xlabel('x'); ylabel('y'); title('Trajectory');
subplot(1,2,2);
plot(t, sqrt(X(1,:).^2+X(2,:).^2), 'r'); hold on;
yline(L,'k--');
xlabel('t'); ylabel('length'); title('Constraint drift (should stay near L)');
grid on;
