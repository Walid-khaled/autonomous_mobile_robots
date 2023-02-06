clc
clear
close all
Ts = 0.033;
t = 0:Ts:30;
q = [0; 0; 0]; % Initial robot pose
freq = 2*pi/30;
xRef = 0 + 0.7*sin(freq*t);
yRef = 0 + 0.7*sin(2*freq*t);
dxRef = freq*0.7*cos(freq*t);
dyRef = 2*freq*0.7*cos(2*freq*t);
ddxRef =-freq^2*0.7*sin(freq*t); 
ddyRef =-4*freq^2*0.7*sin(2*freq*t);

qRef = [xRef; yRef; atan2(dyRef, dxRef)]; % Reference trajectory
vRef = sqrt(dxRef.^2+dyRef.^2);
wRef = (dxRef.*ddyRef-dyRef.*ddxRef)./(dxRef.^2+dyRef.^2);
uRef = [vRef; wRef]; % Reference inputs

m = 1;
for k = 1:length(t)-4
    e = [cos(q(3)), sin(q(3)), 0; ...
         -sin(q(3)), cos(q(3)), 0; ...
         0, 0,1]*(qRef(:,k) - q); % Error vector

    e(3) = wrapToPi(e(3)); % Correct angle
    
    A0 = [1, Ts*uRef(2,k), 0;-Ts*uRef(2,k),1, Ts*uRef(1,k);0,0,1];
    A1 = [1, Ts*uRef(2,k+1), 0;-Ts*uRef(2,k+1), 1, Ts*uRef(1,k+1); 0,0,1];
    A2 = [1, Ts*uRef(2,k+2), 0;-Ts*uRef(2,k+2), 1, Ts*uRef(1,k+2); 0,0,1];
    A3 = [1, Ts*uRef(2,k+3), 0;-Ts*uRef(2,k+3), 1, Ts*uRef(1,k+3); 0,0,1];
    A4 = [1, Ts*uRef(2,k+4), 0;-Ts*uRef(2,k+4), 1, Ts*uRef(1,k+4); 0,0,1];
    B = [Ts, 0; 0, 0; 0, Ts];
    C = eye(3);

    Z = zeros(3,2);
    Hm = [C*B, Z, Z, Z, Z; ...
          C*A1*B, C*B, Z, Z, Z; ...
          C*A1*A2*B, C*A2*B, C*B, Z, Z; ...
          C*A1*A2*A3*B, C*A2*A3*B, C*A3*B, C*B, Z; ...
          C*A1*A2*A3*A4*B, C*A2*A3*A4*B, C*A3*A4*B, C*A4*B, C*B];

    Fm = [C*A0, C*A0*A1, C*A0*A1*A2, C*A0*A1*A2*A3, C*A0*A1*A2*A3*A4].';
    ar = 0.65;
    Ar = eye(3)*ar; % Reference error dynamics
    H = 0;
    Fr = [Ar^(H+1), Ar^(H+2), Ar^(H+3), Ar^(H+4)].';
    % Weight matrices
    Qt = diag(repmat([1; 40; 0.1], 5, 1));
    Rt = diag(repmat([0.001; 0.001], 5, 1));

    % Optimal control calculation
    KKgpc = (Hm.'*Qt*Hm + Rt)\(Hm.'*Qt*(-Fm));
    KK = KKgpc(1:2,:); % Take current control gains

    v = -KK*e;
    uF = [uRef(1,k)*cos(e(3)); uRef(2,k)];
    u = v + uF;
    vMAX = 1; wMAX = 15; % Max velocities
    if abs(u(1))>vMAX, u(1) = sign(u(1))*vMAX; end
    if abs(u(2))>wMAX, u(2) = sign(u(2))*wMAX; end

    % Robot motion simulation
    dq = [u(1)*cos(q(3)); u(1)*sin(q(3)); u(2)];
    noise = 0.00; % Set to experiment with noise (e.g. 0.001)
    q = q + Ts*dq + randn(3,1)*noise; % Euler integration
    q(3) = wrapToPi(q(3)); % Map orientation angle to [-pi, pi]
    x(m) = q(1);
    y(m) = q(2);
    m = m+1;
end

figure(1)
plot(xRef, yRef,'Color','r','LineStyle','--')
hold on
plot(x, y,'b') 
xlabel('x(m)') 
ylabel('y(m)')
legend('Reference trajectory','MPC tracking results')
set(gca,"FontSize",10)
