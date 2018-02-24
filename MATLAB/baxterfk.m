% Forward Kinematics of Baxter robot

%% Defining symbolic variables
l1 = sym('l1'); d1 = sym('d1');
l2 = sym('l2'); d2 = sym('d2');
l3 = sym('l3'); d3 = sym('d3');
l4 = sym('l4'); d4 = sym('d4');
theta0 = sym('q0'); theta1 = sym('q1');
theta2 = sym('q2'); theta3 = sym('q3');
theta4 = sym('q4'); theta5 = sym('q5'); 
theta6 = sym('q6'); pi = sym('pi');
PI = 3.1415926;

%% DH variables in MATLAB
DH_d = [l1; 0; l2; 0; l3; 0; l4];
DH_theta = [theta0; theta1+(pi/2); theta2; theta3; theta4; theta5; theta6];
DH_a = [d1; 0; d2; 0; d3; 0; d4];
DH_alpha = [-90; 90; -90; 90; -90; 90; 0];

%DH_d, DH_theta, DH_a, DH_alpha

DH_alpha = DH_alpha*(pi/180);

%% Converting DH parameters to T matrix
T = eye(4);
Rz_seq = [];

for i = 1:7
    T_new = dhparam2matrix(DH_theta(i), DH_d(i), DH_a(i), DH_alpha(i));
    T = T*T_new; T_new
    Rz_seq = [Rz_seq, T(1:3, 3)];
end

%% Final transformation matrix
T = simplify(T); T

%% Standard lengths and angles (Do not change)
ang_vec = [theta0, theta1, theta2, theta3, theta4, theta5, theta6];
len_vec = [l1, d1, l2, d2, l3, d3, l4, d4, pi];
len_val = [0.27035, 0.069, 0.36435, 0.069, 0.37429, 0.01, 0.229525, 0, PI];

%% Forward Kinematics Equations

fk_eqns = [T(1, 4); T(2, 4); T(3, 4)]; fk_eqns

%% Jacobian Equations

A = [];
for i = 1:7
    A = [A, diff(fk_eqns, ang_vec(i))];
end

B = Rz_seq;

A, B

J = [A; B]; 
J, size(J)

%% Checking for home configuration
home_vec = [0, 0, 0, 0, 0, 0, 0];
home_vec = home_vec*(PI/180);

T_home = subs(T, len_vec, len_val);
T_home = double(subs(T_home, ang_vec, home_vec)); T_home

%% Solving for random configuration
conf_vec = [-45, 30, -30, 30, -45, 0, 100];
conf_vec = conf_vec*(PI/180);

T_conf = subs(T, len_vec, len_val);
T_conf = double(subs(T_conf, ang_vec, conf_vec)); T_conf
