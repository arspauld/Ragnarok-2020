g = 9.805; % Gravity (m/s^2)
rho = 0.9075 * 1.225; % Density (kg/m^3) @ 1000 m ASL

%% Container
m = .6; % Mass (kg)
CDp = 1.0; % Coefficient of Drag of Parachute
A_p = linspace(0,.1); % Area of Parachute (m^2) (0.02 yields 18.7632)
CDc = 0.82; % Coefficient of Drag of Container
A_c = pi * (.125)^2 / 4; % Area of Bottom of Container

V_c = sqrt( (2 *m*g / rho) ./ ((CDp * A_p) + (CDc * A_c))); % Calculates Terminal Velocity (m/s)
figure(1)
plot(A_p,V_c)


m = .15; % Mass (kg)
CDp = 1.0; % Coefficient of Drag of Parachute
A_p = linspace(0,.1); % Area of Parachute (m^2) (0.02 yields 9.3816)
CDc = 0.82; % Coefficient of Drag of Container
A_c = pi * (.125)^2 / 4; % Area of Bottom of Container

V_c = sqrt( (2 *m*g / rho) ./ ((CDp * A_p) + (CDc * A_c))); % Calculates Terminal Velocity (m/s)
figure(2)
plot(A_p,V_c)


%% Glider
% Treats the payload design as two airfoils, one swept and the other 
r = 250; % radius of spiral

% Input Variables
m = .45; % Mass (kg)
CD0 = 0.03; % Estimated zero lift drag coefficient
c_r = .250; % root chord (m)
c_t = .040; % tip chord (m)
b_w = .433; % triangular wing span (m)
b_f = .100; % fuselage span (m)


% Flight Variables
S = c_r * b_f + (c_r + c_t) * b_w; % Area of Wing (m^2)
AR = (b_f+2*b_w)^2 / S; % Aspect Ratio of Wing
phi = atand(3/4*c_r/(c_r*b_w/(c_r-c_t))); % sweep angle of wings
V = linspace(eps,15,1000); % Velocity (m/s)
Q = rho * V.^2 / 2; % Dynamic Pressure (Pa)
CL = m * sqrt(g^2 + (V.^2/r).^2) ./ (Q * S); % Coefficient of Lift Required accounts for Spiral


% Adjusts the Velocity and CL to be reasonable
is_valid = CL < 1; 
CL = CL(is_valid);
V = V(is_valid);


% Finding the span wise efficieny
Cla = 2*pi * AR / (sqrt((AR/(cosd(phi)))^2 + 4) + 2) / (180 / pi); % 2D lift slope of Wing (per degree)
CLa = Cla * AR / (2 + sqrt(4 + AR ^ 2)); % 3D lift slope of Wing (per degree)

a2D = (CL / Cla); % 2D angle of attack
a3D = (CL / CLa); % 3D angle of attack

ai_rad = (a3D - a2D) / (180 / pi); % Angle induced between the 2D and 3D models (rad)
e = CL / (pi * AR * ai_rad); % Span wise efficiency


% Calculates CD
CD = CD0 + CL.^2/ (pi * AR * e); % Coefficient of Drag
L_D = CL ./ CD; % L/D ratio
V_z = - V ./ sqrt(L_D.^2 + 1); % Sink Rate (m/s)
V_xy = sqrt(V.^2 - V_z.^2);


% Plotting
figure(3)
subplot(2,2,1)
plot(V,CL)
xlabel 'Airspeed (m/s)'
ylabel 'C_L'
grid on
subplot(2,2,2)
plot(V,CD)
xlabel 'Airspeed (m/s)'
ylabel 'C_D'
grid on
subplot(2,2,3)
plot(CL,CD)
xlabel 'C_L'
ylabel 'C_D'
grid on
subplot(2,2,4)
plot(V,L_D)
xlabel 'Velocity (m/s)'
ylabel '^L/_D'
grid on

figure(4)
plot(L_D,V_z)
xlabel '^L/_D'
ylabel 'Sink Rate (m/s)'
grid on

figure(5)
plot(V,V_xy)
xlabel 'Airspeed (m/s)'
ylabel 'Horizontal Velocity (m/s)'
grid on

%% Final Descent
m = .45; % Mass (kg)
CDp = 1.0; % Coefficient of Drag of Parachute
A_p = linspace(0,1); % Area of Parachute (m^2) (0.1 yields 8.9095)
V = sqrt((2 *m*g / rho) ./ (CDp * A_p)); % Calculates Terminal Velocity (m/s)
figure(6)
plot(A_p,V)