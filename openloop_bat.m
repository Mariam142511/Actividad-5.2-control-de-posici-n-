%% Limpieza de pantalla
clear; close all; clc

%% Parámetros ------------------------------------------------------
ts    = 0.1;        % paso de muestreo [s]
v_lin = 1.0;        % velocidad lineal [m/s]
v_ang = pi/4;       % velocidad angular [rad/s]

waypoints = [
   -26.08, -12.53;
   -24.10, -10.48;
   -24.17, -11.69;
   -23.11, -11.76;
   -21.42, -12.39;
   -17.46, -14.73;
   -16.26, -14.51;
   -13.64, -12.68;
   -12.65, -11.54;
   -11.52, -11.76;
   -10.60, -11.62;
    -9.47, -12.61;
    -7.49, -14.44;
    -5.65, -16.49;
    -3.82, -14.80;
    -0.07, -13.59;
     1.20, -13.59;
     5.16, -15.72;
     6.72, -17.27;
     7.92, -13.67;
     8.84, -11.47;
    11.10,  -9.85;
    12.87,  -7.66;
    14.85,  -6.74;
    14.92,  -2.78;
    15.84,   1.04;
    16.75,   3.09;
    18.66,   4.86;
    21.56,   5.84;
    22.48,   9.87;
    24.60,  13.48;
    25.52,  15.81;
    23.54,  14.61;
    21.56,  14.61;
    18.52,  14.54;
    14.70,  14.33;
    12.87,  15.60;
    12.80,  11.92;
    11.95,   7.82;
     9.90,   4.08;
     7.92,   0.26;
     6.01,  -1.93;
     4.95,  -0.94;
     3.18,   2.81;
     3.04,   1.04;
     2.05,   0.05;
     0.85,   0.12;
     0.07,  -1.01;
    -1.77,   1.11;
    -0.63,  -4.69;
    -0.85,  -5.75;
    -4.03,  -5.75;
    -6.64,  -5.61;
    -9.33,  -4.90;
   -11.52,  -3.77;
   -14.49,  -1.86;
   -18.38,   0.12;
   -20.64,  -4.55;
   -22.19,  -7.80
];
nWP = size(waypoints,1);

u = [];
w = [];

phi_tmp = atan2(waypoints(2,2)-waypoints(1,2), ...
                waypoints(2,1)-waypoints(1,1));

for i = 1:nWP-1
  dx = waypoints(i+1,1) - waypoints(i,1);
  dy = waypoints(i+1,2) - waypoints(i,2);
  
  th   = atan2(dy,dx);
  dth  = wrapToPi(th - phi_tmp);
  Nturn = ceil(abs(dth)/(v_ang*ts));
  for k=1:Nturn
    u(end+1) = 0;
    w(end+1) = sign(dth)*v_ang;
    phi_tmp  = phi_tmp + w(end)*ts;
  end
  
  dist  = hypot(dx,dy);
  Nmov  = ceil(dist/(v_lin*ts));
  for k=1:Nmov
    u(end+1) = v_lin;
    w(end+1) = 0;
    phi_tmp  = phi_tmp + w(end)*ts;
  end
end

%% Simulación cinemática ------------------------------------------
N = length(u);
x = zeros(1,N+1);
y = zeros(1,N+1);
phi = zeros(1,N+1);
x(1)=waypoints(1,1);
y(1)=waypoints(1,2);
phi(1)=atan2(waypoints(2,2)-waypoints(1,2), ...
             waypoints(2,1)-waypoints(1,1));

for k=1:N
  phi(k+1) = phi(k) + w(k)*ts;
  x(k+1)   = x(k)   + u(k)*cos(phi(k))*ts;
  y(k+1)   = y(k)   + u(k)*sin(phi(k))*ts;
end

%% Escena 3‑D y trayectoria ---------------------------------------
scene = figure;
set(scene,'Color','white','Position',get(0,'ScreenSize'));
set(gca,'FontWeight','bold');
axis equal; grid on; box on;
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
view([-0.1 35]);

% Límite automático con un pequeño margen
margin = 2;
axis([min(x)-margin max(x)+margin  ...
      min(y)-margin max(y)+margin  ...
      0 1]);

hold on
plot3(x, y, zeros(size(x)), 'b.-', 'LineWidth',1);

%% Robot y animación ----------------------------------------------
scale = 4;        % escala
MobileRobot_5;    % “carga” el modelo en variables globales

% Posición inicial del robot sobre la escena
H1 = MobilePlot_4(x(1), y(1), phi(1), scale); hold on

% Para ir pintando la huella que va dejando
H2 = plot3(x(1), y(1), 0, 'r', 'LineWidth', 2);

step = 1;    
for k = 1:step:N
    delete(H1);        % borro el dibujo anterior
    delete(H2);
    
    H1 = MobilePlot_4(x(k), y(k), phi(k), scale); % nuevo
    H2 = plot3(x(1:k), y(1:k), zeros(1,k), 'r', 'LineWidth',2);
    
    pause(ts);         % respeta el periodo de muestreo
end
