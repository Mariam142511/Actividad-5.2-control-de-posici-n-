%% 0) Limpieza y parámetros =================================================
clear all; close all; clc;

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
numWP = size(waypoints,1);

% Parámetros de simulación
ts = 0.1;                     % paso de tiempo [s]
maxSteps = 20000;             % tope de iteraciones

% Ganancias y umbrales
K_rho    = 1.0;               % ganancia de distancia
K_alpha  = 2.0;               % ganancia angular
rho_th   = 0.05;              % umbral de proximidad [m]
alpha_th = 0.01;              % umbral angular [rad]

% Prealocar (se recortará al final)
x    = zeros(maxSteps,1);
y    = zeros(maxSteps,1);
phi  = zeros(maxSteps,1);
v    = zeros(maxSteps-1,1);
w    = zeros(maxSteps-1,1);
Error= zeros(maxSteps-1,1);

% Inicio en primer waypoint
x(1)   = waypoints(1,1);
y(1)   = waypoints(1,2);
phi(1) = pi/2;                % orientación inicial

step = 1;                     % índice para grabar datos

%% 1) Bucle sobre cada destino =====================================
for i = 2:numWP
  xd = waypoints(i,1);
  yd = waypoints(i,2);
  rho = hypot(xd-x(step), yd-y(step));   % distancia inicial
  
  while rho > rho_th && step < maxSteps
    dx = xd - x(step);
    dy = yd - y(step);
    rho = hypot(dx,dy);
    theta_d = atan2(dy, dx);
    alpha = atan2(sin(theta_d - phi(step)), cos(theta_d - phi(step)));
    
    if abs(alpha) > alpha_th          % “gira”
      v(step) = 0;
      w(step) = K_alpha*alpha;
    else                              % “avanza”
      v(step) = K_rho*rho;
      w(step) = 0;
    end
    
    % Integración (Euler)
    phi(step+1) = phi(step) + w(step)*ts;
    x(step+1)   = x(step)   + v(step)*cos(phi(step))*ts;
    y(step+1)   = y(step)   + v(step)*sin(phi(step))*ts;
    Error(step) = rho;
    
    step = step + 1;
  end
end

% Recortar vectores
x    = x(1:step);
y    = y(1:step);
phi  = phi(1:step);
v    = v(1:step-1);
w    = w(1:step-1);
Error= Error(1:step-1);

%% 2) Gráfica 2‑D de la trayectoria ================================
figure(1); clf; hold on; grid on; axis equal
plot(x, y, 'r-', 'LineWidth', 2);
plot(waypoints(:,1), waypoints(:,2), 'bo', 'MarkerFaceColor','w');
xlabel('x [m]'); ylabel('y [m]');
title('Trayectoria Lazo Cerrado');
legend('Camino seguido','Waypoints','Location','Best');

%% 3) Animación 3‑D estilo MobileRobot_5 ===========================
scene = figure(2); clf
set(scene,'Color','white','Position',get(0,'ScreenSize'));
set(gca,'FontWeight','bold');
axis equal; grid on; box on;
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
view([-0.1 35]);

% Límites automáticos con margen
margin = 2;
axis([min(x)-margin max(x)+margin  ...
      min(y)-margin max(y)+margin  ...
      0 1]);

hold on
plot3(x, y, zeros(size(x)), 'r-', 'LineWidth',1.5); 

scale = 4;           
MobileRobot_5;          

% Dibujo inicial del robot
H1 = MobilePlot_4(x(1), y(1), phi(1), scale); hold on
H2 = plot3(x(1), y(1), 0, 'g.', 'MarkerSize', 20);   % marca inicio

% Bucle de animación
N = length(x);
stepAnim = 1;              % 10 fps aprox. (ts = 0.1 s)
for k = 1:stepAnim:N
    delete(H1);           
    H1 = MobilePlot_4(x(k), y(k), phi(k), scale);  
    pause(ts);             % sincronizado al paso de muestreo
end

%% 4) Subplots de v, w y error ====================================
figure(3); clf; set(gcf,'Position',[50 50 600 700])
t = (0:length(v)-1)*ts;

subplot(3,1,1)
plot(t, v,'b','LineWidth',1.5); grid on
ylabel('v [m/s]'); title('Velocidad Lineal')

subplot(3,1,2)
plot(t, w,'g','LineWidth',1.5); grid on
ylabel('\omega [rad/s]'); title('Velocidad Angular')

subplot(3,1,3)
plot(t, Error,'r','LineWidth',1.5); grid on
xlabel('Tiempo [s]'); ylabel('Error [m]'); title('Error de Posición')
