%% 0) Limpieza y parámetros =================================================
clear all; close all; clc;

waypoints = [
   -0.66, 3.7;
    0, 3;
    0.78, 2.32;
    2.28, 3.06;
    3.78, 3.06;
    4.54, 1.56;
    3.78, 0.06;
    3.06, -0.72;
    4.5, -1.44;
    4.54, -2.26;
    3, -3;
    2.3, -2.2;
    0.78, -1.42;
    0.02, -2.22;
   -0.72, -1.46;
   -2.26, -2.2;
   -3.74, -2.98;
   -4.5, -2.2;
   -4.5, -1.5;
   -3.08, -0.76;
   -3.82, 0.14;
   -4.44, 1.52;
   -3.76, 3.06;
   -2.28, 3.06;
   -0.8, 2.34;
    0.84, 3.84;
    0, 3;
   -0.8, 2.34;
   -0.72, -1.48;
    0.02, -2.22;
    0.78, -1.44;
    0.78, 2.34;
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
