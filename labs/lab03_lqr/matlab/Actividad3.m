%% ============================================================
%  LAB 03 - CONTROL LQR DEL PÉNDULO ROTATORIO INVERTIDO
%  Generación de figuras para el informe
%
%  Variables del workspace esperadas:
%    Voltage          [t, V_m]               -> Señal de control
%    alpha_data       [t, alpha, alpha_dot]  -> Estado del péndulo
%    rotacion_angular [t, theta]             -> Estado del brazo
%    energia          [t, E]                 -> Energía total
% ============================================================

clc

%% --- Configuración global --------------------------------------
anchoLinea = 1.5;
tamFuente  = 12;
resolucion = 300;

carpeta_fig = '/home/yholi/control_procesos/labs/lab03_lqr/figures';
if ~exist(carpeta_fig, 'dir')
    mkdir(carpeta_fig)
end

set(groot, 'defaultAxesFontName',   'Helvetica')
set(groot, 'defaultAxesFontSize',   tamFuente)
set(groot, 'defaultTextInterpreter','latex')
set(groot, 'defaultLegendInterpreter','latex')
set(groot, 'defaultAxesTickLabelInterpreter','latex')


%% --- Figura 1: Ángulo del brazo rotatorio theta ----------------
figure('Name','Brazo rotatorio','Color','w')

plot(rotacion_angular(:,1), rotacion_angular(:,2), ...
     'Color',[0 0.45 0.74], 'LineWidth', anchoLinea)
xlabel('Tiempo $t$ [s]')
ylabel('$\theta$ [rad]')
title('\''Angulo del brazo rotatorio')
grid on
legend('$\theta(t)$', 'Location', 'best')

exportgraphics(gcf, fullfile(carpeta_fig,'theta.png'), ...
    'Resolution', resolucion)
fprintf('✓ theta.png guardada\n')


%% --- Figura 2: Posición angular del péndulo alpha --------------
figure('Name','Posición del péndulo','Color','w')

plot(alpha_data(:,1), alpha_data(:,2), 'r', 'LineWidth', anchoLinea)
xlabel('Tiempo $t$ [s]')
ylabel('$\alpha$ [rad]')
title('Posici\''on angular del p\''endulo')
grid on
legend('$\alpha(t)$', 'Location', 'best')

exportgraphics(gcf, fullfile(carpeta_fig,'alpha.png'), ...
    'Resolution', resolucion)
fprintf('✓ alpha.png guardada\n')


%% --- Figura 3: Velocidad angular del péndulo alpha_dot ---------
figure('Name','Velocidad del péndulo','Color','w')

plot(alpha_data(:,1), alpha_data(:,3), 'm', 'LineWidth', anchoLinea)
xlabel('Tiempo $t$ [s]')
ylabel('$\dot{\alpha}$ [rad/s]')
title('Velocidad angular del p\''endulo')
grid on
legend('$\dot{\alpha}(t)$', 'Location', 'best')

exportgraphics(gcf, fullfile(carpeta_fig,'alpha_dot.png'), ...
    'Resolution', resolucion)
fprintf('✓ alpha_dot.png guardada\n')


%% --- Figura 4: Señal de control V_m ----------------------------
figure('Name','Señal de control','Color','w')

plot(Voltage(:,1), Voltage(:,2), 'b', 'LineWidth', anchoLinea)
xlabel('Tiempo $t$ [s]')
ylabel('$V_m$ [V]')
title('Se\~nal de control aplicada al motor')
grid on
legend('$V_m(t)$', 'Location', 'best')

exportgraphics(gcf, fullfile(carpeta_fig,'voltaje.png'), ...
    'Resolution', resolucion)
fprintf('✓ voltaje.png guardada\n')


%% --- Figura 5: Energía total del sistema -----------------------
figure('Name','Energía total','Color','w')

plot(energia(:,1), energia(:,2), 'Color',[0 0.5 0], ...
     'LineWidth', anchoLinea)
xlabel('Tiempo $t$ [s]')
ylabel('Energ\''ia $E$ [J]')
title('Energ\''ia total del sistema (swing-up)')
grid on
legend('$E(t)$', 'Location', 'best')

exportgraphics(gcf, fullfile(carpeta_fig,'energia.png'), ...
    'Resolution', resolucion)
fprintf('✓ energia.png guardada\n')


%% --- Figura 6: Respuesta global (todas en una figura) ----------
figure('Name','Respuesta global del sistema', ...
       'Color','w', ...
       'Position',[100 100 1000 800])

% (1) Ángulo del brazo rotatorio theta
subplot(3,2,1)
plot(rotacion_angular(:,1), rotacion_angular(:,2), ...
     'Color',[0 0.45 0.74], 'LineWidth', anchoLinea)
xlabel('Tiempo $t$ [s]')
ylabel('$\theta$ [rad]')
title('\''Angulo del brazo rotatorio')
grid on
legend('$\theta(t)$', 'Location', 'best')

% (2) Posición angular del péndulo alpha
subplot(3,2,2)
plot(alpha_data(:,1), alpha_data(:,2), 'r', 'LineWidth', anchoLinea)
xlabel('Tiempo $t$ [s]')
ylabel('$\alpha$ [rad]')
title('Posici\''on angular del p\''endulo')
grid on
legend('$\alpha(t)$', 'Location', 'best')

% (3) Velocidad angular del péndulo alpha_dot
subplot(3,2,3)
plot(alpha_data(:,1), alpha_data(:,3), 'm', 'LineWidth', anchoLinea)
xlabel('Tiempo $t$ [s]')
ylabel('$\dot{\alpha}$ [rad/s]')
title('Velocidad angular del p\''endulo')
grid on
legend('$\dot{\alpha}(t)$', 'Location', 'best')

% (4) Señal de control V_m
subplot(3,2,4)
plot(Voltage(:,1), Voltage(:,2), 'b', 'LineWidth', anchoLinea)
xlabel('Tiempo $t$ [s]')
ylabel('$V_m$ [V]')
title('Se\~nal de control aplicada al motor')
grid on
legend('$V_m(t)$', 'Location', 'best')

% (5) Energía total (ocupa las dos columnas inferiores)
subplot(3,2,[5 6])
plot(energia(:,1), energia(:,2), 'Color',[0 0.5 0], ...
     'LineWidth', anchoLinea)
xlabel('Tiempo $t$ [s]')
ylabel('Energ\''ia $E$ [J]')
title('Energ\''ia total del sistema (swing-up)')
grid on
legend('$E(t)$', 'Location', 'best')

sgtitle('Respuesta global del sistema bajo control LQR', ...
    'Interpreter','latex','FontSize',14)

exportgraphics(gcf, fullfile(carpeta_fig,'respuesta_global.png'), ...
    'Resolution', resolucion)
fprintf('✓ respuesta_global.png guardada\n')


fprintf('\nTodas las figuras guardadas en:\n  %s\n', carpeta_fig)