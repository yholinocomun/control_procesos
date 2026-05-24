%% ============================================================
%  Gráfica estilo Trace TIA Portal - Control de Ratio de Caudal
%  Laboratorio 4 - Control de Procesos - UTEC
%  Plataforma: Siemens (G120 + EV11)
%
%  Replica el estilo del Trace de TIA Portal:
%   - Todas las señales en valores ENTEROS CRUDOS (sin dividir)
%   - Eje X en MINUTOS (no segundos)
%   - Todo superpuesto en un solo eje
%   - Colores típicos del Trace: rojo, verde, azul, cyan
% ============================================================

clear; clc; close all;

%% --------------------- CONFIGURACIÓN ------------------------
csv_file = "C:\A_CURSOS20261\CONTROL\Guia_lab4\datas_cp_lab4\Trace2_Parte1.csv";
out_dir  = "C:\A_CURSOS20261\CONTROL\Guia_lab4\images";
Ts       = 0.1;       % período de muestreo (s) - ver TIA Portal

if ~exist(out_dir, 'dir'); mkdir(out_dir); end

%% --------------------- LECTURA ----------------------
T = readtable(csv_file);
vars = T.Properties.VariableNames;
idx_r_des  = find(contains(vars, 'R_deseado'), 1);
idx_r_real = find(contains(vars, 'R_Real'),    1);
idx_g120   = find(contains(vars, 'G120'),      1);
idx_sp     = find(contains(vars, 'ST_FC11'),   1);
idx_ft10   = find(contains(vars, 'FT10'),      1);
idx_ft11   = find(contains(vars, 'FT11'),      1);

% Valores CRUDOS, sin dividir por 1000 (como TIA Portal los muestra)
R_des  = T{:, idx_r_des};
R_real = T{:, idx_r_real};
G120   = T{:, idx_g120};
SP     = T{:, idx_sp};
FT10   = T{:, idx_ft10};
FT11   = T{:, idx_ft11};

N = height(T);
t_min = (0:N-1)' * Ts / 60;   % tiempo en MINUTOS

%% ---------------- LIMPIEZA DE VALORES ESPURIOS ----------------
% Los picos -8388600 y 2143300000 del ratio real (división por cero)
R_real(abs(R_real) > 50000) = NaN;

%% ---------- ESTILO ----------
set(0, 'DefaultAxesFontSize', 11);

%% ============================================================
%  GRÁFICA estilo Trace TIA Portal
% ============================================================
fig = figure('Name','Trace Control Ratio', ...
             'Position',[50 50 1500 600], ...
             'Color','w');

hold on;

% Colores típicos del Trace TIA Portal
plot(t_min, FT10,   'Color',[1.00 0.20 0.40], 'LineWidth',1.0, ...
     'DisplayName','FT10\_Int (MD22)');                        % rojo/rosa
plot(t_min, FT11,   'Color',[0.10 0.30 0.85], 'LineWidth',1.0, ...
     'DisplayName','FT11\_Int (MD26)');                        % azul
plot(t_min, SP,     'Color',[0.20 0.65 1.00], 'LineWidth',1.0, ...
     'DisplayName','Setpoint FC11 (MD78)');                    % cyan
plot(t_min, G120,   'Color',[0.15 0.75 0.20], 'LineWidth',0.8, ...
     'DisplayName','G120 (MD58)');                             % verde
plot(t_min, R_des,  'Color',[0.85 0.55 0.10], 'LineWidth',1.2, ...
     'DisplayName','Ratio deseado (MD70)');                    % naranja
plot(t_min, R_real, 'Color',[0.50 0.50 0.50], 'LineWidth',1.0, ...
     'DisplayName','Ratio real (MD74)');                       % gris

grid on; box on;
set(gca, 'GridLineStyle','-', 'GridColor',[0.85 0.85 0.85], 'GridAlpha',1);
set(gca, 'MinorGridLineStyle',':', 'XMinorGrid','on', 'YMinorGrid','on');

xlabel('Tiempo [min]');
ylabel('Valor entero (DInt)');
title('Trace- Control de Ratio de Caudal (Siemens)');

legend('Location','northeastoutside','FontSize',9);
xlim([0 t_min(end)]);

% El rango de Y se ajusta automáticamente, pero limitamos por si hay outliers
ylim([min([FT10;FT11;SP;G120;R_des;R_real],[],'omitnan')-2000, ...
      max([FT10;FT11;SP;G120;R_des;R_real],[],'omitnan')+5000]);

exportgraphics(fig, fullfile(out_dir, 'proc_siemens_ratio_trace.png'), ...
               'Resolution', 150);

fprintf('\n[OK] Grafica estilo Trace guardada en:\n   %s\\proc_siemens_ratio_trace.png\n\n', out_dir);