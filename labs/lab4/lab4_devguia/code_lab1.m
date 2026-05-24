%% ============================================================
%  Generación de gráficas del Control de Ratio de Caudal
%  Laboratorio 4 - Control de Procesos - UTEC
%  Plataforma: Siemens (G120 + EV11)
%  Datos: Trace exportado desde TIA Portal (CSV)
% ============================================================
%  Salida: 4 figuras PNG en la carpeta indicada,
%          listas para ser referenciadas desde el documento LaTeX.
% ============================================================

clear; clc; close all;

%% --------------------- CONFIGURACIÓN ------------------------
% Ruta al CSV exportado de TIA Portal
csv_file = "C:\A_CURSOS20261\CONTROL\Guia_lab4\datas_cp_lab4\Trace2_Parte1.csv";

% Carpeta donde se guardarán las imágenes (debe coincidir con la ruta
% que usa el documento LaTeX en \includegraphics{images/...})
out_dir = "C:\A_CURSOS20261\CONTROL\Guia_lab4\images";

% Periodo de muestreo del Trace (segundos). Verifica en TIA Portal:
%   Trace -> Configuración -> Tiempo de ciclo de registro.
% Valores típicos: 0.02 (20 ms), 0.05 (50 ms), 0.1 (100 ms).
Ts = 0.1;     % 100 ms por defecto

% Factor de escala Int -> unidad física.
% Si guardaste las variables en TIA como "valor * 1000" para enviarlas
% como DInt, divides por 1000. Ajusta si tu escala es distinta.
scale = 1000;

% Crear carpeta de salida si no existe
if ~exist(out_dir, 'dir'); mkdir(out_dir); end

%% --------------------- LECTURA DE DATOS ----------------------
T = readtable(csv_file);

% Renombramos columnas relevantes (los nombres reales vienen del Trace)
vars = T.Properties.VariableNames;
idx_r_des  = find(contains(vars, 'R_deseado'),  1);
idx_r_real = find(contains(vars, 'R_Real'),     1);
idx_g120   = find(contains(vars, 'G120'),       1);
idx_sp     = find(contains(vars, 'ST_FC11'),    1);
idx_ft10   = find(contains(vars, 'FT10'),       1);
idx_ft11   = find(contains(vars, 'FT11'),       1);

R_des  = T{:, idx_r_des}  / scale;
R_real = T{:, idx_r_real} / scale;
G120   = T{:, idx_g120}   / scale;
SP     = T{:, idx_sp}     / scale;
FT10   = T{:, idx_ft10}   / scale;
FT11   = T{:, idx_ft11}   / scale;

% Vector de tiempo
N = height(T);
t = (0:N-1)' * Ts;

%% ---------------- LIMPIEZA DE VALORES ESPURIOS ----------------
% El Trace mete -8.3886e+06 o 2.1433e+09 cuando la división es inválida
% (división por cero en el bloque DIV del ratio real). Los reemplazamos
% por NaN para que MATLAB no los grafique.
R_real(abs(R_real) > 5) = NaN;        % Ratio físicamente acotado a < 5

% El esfuerzo G120 tiene picos de muestreo (artefactos del scan rápido del
% Trace). Aplicamos un suavizado por media móvil para visualización.
G120_smooth = movmean(G120, 20);      % ventana 20 muestras (~2 s con Ts=0.1)

%% ---------- ESTILO COMÚN DE LAS FIGURAS ----------
set(0, 'DefaultAxesFontSize', 12);
set(0, 'DefaultLineLineWidth', 1.5);

%% ============================================================
%  FIGURA 1: Control FC11 - Setpoint vs caudal medido FT11
% ============================================================
fig1 = figure('Name','FC11 SP vs PV','Position',[100 100 900 450]);
plot(t, SP,   'r-',  'DisplayName','Setpoint FC11'); hold on;
plot(t, FT11, 'b-',  'DisplayName','Caudal medido FT11');
grid on; box on;
xlabel('Tiempo [s]');
ylabel('Caudal [l/min]');
title('Control de caudal FC11: Setpoint vs caudal medido FT11', ...
      'Interpreter','none');
legend('Location','best');
xlim([0 t(end)]);
saveas(fig1, fullfile(out_dir, 'proc_siemens_fc11_sp_vs_pv.png'));

%% ============================================================
%  FIGURA 2: Control FC11 - Esfuerzo de control (% G120)
% ============================================================
fig2 = figure('Name','FC11 Esfuerzo','Position',[100 100 900 450]);
plot(t, G120,        'Color',[1 0.7 0],'DisplayName','Esfuerzo G120 '); hold on;
plot(t, G120_smooth, 'r-','LineWidth',2,'DisplayName','Esfuerzo G120 ');
grid on; box on;
xlabel('Tiempo [s]');
ylabel('Velocidad G120 [\%]');
title('Esfuerzo de control de FC11: porcentaje de velocidad del G120');
legend('Location','best');
xlim([0 t(end)]);
ylim([0 max(G120_smooth)*1.2]);
saveas(fig2, fullfile(out_dir, 'proc_siemens_fc11_esfuerzo.png'));

%% ============================================================
%  FIGURA 3: Ratio deseado vs ratio medido
% ============================================================
fig3 = figure('Name','Ratios','Position',[100 100 900 450]);
plot(t, R_des,  'r-', 'DisplayName','Ratio deseado'); hold on;
plot(t, R_real, 'b-', 'DisplayName','Ratio real (FT11/FT10)');
grid on; box on;
xlabel('Tiempo [s]');
ylabel('Ratio FT11/FT10 [-]');
title('Ratio deseado vs ratio real');
legend('Location','best');
xlim([0 t(end)]);
ylim([-0.1 2]);
saveas(fig3, fullfile(out_dir, 'proc_siemens_ratio_des_vs_med.png'));

%% ============================================================
%  FIGURA 4: Comparación de caudales FT10 vs FT11
% ============================================================
fig4 = figure('Name','FT10 vs FT11','Position',[100 100 900 450]);
plot(t, FT10, 'b-', 'DisplayName','Caudal FT10 (no controlado)'); hold on;
plot(t, FT11, 'r-', 'DisplayName','Caudal FT11 (controlado)');
grid on; box on;
xlabel('Tiempo [s]');
ylabel('Caudal [l/min]');
title('Comparación de caudales FT10 vs FT11');
legend('Location','best');
xlim([0 t(end)]);
saveas(fig4, fullfile(out_dir, 'proc_siemens_ft10_vs_ft11.png'));

fprintf('\n[OK] 4 graficas guardadas en:\n   %s\n', out_dir);
fprintf('   - proc_siemens_fc11_sp_vs_pv.png\n');
fprintf('   - proc_siemens_fc11_esfuerzo.png\n');
fprintf('   - proc_siemens_ratio_des_vs_med.png\n');
fprintf('   - proc_siemens_ft10_vs_ft11.png\n\n');