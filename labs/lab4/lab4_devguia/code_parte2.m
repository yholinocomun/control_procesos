%% ============================================================
%  Gráfica estilo Trace TIA Portal - Control en Cascada
%  Laboratorio 4 - Control de Procesos - UTEC
%  Plataforma: Siemens (G120 + EV11)
%
%  Variables del Trace de la Parte 2 (cascada):
%   Columna del CSV               -> Variable de proceso
%   ----------------------------------------------------------
%   FT10_Int__MD22_               -> FT10 (caudal medido)
%   Int_FC10__MD102_              -> Setpoint FC10 (salida LC10)
%   Int_EV11_CO__MD106_           -> Esfuerzo FC10 -> EV11
%   PT10_Int__MD30_               -> PT10 (nivel medido)
%   Int_PT10__MD98_               -> Setpoint LC10 (consigna nivel)
%
%  NOTA: las etiquetas de TIA están "cruzadas":
%   - "Int_PT10" en realidad guarda el SP de LC10 (siempre = 50000 = 50%)
%   - "Int_FC10" en realidad guarda el SP de FC10 (salida del primario)
%   Esto se confirma observando los valores en el CSV.
% ============================================================

clear; clc; close all;

%% --------------------- CONFIGURACIÓN ------------------------
csv_file = "/home/yholi/control_procesos/labs/lab4/lab4_devguia/datas_cp_lab4/Trace_2_Parte2.csv";
out_dir  = "/home/yholi/control_procesos/labs/lab4/lab4_devguia/images";
Ts       = 0.1;       % período de muestreo del Trace (s)

if ~exist(out_dir, 'dir'); mkdir(out_dir); end

%% --------------------- LECTURA ----------------------
T = readtable(csv_file);
vars = T.Properties.VariableNames;

% Búsqueda EXACTA por nombre de columna (orden importa para diferenciar
% "Int_PT10" de "PT10_Int")
idx_ft10    = find(strcmp(vars, 'FT10_Int__MD22_'),    1);   % FT10 real
idx_pt10    = find(strcmp(vars, 'PT10_Int__MD30_'),    1);   % PT10 real (nivel)
idx_sp_fc10 = find(strcmp(vars, 'Int_FC10__MD102_'),   1);   % SP FC10
idx_u_fc10  = find(strcmp(vars, 'Int_EV11_CO__MD106_'),1);   % Esfuerzo EV11
idx_sp_lc10 = find(strcmp(vars, 'Int_PT10__MD98_'),    1);   % SP LC10

% Verificación: si alguno está vacío, abortamos con un mensaje claro
nombres   = {'FT10','PT10','SP_FC10','U_FC10','SP_LC10'};
indices   = {idx_ft10, idx_pt10, idx_sp_fc10, idx_u_fc10, idx_sp_lc10};
faltantes = nombres(cellfun(@isempty, indices));
if ~isempty(faltantes)
    fprintf('Columnas disponibles en el CSV:\n');
    for k = 1:numel(vars); fprintf('   %2d: %s\n', k, vars{k}); end
    error('No se encontraron columnas para: %s', strjoin(faltantes,', '));
end

% Lectura de cada variable (valores CRUDOS del PLC)
FT10    = T{:, idx_ft10};
PT10    = T{:, idx_pt10};
SP_FC10 = T{:, idx_sp_fc10};
U_FC10  = T{:, idx_u_fc10};
SP_LC10 = T{:, idx_sp_lc10};

N = height(T);
t_min = (0:N-1)' * Ts / 60;   % tiempo en MINUTOS

%% ---------------- LIMPIEZA DE VALORES ESPURIOS ----------------
FT10(abs(FT10)       > 1e6) = NaN;
PT10(abs(PT10)       > 1e6) = NaN;
SP_FC10(abs(SP_FC10) > 1e6) = NaN;
U_FC10(abs(U_FC10)   > 1e6) = NaN;
SP_LC10(abs(SP_LC10) > 1e6) = NaN;

%% ---------- ESTILO ----------
set(0, 'DefaultAxesFontSize', 11);

%% ============================================================
%  GRÁFICA estilo Trace TIA Portal
% ============================================================
fig = figure('Name','Trace Control Cascada', ...
             'Position',[50 50 1500 600], ...
             'Color','w');

hold on;

% Colores típicos del Trace TIA Portal
plot(t_min, FT10,    'Color',[1.00 0.20 0.40], 'LineWidth',1.0, ...
     'DisplayName','FT10 (caudal medido)');                    % rojo/rosa
plot(t_min, SP_FC10, 'Color',[0.20 0.65 1.00], 'LineWidth',1.0, ...
     'DisplayName','Setpoint FC10 (salida de LC10)');          % cyan
plot(t_min, U_FC10,  'Color',[0.15 0.75 0.20], 'LineWidth',0.8, ...
     'DisplayName','Esfuerzo FC10 EV11');          % verde
plot(t_min, PT10,    'Color',[0.10 0.30 0.85], 'LineWidth',1.2, ...
     'DisplayName','PT10 (nivel medido)');                     % azul
plot(t_min, SP_LC10, 'Color',[0.85 0.55 0.10], 'LineWidth',1.2, ...
     'DisplayName','Setpoint LC10 (consigna nivel)');          % naranja

grid on; box on;
set(gca, 'GridLineStyle','-', 'GridColor',[0.85 0.85 0.85], 'GridAlpha',1);
set(gca, 'MinorGridLineStyle',':', 'XMinorGrid','on', 'YMinorGrid','on');

xlabel('Tiempo [min]');
ylabel('Valor entero (DInt)');
title('Trace - Control en Cascada Nivel-Caudal (Siemens)');

legend('Location','northeastoutside','FontSize',9);
xlim([0 t_min(end)]);

ylim([min([FT10;SP_FC10;U_FC10;PT10;SP_LC10],[],'omitnan')-2000, ...
      max([FT10;SP_FC10;U_FC10;PT10;SP_LC10],[],'omitnan')+5000]);

exportgraphics(fig, fullfile(out_dir, 'proc_siemens_cascada_trace.png'), ...
               'Resolution', 150);

fprintf('\n[OK] Grafica estilo Trace (cascada) guardada en:\n   %s/proc_siemens_cascada_trace.png\n\n', out_dir);