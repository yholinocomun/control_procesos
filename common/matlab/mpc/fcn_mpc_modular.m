function u = fcn(r, x)
%% ============================================================
%  Bloque MATLAB Function CORTO que llama a mpc_qp_step.m
%  (version modular: la logica vive en mpc_qp_step.m)
%
%  Requiere que mpc_qp_step.m este en el path de MATLAB.
%  SOLO editas Ad, Bd y N aqui; todo lo demas es automatico.
% ============================================================
coder.extrinsic('mpc_qp_step');

N = 5;                       % <-- HORIZONTE
Ad = [1.0000  0.1000;        % <-- MATRIZ DE ESTADOS
      0       1.0000];
Bd = [0.0050;                % <-- MATRIZ DE ENTRADAS
      0.1000];

u = 0;                       % inicializacion fija para el coder
u = mpc_qp_step(r, x, Ad, Bd, N);
end
