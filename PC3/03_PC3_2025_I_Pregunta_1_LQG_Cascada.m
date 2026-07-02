%% Practica Calificada 3 - 2025 I
%% Pregunta 1: Desacoplamiento, LQG y control en cascada
clear all; close all; clc;

% Item a): controlador por desacoplamiento
s = tf('s');
Gp = 1/(10*s^2);

tau = 5/4;
Gl = 1/(tau*s);
Gc = inv(Gp)*Gl;

Gt = feedback(Gc*Gp, 1);
figure;
step(Gt)
title('PC3 2025-I P1: Desacoplado')
stepinfo(Gt)

% Item b): LQG con accion integral
Sp = minreal(ss(Gp));
A = Sp.A; B = Sp.B;
C = Sp.C; D = Sp.D;

Ae = [A, zeros(2,1); -C, 0];
Be = [B; 0];

Q = diag([1 10 100]); R = 0.1;
[Ke, ~] = lqr(Ae, Be, Q, R);
K = Ke(1:end-1);
Ki = -Ke(end);

G = eye(2);
Qn = diag([1 1]); Rn = 1;
L = lqe(A, G, C, Qn, Rn);

At = [A, -B*K, Ki*B;
      L*C, A-B*K-L*C, Ki*B;
      -C, zeros(1,2), 0];
Bt = [zeros(2,1); zeros(2,1); 1];
Ct = [C, zeros(1,2), 0];
Dt = D;
St = ss(At, Bt, Ct, Dt);
figure;
step(St);
title('PC3 2025-I P1: LQG')

% Item c): control en cascada desacoplado-LQG
Gp2 = 1/(10*s);
tau = 4/4;
Gl2 = 1/(tau*s);
Gc2 = inv(Gp2)*Gl2;

Gt2 = feedback(Gc2*Gp2, 1);
figure;
step(Gt2)
title('PC3 2025-I P1: Lazo interno')
stepinfo(Gt2)

Gp1 = 1/s;
Sp1 = minreal(ss(Gt2*Gp1));
A = Sp1.A; B = Sp1.B;
C = Sp1.C; D = Sp1.D;
zpk(Sp1)

Ae = [A, zeros(2,1); -C, 0];
Be = [B; 0];

Q = diag([1 10 100]); R = 0.1;
[Ke, ~] = lqr(Ae, Be, Q, R);
K = Ke(1:end-1);
Ki = -Ke(end);

G = eye(2);
Qn = diag([1 1]); Rn = 1;
L = lqe(A, G, C, Qn, Rn);

At = [A, -B*K, Ki*B;
      L*C, A-B*K-L*C, Ki*B;
      -C, zeros(1,2), 0];
Bt = [zeros(2,1); zeros(2,1); 1];
Ct = [C, zeros(1,2), 0];
Dt = D;
St1 = ss(At, Bt, Ct, Dt);
figure;
step(St1);
title('PC3 2025-I P1: Desacoplado-LQG en cascada')
