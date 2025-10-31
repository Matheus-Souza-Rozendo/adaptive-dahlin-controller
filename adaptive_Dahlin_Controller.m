clear;
clc;
close all;

fs = 10;
T = 1/fs;

% Planta
R1=9.96*10^3;
R2=9.79*10^3;
C=448*10^-6;
K = R1/(R1+R2);
Tau = C*R1*R2/(R1+R2);
c =-exp(-T/Tau);
a1 = c;
b1 = K*(1+c);
tau_linha = Tau/2;
Kp = 1;
cp = -exp(-T/tau_linha);

% Sinal de entrada
tmax = 50;
tempo = 0:T:(tmax-T);
N = numel(tempo);

r = zeros(1,N);
r(tempo < 10) = 0.5;
r(tempo >= 10 & tempo < 20) = 1;
r(tempo >= 20 & tempo < 30) = 0.75;
r(tempo >= 30 & tempo < 40) = 1.5;
r(tempo >= 40 & tempo < 50) = 0;

% === MALHA FECHADA (COM CONTROLE ADAPTATIVO DAHLIN + MQR) ===
y = zeros(1,N);
u = zeros(1,N);
e = zeros(1,N);

rho = 500;
P = rho * eye(2);
v = zeros(2,1);
theta = zeros(2,N);
lambda = 0.95;

for k = 3:N-1
    % Saída do sistema
    y(k) = b1*u(k-2) - a1*y(k-1);

    % Estimador MQR
    v = [u(k-2); -y(k-1)];
    h = (P * v) / (lambda + v' * P * v);
    theta(:, k) = theta(:, k-1) + h * (y(k) - v.' * theta(:, k-1));
    P = (1/lambda) * (P - (P * v * v' * P) / (lambda + v' * P * v));

    % Sinal de erro
    e(k) = r(k) - y(k);

    % Controlador
    if (tempo(k) <= 5)
        u(k) = r(k);
    else
        b1e = theta(1,k);
        a1e = theta(2,k);

        % Controlador Dahlin adaptativo
        alfa1 = (Kp * (1 + cp)) / b1e;
        alfa2 = (a1e * Kp * (1 + cp)) / b1e;
        alfa3 = -cp;
        alfa4 = Kp * (1 + cp);
        u(k) = alfa3*u(k-1) + alfa4*u(k-2) + alfa1*e(k) + alfa2*e(k-1);
    end
end

% === MALHA ABERTA ===
y_oa = zeros(1,N);  % saída malha aberta
u_oa = r;           % entrada direta

for k = 3:N
    y_oa(k) = b1*u_oa(k-2) - a1*y_oa(k-1);
end

% === Gráfico 1: Comparação das respostas ===
figure;
plot(tempo, r, 'k--', 'LineWidth', 1.5); hold on;
plot(tempo, y, 'b-', 'LineWidth', 2);
plot(tempo, y_oa, 'm:', 'LineWidth', 2);
xlabel('Tempo [s]');
ylabel('Amplitude');
title('Comparação: Malha Fechada (Adaptativo) vs. Malha Aberta');
legend('Referência', 'Saída Malha Fechada', 'Saída Malha Aberta');
grid on;

% === Gráfico 2: Sinal de erro e controle (Malha Fechada) ===
figure;
yyaxis left;
plot(tempo, e, 'r', 'LineWidth', 1.5);
ylabel('Erro');

yyaxis right;
plot(tempo, u, 'g', 'LineWidth', 1.5);
ylabel('Sinal de Controle');

xlabel('Tempo [s]');
title('Sinal de Erro e Controle (Malha Fechada)');
legend('Erro', 'Controle');
grid on;

% === Gráfico 3: Estimativas dos parâmetros (MQR) ===
figure;
plot(tempo, theta(1,:), 'm-', 'LineWidth', 1.5); hold on;
plot(tempo, theta(2,:), 'c--', 'LineWidth', 1.5);
xlabel('Tempo [s]');
ylabel('Parâmetro estimado');
title('Evolução dos Parâmetros Estimados (MQR)');
legend('b_1 estimado', 'a_1 estimado');
grid on;
