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


% === MALHA FECHADA ===
y = zeros(1,N);
u = zeros(1,N);
e = zeros(1,N);

for k = 3:N
    % Saída do sistema (planta)
    y(k) = b1*u(k-2) - a1*y(k-1);

    % Erro
    e(k) = r(k) - y(k);

    % Controlador Dahlin
    alfa1 = (Kp*(1+cp))/b1;
    alfa2 = (a1*Kp*(1+cp))/b1;
    alfa3 = -cp;
    alfa4 = Kp*(1+cp);
    u(k) = alfa3*u(k-1) + alfa4*u(k-2) + alfa1*e(k) + alfa2*e(k-1);
    if u(k) > 5
        u(k)=5;
    end 
    if u(k)<0
        u(k)=0;
    end
end

% === MALHA ABERTA ===
y_oa = zeros(1,N);  % saída malha aberta
u_oa = r;           % entrada = referência (sem controle)

for k = 3:N
    y_oa(k) = b1*u_oa(k-2) - a1*y_oa(k-1);
end



% === Gráfico: Comparação das respostas ===
figure;
plot(tempo, r, 'k--', 'LineWidth', 1.5); hold on;
plot(tempo, y, 'b', 'LineWidth', 2);
plot(tempo, y_oa, 'm:', 'LineWidth', 2);
xlabel('Tempo [s]');
ylabel('Amplitude');
title('Comparação: Malha Fechada (Dahlin) vs. Malha Aberta');
legend('Referência', 'Malha Fechada', 'Malha Aberta');
grid on;

% === Gráfico: Sinal de erro e controle ===
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
