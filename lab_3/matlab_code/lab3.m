%fdatool
function Hd = lab3
%UNTITLED Returns a discrete-time filter object.

% FIR Window Lowpass filter designed using the FIR1 function.

% All frequency values are in Hz.
Fs = 300;  % Sampling Frequency

Fpass = 5;               % Passband Frequency
Fstop = 20;               % Stopband Frequency
Dpass = 0.057501127785;  % Passband Ripple
Dstop = 0.0001;          % Stopband Attenuation
flag  = 'scale';         % Sampling Flag

% Calculate the order from the parameters using KAISERORD.
[N,Wn,BETA,TYPE] = kaiserord([Fpass Fstop]/(Fs/2), [1 0], [Dstop Dpass]);

% Calculate the coefficients using the FIR1 function.
b  = fir1(N, Wn, TYPE, kaiser(N+1, BETA), flag);
Hd = dfilt.dffir(b);
% [EOF]

clc;
close all;
%частота дискретизации 800Гц
Fs=8e2; 
%одна секунда дискретных значений времени
t=0:1/Fs:0.5; 

%А - амплитуда
A=2;
%частота колебаний 16 Гц
f0=16;
%начальная фаза - 45 градусов
phi=pi/4;
%задание гармонического сигнала
s1=A*cos(2*pi*f0*t+phi);
%добавление белого шума к вектору сигнала
x_noise=awgn(s1,6);

%вывод графика синусоиды
figure;
subplot(2,2,1);
plot(t,s1);
title ('Сигнал синусоиды');
xlabel('Время');
ylabel('Амплитуда');

subplot(2,2,2);
plot(t,x_noise);
title ('Сигнал синусоиды с шумом');
xlabel('Время');
ylabel('Амплитуда');

%Количество линий Фурье спектра
fftL=1000;
%вектор частот для расчета спектра
f=0:Fs/fftL:Fs-1;
%Преобразование Фурье от сигнала s1 по fftL точкам
F=fft(s1,fftL);
%вычислим спектр
sp1=abs(F);
sp2=abs(fft(x_noise,fftL));

%выведем спектр
subplot(2,2,3);
plot(f,sp1(1:length(f)));
title ('Спектр синусоиды');
xlabel('Частота (Гц)');
ylabel('Амплитуда');

subplot(2,2,4);
plot(f,sp2(1:length(f)));
title ('Спектр синусоиды с шумом');
xlabel('Частота (Гц)');
ylabel('Амплитуда');

figure;
subplot(2,1,1);
y=filter(Hd,x_noise);
plot(t,x_noise, 'r', t, y, 'b');
title ('Сигнал до и после фильтрации');
xlabel('Время');
ylabel('Амплитуда');

subplot(2,1,2);
sp_filter=abs(fft(y,fftL));
plot(f,sp2(1:length(f)), 'r', f, sp_filter(1:length(f)), 'b');
title ('Сигнал до и после фильтрации');
xlabel('Время');
ylabel('Амплитуда');