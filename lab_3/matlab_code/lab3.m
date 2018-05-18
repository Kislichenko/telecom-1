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
%������� ������������� 800��
Fs=8e2; 
%���� ������� ���������� �������� �������
t=0:1/Fs:0.5; 

%� - ���������
A=2;
%������� ��������� 16 ��
f0=16;
%��������� ���� - 45 ��������
phi=pi/4;
%������� �������������� �������
s1=A*cos(2*pi*f0*t+phi);
%���������� ������ ���� � ������� �������
x_noise=awgn(s1,6);

%����� ������� ���������
figure;
subplot(2,2,1);
plot(t,s1);
title ('������ ���������');
xlabel('�����');
ylabel('���������');

subplot(2,2,2);
plot(t,x_noise);
title ('������ ��������� � �����');
xlabel('�����');
ylabel('���������');

%���������� ����� ����� �������
fftL=1000;
%������ ������ ��� ������� �������
f=0:Fs/fftL:Fs-1;
%�������������� ����� �� ������� s1 �� fftL ������
F=fft(s1,fftL);
%�������� ������
sp1=abs(F);
sp2=abs(fft(x_noise,fftL));

%������� ������
subplot(2,2,3);
plot(f,sp1(1:length(f)));
title ('������ ���������');
xlabel('������� (��)');
ylabel('���������');

subplot(2,2,4);
plot(f,sp2(1:length(f)));
title ('������ ��������� � �����');
xlabel('������� (��)');
ylabel('���������');

figure;
subplot(2,1,1);
y=filter(Hd,x_noise);
plot(t,x_noise, 'r', t, y, 'b');
title ('������ �� � ����� ����������');
xlabel('�����');
ylabel('���������');

subplot(2,1,2);
sp_filter=abs(fft(y,fftL));
plot(f,sp2(1:length(f)), 'r', f, sp_filter(1:length(f)), 'b');
title ('������ �� � ����� ����������');
xlabel('�����');
ylabel('���������');