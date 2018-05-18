function Hd = cheb_2
%CHEB_2 Returns a discrete-time filter object.

% Chebyshev Type II Lowpass filter designed using FDESIGN.LOWPASS.

% All frequency values are in Hz.
Fs = 150;  % Sampling Frequency

Fpass = 5;           % Passband Frequency
Fstop = 20;           % Stopband Frequency
Apass = 1;           % Passband Ripple (dB)
Astop = 80;          % Stopband Attenuation (dB)
match = 'stopband';  % Band to match exactly

% Construct an FDESIGN object and call its CHEBY2 method.
h  = fdesign.lowpass(Fpass, Fstop, Apass, Astop, Fs);
Hd = design(h, 'cheby2', 'MatchExactly', match);

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