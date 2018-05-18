clc;
close all;
clear all;

%BPSK
%генерируем случайное сообщения
msg=randi([0 1], [1 256]);
%M=2 получим сигнал от двоичного фазового модулятора
sig_mod=pskmod(msg,2);
%получим сигнальное созвездие BPSK
scatterplot(sig_mod);
%оценим помехоустойчивость
%смоделируем ошибки
errors=randerr(1,256,13);
%добавим ошибки к сигналу
sig_mod_errors=(sig_mod+errors);
%Полученный сигнал
sig_demod=pskdemod(sig_mod_errors,2);
%вычислим число ошибочных символов 
%и вероятности ошибки на символ
[numberBPSK, ratioBPSK]=symerr(msg,sig_demod);
[BITnumberBPSK, BITratioBPSK]=biterr(msg,sig_demod);
%число ошибочных символов
numberBPSK
%вероятность ошибки на символ
ratioBPSK

%PSK
%генерируем случайное сообщения
msg=randi([0 7], [1 256]);
%M=8 получим сигнал от двоичного фазового модулятора
sig_mod=pskmod(msg,8);
%получим сигнальное созвездие BPSK
scatterplot(sig_mod);
%оценим помехоустойчивость
%смоделируем ошибки
errors=randerr(1,256,13);
%добавим ошибки к сигналу
sig_mod_errors=sig_mod+errors;
%Полученный сигнал
sig_demod=pskdemod(sig_mod_errors,8);
%вычислим число ошибочных символов 
%и вероятности ошибки на символ
[numberPSK, ratioPSK]=symerr(msg,sig_demod);
[BITnumberPSK, BITratioPSK]=biterr(msg,sig_demod);
%число ошибочных символов
numberPSK
%вероятность ошибки на символ
ratioPSK

%OQPSK
%генерируем случайное сообщения
msg=randi([0 3], [1 256]);
%M=4 получим сигнал от двоичного фазового модулятора
%второй ini_phase задает начальную 
%фазу комплексной огибающей в радианах
sig_mod=oqpskmod(msg,pi/2);
%получим сигнальное созвездие BPSK
scatterplot(sig_mod);
%оценим помехоустойчивость
%смоделируем ошибки
errors=randerr(1,513,13);
%добавим ошибки к сигналу
sig_mod_errors=sig_mod+errors;
%Полученный сигнал
sig_demod=oqpskdemod(sig_mod_errors,pi/2);
%вычислим число ошибочных символов 
%и вероятности ошибки на символ
[numberOQPSK, ratioOQPSK]=symerr(msg,sig_demod);
%число ошибочных символов
numberOQPSK
%вероятность ошибки на символ
ratioOQPSK


%genQAM
%генерируем случайное сообщения
M=8;
msg=randi([0 M-1], [1 256]);
%M=8 получим сигнал от двоичного фазового модулятора
sig_mod=genqammod(msg,exp(j*[0:M-1]));
%получим сигнальное созвездие BPSK
scatterplot(sig_mod);
%оценим помехоустойчивость
%смоделируем ошибки
errors=randerr(1,256,13);
%добавим ошибки к сигналу
sig_mod_errors=sig_mod+errors;
%Полученный сигнал
sig_demod=genqamdemod(sig_mod_errors,exp(j*[0:M-1]));
%вычислим число ошибочных символов 
%и вероятности ошибки на символ
[numbergenQAM, ratiogenQAM]=symerr(msg,sig_demod);
[BITnumbergenQAM, BITratiogenQAM]=biterr(msg,sig_demod);
%число ошибочных символов
numbergenQAM
%вероятность ошибки на символ
ratiogenQAM

%MSK
%генерируем случайное сообщения
%элементы входной информации могут принимать 1 или 0
msg=randi([0 1], [1 256]);
%M=2 получим сигнал от двоичного фазового модулятора
%второй параметр - число отсчетов рез.вектора
sig_mod=mskmod(msg,16);
%получим сигнальное созвездие BPSK
scatterplot(sig_mod);
%оценим помехоустойчивость
%смоделируем ошибки
errors=randerr(1,256*16,13);
%добавим ошибки к сигналу
sig_mod_errors=sig_mod+errors;
%Полученный сигнал
sig_demod=mskdemod(sig_mod_errors,16);
%вычислим число ошибочных символов 
%и вероятности ошибки на символ
[numberMSK, ratioMSK]=symerr(msg,sig_demod);
%число ошибочных символов
numberMSK
%вероятность ошибки на символ
ratioMSK

%FSK
%генерируем случайное сообщения
%элементы входной информации могут принимать 1 или 0
msg=randi([0 1], [1 256]);
%M=2 получим сигнал от двоичного фазового модулятора
%второй параметр - число отсчетов рез.вектора
M=2; % число позиций манипуляции
freqsep=4; % разнос частот манипуляции (в герцах)
nsamp=4; % число отсчетов на символ
Fs=32; % частота дискретизации (в герцах)
sig_mod=fskmod(msg,M,freqsep,nsamp,Fs);
%получим сигнальное созвездие BPSK
scatterplot(sig_mod);
%оценим помехоустойчивость
%смоделируем ошибки
errors=randerr(1,256*nsamp,13);
%добавим ошибки к сигналу
sig_mod_errors=sig_mod+errors;
%Полученный сигнал
sig_demod=fskdemod(sig_mod_errors,M,freqsep,nsamp,Fs);
%вычислим число ошибочных символов 
%и вероятности ошибки на символ
[numberFSK, ratioFSK]=symerr(msg,sig_demod);
%число ошибочных символов
numberFSK
%вероятность ошибки на символ
ratioFSK

