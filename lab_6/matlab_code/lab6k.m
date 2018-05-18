clc;
clear all;
close all;

msg = randi([3 7],[1 256])
sig_mod=pskmod(msg,8);
scatterplot(sig_mod);
err_signal = awgn(sig_mod,20);
%err_sig_mod = pskmod(err_signal,8);
figure;
scatterplot(err_signal);

y=pskdemod(sig_mod,8);
scatterplot(y);