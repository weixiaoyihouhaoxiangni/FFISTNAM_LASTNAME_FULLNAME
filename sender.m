close all
clear all
clc

t = tcpip('127.0.0.1',30000);
% Open socket and wait before sending data
fopen(t);
pause(0.2);

% Send data every 500ms
N = 0;
while N < 10
DataToSend= "Hello mainwindow";
fwrite(t,DataToSend);
pause (0.5);
N = N + 1;
end