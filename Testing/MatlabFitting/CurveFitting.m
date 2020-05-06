%Requires the curve fitting toolbox to be installed


clc;
clear;
[Distance1, Sensor1Reading,Distance3, Sensor3Reading] = readvars('IRSensorTesting.xlsx');

%Select a two term, exponential fit to get correct values.
cftool(Distance1, Sensor1Reading);

cftool(Distance3,Sensor3Reading);