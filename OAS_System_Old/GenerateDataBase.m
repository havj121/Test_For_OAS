% Generate the IndicatorPool and IndicatorParameterLookUpTable from the
% parameterindicator table;
clear;
clc;
addpath('../Prescan_Carsim_1027');
Parameter_IndicatorTable=readtable('../Prescan_Carsim_1027\Parameter_IndicatorAll.csv');

JNDs=struct('LateralAcceleration',0.15,'LongitudinalAcceleration',0.15,'MaxLateralOffset_Left',0.1,'MaxLateralOffset_Right',0.1,'Time',2);% uinis: m/s2,m,s
[NormalisedParam_IndBaseTable,IndicatorRange]=Generate_NormalisedIndicatorDataBase(Parameter_IndicatorTable,JNDs);
writetable(NormalisedParam_IndBaseTable,'NormalisedParam_IndBaseTable.csv','Delimiter',',');
