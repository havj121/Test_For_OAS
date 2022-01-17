V=60/3.6; %�������ĵ�������,km/h
load('osqpcecInputBus.mat');
%==============����ģ�Ͳ���=================
% ===========The parameters is same with the JTEKT Vehicle========
g=9.8;%�������ٶ�m/s^2
IH=0.03; %ת���̵�ת������kg��m2
IC=2e-4; %ת������ת������kg��m2
IM=3e-4; %�����ת������kg��m2
IW=1.3; %ǰ�ֵ�ת��������������kg��m2
CC=0.05; %ת����������ϵ��Nm��s/rad
CM=6.9e-4; %���������ϵ��Nm��s/rad
CW=25; %ǰ�ֵ�����ϵ����������Nm��s/rad
FC=0.6; %ת������Ħ������Nm
FM=0.07; %�����Ħ������Nm
FW=8; %ǰ�ֵ�Ħ�����أ�������Nm
KC=1000; %ת�����ĸն�Nm/rad
KTS=120; %Ť�ش������ĸն�Nm/rad
KG=20000; %ת�����ĸն�Nm/rad
nM=17; %������ٻ����Ĵ�����
% Alpha ת����ת��
% Theta_C ת����ת��
% Theta_M ���ת��
% Theta_P С����ת��
% T_H ת�����ϵ�ת������
% T_W ǰ���ϵĻ������أ�������
% T_armature ���ת���ϵĵ��ת��
%{
nG=16; %ת�����Ĵ�����
M=1396; %����������kg
I=2204; %�����ĺ��ת������kg��m2
Kf=50192; %ǰ�ֵĲ�ƫ�ն�Nm/rad
Kr=50192; %���ֵĲ�ƫ�ն�Nm/rad
Lf=1.01; %�������ĵ�ǰ��ľ���m
Lr=1.59; %�������ĵ�����ľ���m
%}

%=====The JTEKT vehicle Parameter
M=1275; %����������kg
I=1523; %�����ĺ��ת������kg��m2
Kf=20000; %Calculated According to the carsim vehicle tire (under Fn=3500N)
Kr=20000; %���ֵĲ�ƫ�ն�Nm/rad
Lf=1.016; %�������ĵ�ǰ��ľ���m
Lr=1.644; %�������ĵ�����ľ���m
VehicleBaseLength=Lf+Lr;
nG=24; %ת�����Ĵ�����
%==============

zeta=0.03; %��̥�����Ͼ�m
D=0.01; %������������m
sigma=(11+50/60)*pi/180; %�����������rad
Fzf=M*g*Lr/(Lr+Lf);
% Beta �������Ĵ��Ĳ�ƫ��
% r �����ĺ�ڽ��ٶ�
% Delta ������ǰ��ת��
% Alpha_f ǰ�ֵĲ�ƫ��
% Alpha_r ���ֵĲ�ƫ��
% F_yf ǰ�ֵĲ�ƫ��
% F_yr ���ֵĲ�ƫ��
% F_zf ǰ�ֵĴ����غ�
% a_y �������Ĵ��Ĳ�����ٶ�
% T_w ��������

%������ģ��
Cv=[M*V 0;
    0   I];
Kv=[2*(Kf+Kr)       (M*V+2*(Lf*Kf-Lr*Kr)/V);
    2*(Lf*Kf-Lr*Kr) 2*(Lf^2*Kf+Lr^2*Kr)/V];
Iv=[2*Kf;
    2*Lf*Kf];

%ת����ģ��
Ms=[IC  0         0;
    0   nM*nM*IM   0;
    0   0         IW];
Cs=[CC  0         0;
    0   nM*nM*CM   0;
    0   0         CW];
Ks=[KC+KTS  -KTS          0;
    -KTS    KTS+KG/nG/nG   -KG/nG;
    0       -KG/nG        KG];

%==============���Ħ�����ز�������=================
friction_enable=1;
td=0.01;
sigma0_p=200;    %N/m �ͻضε�б��    ���εĸն�ϵ��
sigma1_p=(200)^0.5;   %Ns/m �ͻضεĿ�������Զȣ�ԽСԽ�����Զ�Խ�� ͨ���޸Ĵ�ֵ���Ըı�ϵͳ�������ʣ������ʵ�����ϵ��      
Ff_c_p=0.4;     %Nm ������Ħ��������ϵͳ�����ȶ����йأ�Խ�ӽ�mu_abԽ�ȶ�
Ff_ab_p=0.41;     %Nm �������Ħ������
theta_sb_p=0.03;   %rad/s ���Ħ�����ġ�Ӱ�췶Χ��.��ϵͳ�����ȶ����й� Խ��Խ�ȶ� %�ı����Ħ�������÷�Χ
damping_p=0.113;   %Nms/rad ���������С

%============== Initialization=======================
% note: the name must be consistent with the OAS_Simulink Model method
% UpdateParam!!
InitialPlanningParameter=[0,0,1,0,2,2,2,40,60]'; 
Initial_Beta1=0;
Initial_Beta2=0;
Initial_Beta3=1;
Initial_Beta4=0;
Initial_MaxLateralAceel=2;
Initial_MaxLongitudinalAccel=2;
Initial_MaxLongitudinalDecel=2;
Initial_MinSpeed=40;
Initial_MaxSpeed=60;