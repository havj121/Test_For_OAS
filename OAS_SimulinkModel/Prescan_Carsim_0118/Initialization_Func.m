V=60/3.6; %车辆质心的纵向车速,km/h
load('osqpcecInputBus.mat');
%==============车辆模型参数=================
% ===========The parameters is same with the JTEKT Vehicle========
g=9.8;%重力加速度m/s^2
IH=0.03; %转向盘的转动惯量kg·m2
IC=2e-4; %转向柱的转动惯量kg·m2
IM=3e-4; %电机的转动惯量kg·m2
IW=1.3; %前轮的转动惯量（两个）kg·m2
CC=0.05; %转向柱的阻尼系数Nm·s/rad
CM=6.9e-4; %电机的阻尼系数Nm·s/rad
CW=25; %前轮的阻尼系数（两个）Nm·s/rad
FC=0.6; %转向柱的摩擦力矩Nm
FM=0.07; %电机的摩擦力矩Nm
FW=8; %前轮的摩擦力矩（两个）Nm
KC=1000; %转向柱的刚度Nm/rad
KTS=120; %扭矩传感器的刚度Nm/rad
KG=20000; %转向器的刚度Nm/rad
nM=17; %电机减速机构的传动比
% Alpha 转向盘转角
% Theta_C 转向柱转角
% Theta_M 电机转角
% Theta_P 小齿轮转角
% T_H 转向盘上的转向力矩
% T_W 前轮上的回正力矩（两个）
% T_armature 电机转子上的电磁转矩
%{
nG=16; %转向器的传动比
M=1396; %车辆的质量kg
I=2204; %车辆的横摆转动惯量kg·m2
Kf=50192; %前轮的侧偏刚度Nm/rad
Kr=50192; %后轮的侧偏刚度Nm/rad
Lf=1.01; %车辆质心到前轴的距离m
Lr=1.59; %车辆质心到后轴的距离m
%}

%=====The JTEKT vehicle Parameter
M=1275; %车辆的质量kg
I=1523; %车辆的横摆转动惯量kg·m2
Kf=20000; %Calculated According to the carsim vehicle tire (under Fn=3500N)
Kr=20000; %后轮的侧偏刚度Nm/rad
Lf=1.016; %车辆质心到前轴的距离m
Lr=1.644; %车辆质心到后轴的距离m
VehicleBaseLength=Lf+Lr;
nG=24; %转向器的传动比
%==============

zeta=0.03; %轮胎的总拖距m
D=0.01; %主销的内移量m
sigma=(11+50/60)*pi/180; %主销的内倾角rad
Fzf=M*g*Lr/(Lr+Lf);
% Beta 车辆质心处的侧偏角
% r 车辆的横摆角速度
% Delta 车辆的前轮转角
% Alpha_f 前轮的侧偏角
% Alpha_r 后轮的侧偏角
% F_yf 前轮的侧偏力
% F_yr 后轮的侧偏力
% F_zf 前轮的垂向载荷
% a_y 车辆质心处的侧向加速度
% T_w 回正力矩

%动力子模块
Cv=[M*V 0;
    0   I];
Kv=[2*(Kf+Kr)       (M*V+2*(Lf*Kf-Lr*Kr)/V);
    2*(Lf*Kf-Lr*Kr) 2*(Lf^2*Kf+Lr^2*Kr)/V];
Iv=[2*Kf;
    2*Lf*Kf];

%转向子模块
Ms=[IC  0         0;
    0   nM*nM*IM   0;
    0   0         IW];
Cs=[CC  0         0;
    0   nM*nM*CM   0;
    0   0         CW];
Ks=[KC+KTS  -KTS          0;
    -KTS    KTS+KG/nG/nG   -KG/nG;
    0       -KG/nG        KG];

%==============电机摩擦力矩补偿参数=================
friction_enable=1;
td=0.01;
sigma0_p=200;    %N/m 滞回段的斜率    变形的刚度系数
sigma1_p=(200)^0.5;   %Ns/m 滞回段的宽度与线性度，越小越宽，线性度越差 通过修改此值可以改变系统的阻尼率，变形率的阻尼系数      
Ff_c_p=0.4;     %Nm 补偿动摩擦力。与系统仿真稳定性有关，越接近mu_ab越稳定
Ff_ab_p=0.41;     %Nm 补偿最大静摩擦力。
theta_sb_p=0.03;   %rad/s 最大静摩擦力的“影响范围”.与系统仿真稳定性有关 越大越稳定 %改变最大静摩擦力作用范围
damping_p=0.113;   %Nms/rad 补偿阻尼大小

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