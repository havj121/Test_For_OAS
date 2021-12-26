% The main function of the OAS system function
% addpath('../Prescan_Carsim_1027');
%==============The Trajectory Indicators Pool and Initial PDF of DEM and DPM=================
% NormalisedParam_IndBaseTable=readtable('NormalisedParam_IndBaseTable.csv');

%%
% Instance a OAS_HandleManager
OAS_handlemanager=OAS_HandleManager.getInstance();
% Construct a Driver Object
ARealDriver=OAS_RealDriver();
% AOASSystemObj=OAS_System(ASimulationDriver,InitialPDF_DEM,InitialDPMPDF_Safety,InitialDPMPDF_Comfort,InitialDPMPDF_Efficiency);
oassystemobj=OAS_System(ARealDriver);
AGUI=OAS_GUI(oassystemobj);


%%
% finally,delete the Current OAS_handlemanager
delete(OAS_handlemanager)





























