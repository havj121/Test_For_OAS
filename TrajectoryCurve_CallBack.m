function TrajectoryCurve_CallBack(block,~)
% This is a callback function which deals the curve trajectory data when
% the escape the curve road!

%=========== send the trajectory data to base workspace =============
% This make is possible for for all object to get available of this data.
Data_Curve=block.OutputPort(1).Data;
VarNames={'t','Vx','Vy','Vz','Ax','Ay','Az','AVx','AVy','AVz','AAx','AAy','AAz','LateralOffset','X','Y','Yaw'};
TrajectoryDataTable=array2table(Data_Curve,'VariableNames',VarNames);
ID_Valid=1:(find(TrajectoryDataTable.t==0,1)-1);
TrajectoryDataTable=TrajectoryDataTable(ID_Valid,:);
%

% ============ Notify the GUI that Escaped Curve===========
% Method 1 to get the simulink model handle
%{
guifigure=findall(0,'Name','OAS_GUI');
oas_guiobj=guifigure.UserData;
SimulinkModelobj=oas_guiobj.OAS_SimulinkModelobj;
%}
% Method 2 to get the simulink model handle
try
    OAS_handlemanager=OAS_HandleManager.getInstance();
    SimulinkModelobj=OAS_handlemanager.getData('SimulinkModel');
    ed=OAS_EventData(TrajectoryDataTable);
    SimulinkModelobj.notify('EscapedCurve',ed); 
catch
    assignin('base','CurveTrajectoryDateTable',TrajectoryDataTable);
    return
end



end