function CurveLane_CallBack(block,~)
% This is a callback function which deals the curve lane data when
% the enter the curve road!

%=========== send the trajectory data to base workspace =============
% This make is possible for for all object to get available of this data.
CurveLaneData=block.OutputPort(1).Data;
VarNames={'Left_X','Left_Y','Right_X','Right_Y'};
CurveLaneDataTable=array2table(CurveLaneData,'VariableNames',VarNames);
%

% ============ Notify Entered Curve===========
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
    ed=OAS_EventData(CurveLaneDataTable);
    SimulinkModelobj.notify('EnteredCurve',ed); 
catch
    assignin('base','CurveLaneDateTable',CurveLaneDataTable);
    return
end
end