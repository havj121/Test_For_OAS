classdef OAS_Recorder < handle
    %This is a Recorder used to record the trajectory data and curve lane
    %data and corresponding indicator data and so on
    
    properties
        SimulinkModelObj
        OAS_SystemObj
        
        
        % all alternative ParamSet, the order of the parameternames should
        % be consistent with the simulink model
        ParameterNames={'Beta_PathLength','Beta_Curvature','Beta_Centering','Beta_HeadingError',...
    'LatAcceleration_UpperBound','LongidAcceleration_UpperBound','LongidDeceleration_UpperBound','Velocity_LowerBound','Velocity_UpperBound'};
        ParamSet
        Data
        
        CurrentParamID=0
        IndicatorBuffer
        
        RecordFilePath
        RecordFileName
    end
    
    % Constructor
    methods
        function obj = OAS_Recorder(OAS_SystemObj,SimulinkModelObj) 
            obj.OAS_SystemObj=OAS_SystemObj;
            obj.SimulinkModelObj = SimulinkModelObj;
            
            % add listerner for the oas_SimulinkMOdelobj
            obj.SimulinkModelObj.addlistener('EscapedCurve',@obj.RecordData);
%             obj.OAS_SystemObj.addlistener('EnteredCurve',@obj.RecordLaneData);
            
        end
    end
    
    methods
        function Start_Record(obj)
            obj.MakeSaveFolder();
            obj.MakeSaveFile();
            %Start the Simulink
            obj.SimulinkModelObj.Start_Model();
            obj.UpdateParamValue();
        end        
        function RecordData(obj,~,~)
            CurveLaneDataTable=obj.SimulinkModelObj.CurveLaneDataTable;
            TrajectoryDataTable=obj.SimulinkModelObj.TrajectoryDataTable;
            Paramtable=obj.SimulinkModelObj.GetParam();
            tabletemp=table({Paramtable},{CurveLaneDataTable},{TrajectoryDataTable});
            tabletemp.Properties.VariableNames=[{'PlanningParam'},{'CurveLaneData'},{'TrajectoryData'}];
            obj.Data=[obj.Data;tabletemp];
            obj.UpdateParamValue();
        end       
        function UpdateParamValue(obj)
            % assigin value for the planning param
            if isempty(obj.ParamSet)
                return
%                 errordlg('Specify the ParamSet First!');
            elseif obj.CurrentParamID>=size(obj.ParamSet,1)
                obj.SimulinkModelObj.Stop_Model();
                pause(15)
                try
                    obj.SimulinkModelObj.Close_Model();
                catch
                    pause(20)
                    obj.SimulinkModelObj.Close_Model();
                end
                obj.SaveFile();
            else
                obj.CurrentParamID=obj.CurrentParamID+1;
                currentparam=obj.ParamSet{obj.CurrentParamID,obj.ParameterNames};
                obj.SimulinkModelObj.UpdateParam(currentparam);
            end
            
        end
        function MakeSaveFolder(obj)
            % Select the Save File Position
            selpath=uigetdir;
            % Make a New File Folder to Store the Data
%             dlgtitle='Make A New Folder';
%             Prompt={'Enter the Folder Name:'};
%             NewFolderName=inputdlg(Prompt,dlgtitle);
%             status=mkdir(selpath,NewFolderName{1});
%             if status==1
%                 obj.RecordFilePath=fullfile(selpath,NewFolderName{1});
%             end
        obj.RecordFilePath=selpath;
        end
        function MakeSaveFile(obj)
            dlgtitle='Save File-mat';
            Prompt={'Enter the File Name:'};
            NewFileName=inputdlg(Prompt,dlgtitle);
            obj.RecordFileName=NewFileName{1};
            % create the save table
            obj.Data=table();
        end
        function SaveFile(obj)
            filename=fullfile(obj.RecordFilePath,obj.RecordFileName);
            Data=obj.Data;
            save(filename,'Data','-mat');
        end
    end
        
end

