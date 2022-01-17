classdef EscapeCurveData < event.EventData
    %This is a class that defines the data that send to the OAS_GUI by the
    %simulink model, it is about the trajectory data during the curve.
    
    
    properties
        Data
        VarName
        
    end
    
    methods
        function obj = EscapeCurveData(Data,VarName)
            % Constructor
            obj.Data=Data;
            obj.VarName=VarName;
        end
        
    end
end

