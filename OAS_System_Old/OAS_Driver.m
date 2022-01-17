classdef OAS_Driver< handle
    %This is an abstract driver class, which the real drivers and simulated
    %drivers comes from
    
    properties
        Driver_Type
        % Buffer of driver comparison history
        Comparing_Buffer
        LastPreference_Trajectory
    end
    
    events
        DriverEvaluated
    end
    
%     methods
%         function obj = OAS_Driver(inputArg1,inputArg2)
%             %Constructor
%             %   此处显示详细说明
%             obj.Property1 = inputArg1 + inputArg2;
%         end
%     end

    methods (Abstract)
            Evaluate(obj,Trajectory);
    end
    
    
    
end

