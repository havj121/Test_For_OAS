classdef OAS_RealDriver < OAS_Driver
    %This class defines a real driver which contains only a method
    
    properties
        ID='RealDriver'
        % Basic information of the driver
        Name
        Age
        Gender
        PersonalTraits
    end
    
    methods
        function obj = OAS_RealDriver(Name,Gender,Age)
            % Constructor
            if nargin==1
                obj.Name=Name;
            elseif nargin==2
                obj.Name=Name;
                obj.Gender=Gender;
            elseif nargin==3
                obj.Name=Name;
                obj.Gender=Gender;
                obj.Age=Age;
            end
            obj.Driver_Type="Real";
            % register this driver object in the oas_handlemanager
            OAS_handlemanager=OAS_HandleManager.getInstance();
            OAS_handlemanager.register(obj.ID,obj);
        end
        
        function Evaluation = Evaluate(obj,Trajectory)
            %Implementation of the abstract method in the super class, and
            %the Evaluation comes from the GUI.
            Evaluation.Type="Compare";
            Evaluation.Result=0;
        end
    end
end

