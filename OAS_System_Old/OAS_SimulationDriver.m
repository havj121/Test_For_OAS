classdef OAS_SimulationDriver < OAS_Driver
    % This is a class that defines a simulation driver model with the DEM and DPM
    properties
        ID='SimulationDriver'
         %Properties For Simulation Driver only
        Ration_Type="Complete_Rational"
        SimPreferenceCoef=[0.1,0.4,0.5]
        SimPreferenceCoef_Safety=1
        SimPreferenceCoef_Comfort=1
        SimPreferenceCoef_Efficiency=1
        Rational_Coefficient_Simulation=10
        
        % Trajectory Index for Safety,Comfort and Efficiency
        Safety_Var={'LateralOffset'}
        Comfort_Var={'LateralAcceleration'}
        Efficiency_Var={'Time'}                
        % JND-just noticeable difference
        JNDs=struct('LateralAcceleration',0.1,'LateralOffset',0.1,'Time',2)% uinis: m/s2,m,s
        % Can You Feel The Difference? The Just Noticeable Difference of
        % Longitudinal Acceleration, http://sage.cnpereading.com/paragraph/article/?doi=10.1177/1541931213571271  
        
        Indicator_Range
        NormalizeFunc=@Trajectory_Normalization% function handle
        LikelihoodFunc        
    end
    
    properties (Dependent)
        % Simulation Driver Only
        Perception_Var
    end

    methods
        % Contructors
        function obj=OAS_SimulationDriver(Simpreference)
            if nargin==1    
                obj.SimPreferenceCoef=Simpreference;
            end
            % register this driver object in the oas_handlemanager
            OAS_handlemanager=OAS_HandleManager.getInstance();
            OAS_handlemanager.register(obj.ID,obj);
        end
    end
    
    
    methods      
        % ================== Evaluation Model================
        function Evaluation=Evaluate(obj,tarjectory,varargin)
            % Return a struct with fields Type and Result
            SubPerception_Normalised=obj.Perception(tarjectory);
            %
            % Need Further Completion
            %
            % Choose the Evaluation Method
            switch varargin
                case{[],"Compare"}
                    Query=obj.Query_Generate(SubPerception_Normalised);
                    Evaluation_Result=obj.Compare(Query);
                    Evaluation_Type="Compare";
                otherwise
                    Evaluation_Result=0;
                    Evaluation_Type="Other";
            end
            Evaluation.Type=Evaluation_Type;
            Evaluation.Result=Evaluation_Result;
        end

        function Compare_Result=Compare(obj,Query)
            %
            % Need Further Completion
            %
            switch obj.Ration_Type
                case "Probability_Rational"
                    Compare_Result=0;
                case "Bound_Rational"
                    Compare_Result=0;
                case "Complete_Rational"
                    Compare_Result=0;
                otherwise
                    Compare_Result=0;
            end        
        end
        
        
        %=============Perception Model===============
        function Sub_perception=Perception(obj,Trajectory_Ind)
            % Only for "Simulation Driver"
            % input the trajectory, which is data array with each column is
            % a variable sequence during the curve time.
            safety=obj.Safety_Perception(Trajectory_Ind);
            comfort=obj.Comfort_Perception(Trajectory_Ind);
            efficiency=obj.Efficiency_Perception(Trajectory_Ind);
            Sub_perception=[safety,comfort,efficiency]';
        end
                        
        function Subjective_Safety=Safety_Perception(obj,Trajectory_Ind)
            Safety_Ind=Trajectory_Ind(:,obj.Safety_Index);
            Safety_NormalisedInd=obj.NormaizeInd(Safety_Ind,obj.Perception_Var.Safety);
            Subjective_Safety=obj.SimPreferenceCoef_Safety'*Safety_NormalisedInd;           
        end
        
        function Subjective_Comfort=Comfort_Perception(obj,Trajectory_Ind)  
            Comfort_Ind=Trajectory_Ind(:,obj.Comfort_Index);
            Comfort_NormalisedInd=obj.NormaizeInd(Comfort_Ind,obj.Perception_Var.Comfort);
            Subjective_Comfort=obj.SimPreferenceCoef_Comfort'.*Comfort_NormalisedInd;           
        end
        
        function Subjective_Efficiency=Efficiency_Perception(obj,Trajectory_Ind)
            Efficiency_Ind=Trajectory_Ind(:,obj.Efficiency_Index);
            Efficiency_NormalisedInd=obj.NormaizeInd(Efficiency_Ind,obj.Perception_Var.Efficiency);
            Subjective_Efficiency=obj.SimPreferenceCoef_Efficiency'.*Efficiency_NormalisedInd;           
        end
        
       %================== Indicators================

        
        function NormalisedIndicator = NormaizeInd(obj,Trajectory_Ind,Variables)
            % Normalise the trjectory indicators within 0-1
            Var_JND=zeros(1,length(Variables));
            for i=1:length(Variables)
                Var_JND(i)=obj.JNDs(Variables{i});
            end
            NormalisedIndicator=Trajectory_Normalization(IndicatorDataBase,Trajectory_Ind,Variables,Var_JND);
        end
        
    end
         
    methods   
        %============Set and Get function==========        
        function Perception_Var=get.Perception_Var(obj)
            Perception_Var=[obj.Safety_Var,obj.Comfort_Var,obj.Efficiency_Var];
        end
        
        function set.SimPreferenceCoef(obj,val)
            if not(sum(val)==1)
                error('The sum of the val should equals to 1!\n');
            end
            obj.SimPreferenceCoef=val;       
        end
        
        
        function set.SimPreferenceCoef_Safety(obj,val)
            if not(sum(val)==1)
                error('The sum of the val should equals to 1!\n');
            end
            obj.SimPreferenceCoef_Safety=val;       
        end
        
        function set.SimPreferenceCoef_Comfort(obj,val)
            if not(sum(val)==1)
                error('The sum of the val should equals to 1!\n');
            end
            obj.SimPreferenceCoef_Comfort=val;       
        end
        
        function set.SimPreferenceCoef_Efficiency(obj,val)
            if not(sum(val)==1)
                error('The sum of the val should equals to 1!\n');
            end
            obj.SimPreferenceCoef_Efficiency=val;       
        end
        
    end
           
end




