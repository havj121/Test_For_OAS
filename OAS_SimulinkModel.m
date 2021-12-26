classdef OAS_SimulinkModel < handle
    %This is a Simulink Model class which runs the trajectory planning and
    %tracking model.
    
    properties
        ID='SimulinkModel'
        FilePath
        ModelName
        ModelHandle
        % 
        
        
        % Trajectory Data Information
        % maybe it is ok to send a data table
        TrajectoryData
        TrajectoryVariableNames
        CurveLaneData
        CurveLaneVariableNames
  
        ParamSetHandle
        
        % Trajectory Indicator and planning parameter lookup table,the No
        % of the planning parameter in the param_ind database.
        EvaluatedParamBuffer
        EValuatedParam_IndID
        
    end
    properties (Dependent)
        TrajectoryDataTable
        CurveLaneDataTable
        RoadWidth
        VehicleWidth
        VehicleBaseLength
    end
    
    
    events
        EscapedCurve % escape the curve
        EnteredCurve
    end
    
    
    methods
        function obj = OAS_SimulinkModel(ModelName)
            %Constructor
            if nargin==1
                obj.ModelName=ModelName;
            end
            
            % register this driver object in the oas_handlemanager
            OAS_handlemanager=OAS_HandleManager.getInstance();
            OAS_handlemanager.register(obj.ID,obj);
        end        
    end
    
    methods
        function Flag=select_Model(obj)
            [file,path]=uigetfile({'*.slx','*.mdl'},'File Selector');
            if isequal(file,0)
                disp('User selected cancel');
                Flag=0;
            else
                obj.FilePath=fullfile(path,file);
                Flag=1;
            end

        end

        function Open_Model(obj)
            systemhandle=load_system(obj.FilePath);
            obj.ModelHandle=systemhandle;
            open_system(obj.ModelHandle);
            % register the paramset handle
            paramhandle=Simulink.findBlocks(systemhandle,'Name','Init_PlanningParameter');
            obj.ParamSetHandle=paramhandle;
            
            

        end
        function Close_Model(obj)
            close_system(obj.ModelHandle);            
        end        
        function Start_Model(obj)
            set(obj.ModelHandle,'SimulationCommand','start');
        end       
        function Continue_Model(obj)
            set(obj.ModelHandle,'SimulationCommand','continue');
        end      
        function Pause_Model(obj)
            set(obj.ModelHandle,'SimulationCommand','pause');
        end
        function Stop_Model(obj)
            set(obj.ModelHandle,'SimulationCommand','stop');
        end
        function AddListener_Model(obj)
            % add listener
            outblockhandle=Simulink.findBlocks(obj.ModelHandle,'Name','trajectorycurveoutput_Gain');
            addstartfcn=['CurveListenerHandle=add_exec_event_listener(''',getfullname(outblockhandle),''',''PostOutPuts'',@TrajectoryCurve_CallBack);'];
            startfcn=get_param(systemhandle,'StartFcn');
            newstartfcn=[startfcn,addstartfcn];
            set_param(systemhandle,'StartFcn',newstartfcn);
        end       
        function [TLC_Left,TLC_Right]=CalculateTLC(obj,CurveLaneData,CurveTrajectoryData)
            % considering the range to calculate tlc
            RangeTLC=150;
            LastDropNumTLC=15;% drop last 20 trajectory data to calculate TLC
            [TLC_Left,TLC_Right]=TCL_Calculate(CurveLaneData,CurveTrajectoryData,obj.VehicleWidth,obj.VehicleBaseLength,RangeTLC,LastDropNumTLC);
        
        
        end
    end
    %===========get and set=================
    methods
        function VehicleWidth=get.VehicleWidth(obj)
            modelhandle=Simulink.findBlocks(obj.ModelHandle,'Name','Constant_VehicleWidth');
            VehicleWidth=get_param(modelhandle,'Value');  
        end
        function VehicleBaseLength=get.VehicleBaseLength(obj)
            modelhandle=Simulink.findBlocks(obj.ModelHandle,'Name','Constant_VehicleBaseLength');
            VehicleBaseLength=evalin('base',get_param(modelhandle,'Value'));  
        end
        function RoadWidth=get.RoadWidth(obj)
            modelhandle=Simulink.findBlocks(obj.ModelHandle,'Name','Constant_RoadWidth');
            RoadWidth=get_param(modelhandle,'Value');  
        end        
 
        function trajectorydatatable=get.TrajectoryDataTable(obj)
            if ~isempty(obj.TrajectoryData)
                trajectorydatatable=array2table(obj.TrajectoryData,'VariableNames',obj.TrajectoryVariableNames);
            else
                trajectorydatatable=table;
            end
        end
        function curvelanedatatable=get.CurveLaneDataTable(obj)
            if ~isempty(obj.CurveLaneData)
                curvelanedatatable=array2table(obj.CurveLaneData,'VariableNames',obj.CurveLaneVariableNames);
            else
                curvelanedatatable=table;
            end
        end
        function UpdateParam(obj,PlanningParam)
            PlanningParam_col=reshape(PlanningParam,[],1);
%             set(obj.ParamSetHandle,'Value',num2str(PlanningParam_col));
            assignin('base','InitialPlanningParameter',PlanningParam_col);
            %note: the name must be consistent with the OAS_Simulink Model method
            % UpdateParam!!
        end
        function Update_EvaluatedPlanningParam(obj,ParamIndID)
            CurrentParam=evalin('base','InitialPlanningParameter'); 
                % note: the name-InitialPlanningParameter must be consistent with the OAS_Simulink Model method
            % UpdateParam!!
            obj.EvaluatedParamBuffer=[obj.EvaluatedParamBuffer;CurrentParam'];
            if nargin==1 
                % for the intial, the paramIndID may not in the Param_Ind ID
                obj.EValuatedParam_IndID=[obj.EValuatedParam_IndID,0];  
            else
                obj.EValuatedParam_IndID=[obj.EValuatedParam_IndID,ParamIndID]; 
            end
        end
    end
    
    
end

%============= Local Functins==========
function [TLC_Left,TLC_Right]=TCL_Calculate(CurveLaneData,CurveTrajectoryData,Width_Vehicle,Length_Vehicle,Range,LastDropNum)
% This is a function used to calculate the TLCs of the Trajectory.
StateVar={'X','Y','Yaw'};
% Global is relative to the beginning position
VehicleState_LocalOrigin=CurveTrajectoryData{1,StateVar};
LeftLane_LocalBegin=[CurveLaneData.Left_X,CurveLaneData.Left_Y];
RightLane_LocalBegin=[CurveLaneData.Right_X,CurveLaneData.Right_Y];
LeftLane_Global=Transfer2Global(LeftLane_LocalBegin,VehicleState_LocalOrigin);
RightLane_Global=Transfer2Global(RightLane_LocalBegin,VehicleState_LocalOrigin);
Order_polyfit=6;
Rotationangle=deg2rad(45);
Rotationmatrix=[cos(Rotationangle),sin(Rotationangle);-sin(Rotationangle),cos(Rotationangle)];

WheelOffset=0.5*[Length_Vehicle,Width_Vehicle];
options = struct('GradObj','off','Display','off','LargeScale','off','HessUpdate','bfgs','InitialHessType','identity','GoalsExactAchieve',1,'GradConstr',false,'TolFun',1e-2);
% Lane and trajectory
threshold_criteria=5e-2;
TLC_Left=nan(size(CurveTrajectoryData,1),1);
TLC_Right=nan(size(CurveTrajectoryData,1),1);
for id_vehpos=1:size(CurveTrajectoryData,1)-LastDropNum
    VehicleState=CurveTrajectoryData{id_vehpos,StateVar};
    % func_lane under vehicle coordinate
    %left lane
    LaneLeft_Local=Transfer2Local(LeftLane_Global,VehicleState);
    LeftLane_Valid=LaneLeft_Local(LaneLeft_Local(:,1)<=Range & LaneLeft_Local(:,1)>=0,:);
    LeftLane_ValidRot=LeftLane_Valid*Rotationmatrix;
    LaneLeftPolyfitRot=polyfit(LeftLane_ValidRot(:,1),LeftLane_ValidRot(:,2),Order_polyfit);
    Func_LaneLeft_LocalRot=@(t) [t,polyval(LaneLeftPolyfitRot,t)];
    
    % right lane
    LaneRight_Local=Transfer2Local(RightLane_Global,VehicleState);
    RightLane_Valid=LaneRight_Local(LaneRight_Local(:,1)<=Range & LaneRight_Local(:,1)>=0,:);
    RightLane_ValidRot=RightLane_Valid*Rotationmatrix;    
    LaneRightPolyfitRot=polyfit(RightLane_ValidRot(:,1),RightLane_ValidRot(:,2),Order_polyfit);
    Func_LaneRight_LocalRot=@(t) [t,polyval(LaneRightPolyfitRot,t)];

    % func_vehicle under vehicle coordinate
    Rtemp=sign(CurveTrajectoryData.AVz(id_vehpos)).*CurveTrajectoryData.Vx(id_vehpos)./(abs(CurveTrajectoryData.AVz(id_vehpos))+1e-6);% Positive means 
    Func_VehCog_Local = @(t) [abs(Rtemp)*sin(t),Rtemp*(1-cos(t))];
    roation_matrix=@(t)[cos(t),sin(t);-sin(t),cos(t)];% anti-clock rotation
    Func_LeftWheel_LocalRot=@(t) (Func_VehCog_Local(t)+WheelOffset*roation_matrix(t))*Rotationmatrix;
    Func_RightWheel_LocalRot=@(t) (Func_VehCog_Local(t)-WheelOffset*roation_matrix(t))*Rotationmatrix;
    
    % find the minimum value
    %t=[theta,X],theta is the curve angle, X is the X_coordinate of the
    %lane
    
    % Right
    Optwheel=@(t) (Func_RightWheel_LocalRot(t(1))-Func_LaneRight_LocalRot(t(2)))*(Func_RightWheel_LocalRot(t(1))-Func_LaneRight_LocalRot(t(2)))';
    [solution, fvalue] = fminlbfgs(Optwheel,[0.03,20],options);
%      [solution,fvalue]=InterSection_LC(Func_RightWheel_LocalRot,Func_LaneRight_LocalRot,abs(Rtemp),Range);
    solution_local=Func_RightWheel_LocalRot(solution(1))*Rotationmatrix';% coordination
    if solution_local(1)>=0 && solution_local(1)<=Range && fvalue<threshold_criteria
         % find a valid solution
        TLC_Right(id_vehpos)=-abs(Rtemp)*solution(1)/CurveTrajectoryData.Vx(id_vehpos);
    end

    % left
    Optwheel=@(t) (Func_LeftWheel_LocalRot(t(1))-Func_LaneLeft_LocalRot(t(2)))*(Func_LeftWheel_LocalRot(t(1))-Func_LaneLeft_LocalRot(t(2)))';
    [solution, fvalue] = fminlbfgs(Optwheel,[0.03,20],options);
%     [solution,fvalue]=InterSection_LC(Func_LeftWheel_LocalRot,Func_LaneLeft_LocalRot,abs(Rtemp),Range);
    solution_local=Func_LeftWheel_LocalRot(solution(1))*Rotationmatrix';% coordination
    if solution_local(1)>=0 && solution_local(1)<=Range && fvalue<threshold_criteria
        TLC_Left(id_vehpos)=abs(Rtemp)*solution(1)/CurveTrajectoryData.Vx(id_vehpos);
    end
end

end
