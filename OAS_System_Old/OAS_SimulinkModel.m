classdef OAS_SimulinkModel < handle
    %This is a Simulink Model class which runs the trajectory planning and
    %tracking model.
    
    properties
        ID='SimulinkModel' % 
        FilePath
        ModelName
        ModelHandle
        %  
        % Trajectory Data Information
        % maybe it is ok to send a data table
        ParamEnterCurve % table
        CurveLaneDataTable
        TrajectoryDataTable
        ParamSetHandle
        % Trajectory Indicator and planning parameter lookup table,the No
        % of the planning parameter in the param_ind database.
        Param_TrajectoryDataBase
        OAS_SystemObj
    end
    properties (Dependent)
        RoadWidth
        VehicleWidth
        VehicleBaseLength
        NormalisedParamIndicatorBase
        IndicatorRangeTable
    end
    
    events
        EscapedCurve % escape the curve
        TrajectoryUpdated %used to update the database
        EnteredCurve % enter the curve
        ParameterUpdated
    end
    
    
    methods
        function obj = OAS_SimulinkModel(OAS_Systemobj)
            %Constructor
            if nargin==1
                obj.OAS_SystemObj=OAS_Systemobj;    
            end
            % add listener for the OAS_simulinkobj for EnteredCurve
            obj.addlistener('EnteredCurve',@obj.UpdateCurveLaneData);
            obj.addlistener('EscapedCurve',@obj.UpdateTrajectoryData);
            obj.addlistener('TrajectoryUpdated',@obj.UpdateParamTrajectoryDataBase);
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
    
    %============ Update Simulink Model Obj============
    methods
        function UpdateCurveLaneData(obj,~,ed_curvelanedata)
            % src: the oas_simulink model obj
            % ed_curvelanedata:curvelanedatatable
            obj.CurveLaneDataTable=ed_curvelanedata.Data;
            % record the param when enter the curve
            obj.ParamEnterCurve=obj.GetParam();
        end
        
        function UpdateTrajectoryData(obj,~,ed_trajectorydata)
            % src: the oas_simulink model obj
            % ed_curvelanedata:curvelanedatatable
            trajectorydata=ed_trajectorydata.Data;
            % if the trajectory data from the model has no TLCs, calculate
            % it.
            if ~all(ismember({'TLC_Left','TLC_Right'},trajectorydata.Properties.VariableNames))
            % Calculate the TLCs
                CurveLaneData=obj.CurveLaneDataTable;
                [TLC_Left,TLC_Right]=obj.CalculateTLC(CurveLaneData,trajectorydata);
                trajectorydata.TLC_Left=TLC_Left;
                trajectorydata.TLC_Right=TLC_Right;
            end
            obj.TrajectoryDataTable=trajectorydata;
            % data for update paramtrajectorydatabase
            S.TrajectoryData=trajectorydata;
            S.PlanningParam=obj.ParamEnterCurve;
            ed_param_trajectory=OAS_EventData(S);
            obj.notify('TrajectoryUpdated',ed_param_trajectory);
        end
        
        function UpdateParamTrajectoryDataBase(obj,~,ed_param_trajectory) 
            % update the param trajectory data base when the
            % trajectory updated.
            % ed_trajectoyrdata: the Data of it is a structure, with field
            % name TrajectoryData, PlanningParam
            
            % check if the planning param has been stored in the database.
            S=ed_param_trajectory.Data;
            PlanningParamtable=S.PlanningParam;
            TrajectoryData=S.TrajectoryData;
            if ~isStored(PlanningParamtable)
                tabletemp=table({PlanningParamtable},{obj.CurveLaneDataTable},{TrajectoryData},'VariableNames',{'PlanningParam'},{'CurveLaneData'},{'TrajectoryData'});
                % call the OAS_System method to calculate the indicators
                Indicators=OAS_DPM.Indicator_Calculate(TrajectoryData);
                tabletemp.Indicators={Indicators};
                obj.Param_TrajectoryDataBase=[obj.Param_TrajectoryDataBase;tabletemp];
            end
        end
            
        function SetNextParam(obj,src,~)
            % src: OAS_Systemobj
           PlanningParam=src.SelectedNextParam;
           obj.UpdateParam(PlanningParam);
        end
        
        function UpdateParam(obj,PlanningParam)
             PlanningParam_col=reshape(PlanningParam,[],1);
%             set(obj.ParamSetHandle,'Value',num2str(PlanningParam_col));
            % assigne value
            Beta1=Simulink.findBlocks(obj.ModelHandle,'Name','Beta_1');
            set_param(Beta1,'Value',num2str(PlanningParam_col(1)));
            Beta2=Simulink.findBlocks(obj.ModelHandle,'Name','Beta_2');
            set_param(Beta2,'Value',num2str(PlanningParam_col(2)));
            Beta3=Simulink.findBlocks(obj.ModelHandle,'Name','Beta_3');
            set_param(Beta3,'Value',num2str(PlanningParam_col(3)));
            Beta4=Simulink.findBlocks(obj.ModelHandle,'Name','Beta_4');
            set_param(Beta4,'Value',num2str(PlanningParam_col(4)));
            Max_LateralAccel=Simulink.findBlocks(obj.ModelHandle,'Name','Max_LateralAcceleration');
            set_param(Max_LateralAccel,'Value',num2str(PlanningParam_col(5)));
            Max_LongitudinalAccel=Simulink.findBlocks(obj.ModelHandle,'Name','Max_LongitudinalAcceleration');
            set_param(Max_LongitudinalAccel,'Value',num2str(PlanningParam_col(6)));
            Max_LongitudinalDecel=Simulink.findBlocks(obj.ModelHandle,'Name','Max_LongitudinalDeceleration');
            set_param(Max_LongitudinalDecel,'Value',num2str(PlanningParam_col(7)));
            Min_Speed=Simulink.findBlocks(obj.ModelHandle,'Name','Min_Speed');
            set_param(Min_Speed,'Value',num2str(PlanningParam_col(8)));
            Max_Spped=Simulink.findBlocks(obj.ModelHandle,'Name','Max_Speed');
            set_param(Max_Spped,'Value',num2str(PlanningParam_col(9)));
            obj.notify('ParameterUpdated')
        end
        
        function Update_EvaluatedPlanningParam(obj,ParamIndID)
            CurrentParam=obj.GetParam();
            CurrentParam=reshape(CurrentParam,[],1);
                % note: the name-InitialPlanningParameter must be consistent with the OAS_Simulink Model method
            % UpdateParam!!
            obj.EvaluatedParamBuffer=[obj.EvaluatedParamBuffer;CurrentParam'];
            if nargin==2 
                obj.EValuatedParam_IndID=[obj.EValuatedParam_IndID,ParamIndID]; 
            end
        end
        
        function PlanningParamTable=GetParam(obj)
            Beta1Block=Simulink.findBlocks(obj.ModelHandle,'Name','Beta_1');
            Beta1=str2double(get_param(Beta1Block,'Value'));
            Beta2Block=Simulink.findBlocks(obj.ModelHandle,'Name','Beta_2');
            Beta2=str2double(get_param(Beta2Block,'Value'));
            Beta3Block=Simulink.findBlocks(obj.ModelHandle,'Name','Beta_3');
            Beta3=str2double(get_param(Beta3Block,'Value'));
            Beta4Block=Simulink.findBlocks(obj.ModelHandle,'Name','Beta_4');
            Beta4=str2double(get_param(Beta4Block,'Value'));
            Max_LateralAccelBlock=Simulink.findBlocks(obj.ModelHandle,'Name','Max_LateralAcceleration');
            Max_LateralAccel=str2double(get_param(Max_LateralAccelBlock,'Value'));
            Max_LongitudinalAccelBlock=Simulink.findBlocks(obj.ModelHandle,'Name','Max_LongitudinalAcceleration');
            Max_LongitudinalAccel=str2double(get_param(Max_LongitudinalAccelBlock,'Value'));
            Max_LongitudinalDecelBlock=Simulink.findBlocks(obj.ModelHandle,'Name','Max_LongitudinalDeceleration');
            Max_LongitudinalDecel=str2double(get_param(Max_LongitudinalDecelBlock,'Value'));
            Min_SpeedBlock=Simulink.findBlocks(obj.ModelHandle,'Name','Min_Speed');
            Min_Speed=str2double(get_param(Min_SpeedBlock,'Value'));
            Max_SppedBlock=Simulink.findBlocks(obj.ModelHandle,'Name','Max_Speed');
            Max_Speed=str2double(get_param(Max_SppedBlock,'Value'));
%             PlanningParam=[Beta1;Beta2;Beta3;Beta4;Max_LateralAccel;Max_LongitudinalAccel;Max_LongitudinalDecel;Min_Speed;Max_Speed];
            ParameterNames={'Beta_PathLength','Beta_Curvature','Beta_Centering','Beta_HeadingError',...
                'LatAcceleration_UpperBound','LongidAcceleration_UpperBound','LongidDeceleration_UpperBound','Velocity_LowerBound','Velocity_UpperBound'};
            PlanningParamTable=table(Beta1,Beta2,Beta3,Beta4,Max_LateralAccel,Max_LongitudinalAccel,Max_LongitudinalDecel,Min_Speed,Max_Speed,'VariableNames',ParameterNames);
            
        end       
    end 

    %===========get and set=================
    methods
        function set.OAS_SystemObj(obj,OAS_Systemobj)
            obj.OAS_SystemObj=OAS_Systemobj;
            % add listerner for the oas_system 
            obj.OAS_SystemObj.addlistener('EvaluatedTrajectoryBufferUpdated',@obj.Update_EvaluatedPlanningParam);
            obj.OAS_SystemObj.addlistener('NextParamSelected',@obj.SetNextParam);
        end
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
