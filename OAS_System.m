classdef OAS_System < handle
    %The OAS_System is to build the DEM and DPM, also update the DEM
    %according to the drivers' feedback and select the next query to the
    %simmodel.
     
    properties
        ID='OAS_System' %used for register in the oas_handlemanager
        % Driver Evaluation Model and Intialization
        DEMPDF_Type
        DEMPDF
        DPMPDF_Safety
        DPMPDF_Comfort
        DPMPDF_Efficiency
        DPMPDF_Safety_Type
        DPMPDF_Comfort_Type
        DPMPDF_Efficiency_Type        
        
        % Variables for safety,comfort and efficiency
        Available_SafetyVar={'LeftLateralOffset','RightLateralOffset','MinTLC_Left','MaxITLC_Left','MinTLC_Right','MaxITLC_Right'}
        Available_ComfortVar={'MaxLateralAcceleration','MaxLongitudinalAcceleration','MaxYawRate','WARMS','RideComfort_Level'}
        Available_EfficiencyVar={'MinimumSpeed','Time'}
        Safety_PerceptionVar
        Comfort_PerceptionVar
        Efficiency_PerceptionVar
        
        % Initialization 
        % The Lookup Table that contains the planning parameter and the
        % original trajectory indicators database buffer
        Param_IndBaseFile='E:\Ranwei\tarjectory_tracking\Prescan_scene\Prescan_Carsim\Prescan_Carsim_1110\Parameter_IndicatorAll.csv'  % the filename that contains all indicators
        NormalisedParam_IndDataBaseTable=[]% Num_Data*Num_Indicators
        VarInParam_IndDataBase
        IndicatorRange=struct('MaxLateralAcceleration',[0 3],'MaxLongitudinalAcceleration',[0 3],'RightLateralOffset',[0 0.9],'LeftLateralOffset',[0 0.9],'Time',[12 20])% uinis: m/s2,m,s
        JNDs=struct('MaxLateralAcceleration',0.15,'MaxLongitudinalAcceleration',0.15,'LeftLateralOffset',0.1,...
            'RightLateralOffset',0.1,'Time',2);% uinis: m/s2,m,s
        % Rational Coefficient
        Rational_Coefficient_Learning=10;% learning agent rational_coefficient learning rate

        % Evaluation History Buffer
        EvaluatedTrajectoryIndBuffer=[] % each rows is Trajectory Indicators for a trajectory
        EValuatedTrajectoryID=[]% the evaluatedTrajectoryID in the NormalisedIndicatorDataBase
        CurrentTrajectoryInd
        LastPreferenceTrajectoryInd=[]
        EvaluationResultBuffer % Comparing Trajectory Group and Evaluation Result
        
        OAS_SimulinkModelobj
        DriverObj 
    end
    properties (Dependent,Access=private)
        % The first column is the pdf, and the other column are the weight
        % for each combination of weight, the last four column is the
        % ID_PDFTable
        PDF_WeightTable 
    end
    properties (Dependent)
        Available_PerceptionVar
        Perception_Var
        Preference_Estimation
        PreferenceSafety_Estimation
        PreferenceComfort_Estimation
        PreferenceEfficiency_Estimation
        
    end
    
    events
        LastPreferenceChanged
        DEMUpdated
    end
    
    % ================Constructor=============================
    methods
        function obj = OAS_System(DriverObj)
            % Initialize the DEM and DPM and the AlternativeTable
            obj.DriverObj=DriverObj;         
            obj.OAS_SimulinkModelobj=OAS_SimulinkModel();
            
            % register this driver object in the oas_handlemanager
            OAS_handlemanager=OAS_HandleManager.getInstance();
            OAS_handlemanager.register(obj.ID,obj);
        end
    end
    
    %==================The main functions==================
    methods

        function Update_OAS(obj,Evaluation)
            % Evaluation is a structor with fields "Type" and "Result".
            % trajectory is a data array with each column is a timeseries
            % array.
            obj.Update_EvaluatedTrajectory(obj.CurrentTrajectoryInd);
            obj.OAS_SimulinkModelobj.Update_EvaluatedPlanningParam();
            % Update the PDFs according to the  evaluation
            switch Evaluation.Type
                case 'Compare'
                    if isempty(obj.LastPreferenceTrajectoryInd)
                        obj.LastPreferenceTrajectoryInd=obj.CurrentTrajectoryInd;
                        obj.notify('LastPreferenceChanged');
                        return
                    end
                    
                    obj.UpdataPDFs_CompareResult(obj.CurrentTrajectoryInd,Evaluation.Result);
                case 'Fuzzy Linguistic Instruction'
                    sprintf('Fuzzy Linguistic Instruction not available for now!');  
                otherwise
                    warning('No Such Evaluation Type');
            end
            obj.notify('DEMUpdated');
        end
        
        function NextParam_IndID=Select_NextTrajectory(obj,SelectCriteria)
            switch SelectCriteria
                case 'Maximum Expected Information'
                    [NextParam,NextParam_IndID]=obj.Select_ExpectedInformationVolume();
                case 'Random'
                    [NextParam,NextParam_IndID]=obj.Select_Random();
            end
            % change the simulink model planning parameter
            obj.OAS_SimulinkModelobj.UpdateParam(NextParam);
            obj.OAS_SimulinkModelobj.Update_EvaluatedPlanningParam(NextParam_IndID);
        end
        
    end
    
    % ====================Helpful Methods================
   methods
        %=============== Update DEMs based on Evaluation=================
        function UpdataPDFs_CompareResult(obj,CurrentTrajectoryInd,Compare_Result)
            CurrentNomalisedTrajectoryInd=obj.Normalize(CurrentTrajectoryInd,obj.Perception_Var);
            switch Compare_Result
                case 'No Ideal'
                    return
                case 'Almost the same'
                    sprintf('Not Available Yet')
                    return
                case 'First'
                    CompareResult=0;
                    
                    Query=(CurrentNomalisedTrajectoryInd-obj.Normalize(obj.LastPreferenceTrajectoryInd,obj.Perception_Var))';
                case 'Second'
                    CompareResult=1;
                    Query=(CurrentNomalisedTrajectoryInd-obj.Normalize(obj.LastPreferenceTrajectoryInd,obj.Perception_Var))';
                    obj.LastPreferenceTrajectoryInd=CurrentTrajectoryInd;
                    obj.notify('LastPreferenceChanged');
                otherwise
                    error('No such Compare Result!');
            end
           % Update the PDFs according to the Query and Compare Result
            PDF_WeightTableTemp=obj.PDF_WeightTable;
            ID_PDFTable=PDF_WeightTableTemp(:,end-3:end);
            LikeHood_CompareResult=LikeHood_Func(PDF_WeightTableTemp(:,2:end-4),Query,CompareResult,obj.Rational_Coefficient_Learning);
            Post_PDFWeight=PDF_WeightTableTemp(:,1).*LikeHood_CompareResult;
            Num_Query=size(Post_PDFWeight,2);
            
            % Update thethe PDF_DEM
            Num_WeightDEM_temp=size(obj.DEMPDF,1);
            PostPDF_DEMTemp=zeros(Num_WeightDEM_temp,Num_Query);
            for DEM_i=1:Num_WeightDEM_temp
                PostPDF_DEMTemp(DEM_i,:)=sum(Post_PDFWeight(ID_PDFTable(:,4)==DEM_i,:));
            end
            [PostPDF_DEMTemp,~]=PDF_NormFunc(PostPDF_DEMTemp,obj.DEMPDF(:,2:end));
            obj.DEMPDF(:,1)=PostPDF_DEMTemp;

            % update the PDF_DPM
            Num_WeightDPM=[size(obj.DPMPDF_Safety,1),size(obj.DPMPDF_Comfort,1),size(obj.DPMPDF_Efficiency,1)];

            % Update the PDFDPM_Safety
            PostPDF_DPMTemp=zeros(Num_WeightDPM(1),Num_Query);
            for NumWeight_i=1:Num_WeightDPM(1)
                PostPDF_DPMTemp(NumWeight_i,:)=sum(Post_PDFWeight(ID_PDFTable(:,1)==NumWeight_i,:));
            end
            [PostDPMPDFWeight_Temp,~]=PDF_NormFunc(PostPDF_DPMTemp,obj.DPMPDF_Safety(:,2:end));
            obj.DPMPDF_Safety(:,1)=PostDPMPDFWeight_Temp;
            % Update the PDFDPM_Comfort
            PostPDF_DPMTemp=zeros(Num_WeightDPM(2),Num_Query);
            for NumWeight_i=1:Num_WeightDPM(2)
                PostPDF_DPMTemp(NumWeight_i,:)=sum(Post_PDFWeight(ID_PDFTable(:,2)==NumWeight_i,:));
            end
            [PostDPMPDFWeight_Temp,~]=PDF_NormFunc(PostPDF_DPMTemp,obj.DPMPDF_Comfort(:,2:end));
            obj.DPMPDF_Comfort(:,1)=PostDPMPDFWeight_Temp;

            % Update the PDFDPM_Efficiency
            PostPDF_DPMTemp=zeros(Num_WeightDPM(3),Num_Query);
            for NumWeight_i=1:Num_WeightDPM(3)
                PostPDF_DPMTemp(NumWeight_i,:)=sum(Post_PDFWeight(ID_PDFTable(:,3)==NumWeight_i,:));
            end
            [PostDPMPDFWeight_Temp,~]=PDF_NormFunc(PostPDF_DPMTemp,obj.DPMPDF_Efficiency(:,2:end));
            obj.DPMPDF_Efficiency(:,1)=PostDPMPDFWeight_Temp;  
        end
        function Flag=GeneratePDFs(obj)
            if isempty(obj.DEMPDF_Type)
               errordlg('Specify DEM PDF Type First!');
               Flag=1;
               return
            else
                obj.DEMPDF=obj.GeneratePDF(3,obj.DEMPDF_Type);
            end
            
            if isempty(obj.DPMPDF_Safety_Type)
               errordlg('Specify DPM_Safety PDF Type First!');
               Flag=1;
               return
%             elseif isempty(obj.Safety_PerceptionVar)
%                errordlg('Specify Safety Perception Indicator First!');
%                Flag=1;
%                return    
            else
                obj.DPMPDF_Safety=obj.GeneratePDF(length(obj.Safety_PerceptionVar),obj.DPMPDF_Safety_Type);
            end
            
            if isempty(obj.DPMPDF_Comfort_Type)
               errordlg('Specify DPM_Comfort PDF Type First!');
               Flag=1;
               return
%             elseif isempty(obj.Comfort_PerceptionVar)
%                errordlg('Specify Comfort Perception Indicator First!');
%                Flag=1;
%                return    
            else
                obj.DPMPDF_Comfort=obj.GeneratePDF(length(obj.Comfort_PerceptionVar),obj.DPMPDF_Comfort_Type);               
            end
            
            if isempty(obj.DPMPDF_Efficiency_Type)
               errordlg('Specify DPM_Efficiency PDF Type First!');
               Flag=1;
               return
%             elseif isempty(obj.Efficiency_PerceptionVar)
%                errordlg('Specify Efficiency Perception Indicator First!');
%                Flag=1;
%                return    
            else
                obj.DPMPDF_Efficiency=obj.GeneratePDF(length(obj.Efficiency_PerceptionVar),obj.DPMPDF_Efficiency_Type);     
            end
            Flag=0;
        end
        
        %===============Select Next Trajectory===========
        function [NextParam,max_queryID]=Select_ExpectedInformationVolume(obj) 
            Available_ID=setdiff((1:size(obj.NormalisedParam_IndDataBaseTable,1)),unique(obj.OAS_SimulinkModelobj.EValuatedParam_IndID));
            LastPreference_NormalisedTrajectoryInd=obj.Normalize(obj.LastPreferenceTrajectoryInd,obj.Perception_Var);
            Alternative_Query=obj.NormalisedParam_IndDataBaseTable{Available_ID,obj.Perception_Var}-repmat(LastPreference_NormalisedTrajectoryInd',length(Available_ID),1);
            Num_Query=size(Alternative_Query,1);
            DEM_WeightSet=obj.DEMPDF(:,2:end);
            Num_WeightDEM=size(obj.DEMPDF,1);
            tempPDF_WeightTable=obj.PDF_WeightTable;
            ID_PDFTable=tempPDF_WeightTable(:,end-3:end);

            tempPDF_WeightTable(:,1)=obj.DPMPDF_Safety(ID_PDFTable(:,1),1).*obj.DPMPDF_Comfort(ID_PDFTable(:,2),1).*obj.DPMPDF_Efficiency(ID_PDFTable(:,3),1).*obj.DEMPDF(ID_PDFTable(:,4),1);
            LikeHood_1_WeightByQuery=LikeHood_Func(tempPDF_WeightTable(:,2:end-4),Alternative_Query,1,obj.Rational_Coefficient_Learning);
            Post_PDF=repmat(tempPDF_WeightTable(:,1),1,Num_Query).*LikeHood_1_WeightByQuery;

            % the expected information difference of DEM
            PostPDF_DEM=zeros(Num_WeightDEM,Num_Query);
            for DEM_i=1:Num_WeightDEM
                PostPDF_DEM(DEM_i,:)=sum(Post_PDF(ID_PDFTable(:,4)==DEM_i,:),1);
            end
            [Post_PDFNorm,sum_post]=PDF_NormFunc(PostPDF_DEM,DEM_WeightSet);
            Prob_Difference=abs(repmat(obj.DEMPDF(:,1),1,Num_Query)-Post_PDFNorm);
            [~,Sum_Prob_Difference]=PDF_NormFunc(Prob_Difference,DEM_WeightSet);
            Expected_Information=Sum_Prob_Difference.*sum_post;
            [~,max_queryID]=max(Expected_Information);
%             NextNormalisedTrajectory_Ind=obj.NormalisedParam_IndDataBaseTable{max_queryID,obj.Perception_Var};
            NextParam=obj.NormalisedParam_IndDataBaseTable{max_queryID,1:9}';
        end
        
        function [NextParam,NextID]=Select_Random(obj) 
            Available_ID=setdiff((1:size(obj.NormalisedParam_IndDataBaseTable,1)),unique(obj.EValuatedTrajectoryID));
            NextID=random(length(Available_ID));
            NextParam=[0.1:0.1:0.9]';
        end
        
        % ============== Perception Model================
        function Sub_Perception=Perception(obj,trajectoryInd)
            % the trajectoryInd is a data array that is correspondting to
            % the perception_var;
            safety=obj.Safety_Perception(trajectoryInd);
            comfort=obj.Comfort_Perception(trajectoryInd);
            efficiency=obj.Efficiency_Perception(trajectoryInd);
            Sub_Perception=[safety,comfort,efficiency]';
        end
        
        function Subjective_Safety=Safety_Perception(obj,TrajectoryInd)
            % the trajectoryInd is a data array that is correspondting to
            % the perception_var;
            Safety_Ind=TrajectoryInd(ismember(obj.Perception_Var,obj.Safety_PerceptionVar));
            Safety_NormalisedInd=obj.Normalize(Safety_Ind,obj.Safety_PerceptionVar);
            Subjective_Safety=obj.PreferenceSafety_Estimation'*Safety_NormalisedInd;           
        end
        
        function Subjective_Comfort=Comfort_Perception(obj,TrajectoryInd)  
            Comfort_Ind=TrajectoryInd(ismember(obj.Perception_Var,obj.Comfort_PerceptionVar));
            Comfort_NormalisedInd=obj.Normalize(Comfort_Ind,obj.Comfort_PerceptionVar);
            Subjective_Comfort=obj.PreferenceComfort_Estimation'*Comfort_NormalisedInd;           
        end
        
        function Subjective_Efficiency=Efficiency_Perception(obj,TrajectoryInd)
            Efficiency_Ind=TrajectoryInd(ismember(obj.Perception_Var,obj.Efficiency_PerceptionVar));
            Efficiency_NormalisedInd=obj.Normalize(Efficiency_Ind,obj.Efficiency_PerceptionVar);
            Subjective_Efficiency=obj.PreferenceEfficiency_Estimation'*Efficiency_NormalisedInd;           
        end
        
        % =============== Param_Indicator Data Base and Normalize================        
        function Flag=SelectParam_IndicatorDataBase(obj)
            % select the Indicator table file,which is a table that contains
            % all indicators that have obtained from all collected
            % trajectories(a data base file),may also includes parameters.
            [file,path]=uigetfile({'*.csv';'*.xls';'*.mat'},'File Selector');
            if isequal(file,0)
                msgbox('User selected cancel');
                Flag=0;
            else
                obj.Param_IndBaseFile=fullfile(path,file);
                Flag=1;
            end
        end
        
        function Initial_NormalisedIndicatorDataBase(obj)
            % Generate the NormalisedIndicatorDataBase based on the
            % Indicator table file,which is a table that contains
            % all indicators that have obtained from all collected
            % trajectories(a data base file),may also includes parameters.
            if ~isempty(obj.Param_IndBaseFile)
                IndicatorDataTable=readtable(obj.Param_IndBaseFile);
                obj.VarInParam_IndDataBase=IndicatorDataTable.Properties.VariableNames;
                [NormalisedIndicatorDataBase,IndicatorRangenew]=Generate_NormalisedParam_IndDataBase(IndicatorDataTable,obj.JNDs);
                obj.NormalisedParam_IndDataBaseTable=NormalisedIndicatorDataBase;
                obj.IndicatorRange=IndicatorRangenew;
                msgbox('Success! Param_Indicator DataBase Updated!');
            else
                errordlg('Failed! Specify the Param_Ind Data Base File First!')
            end
            
        end
           
        function NormalisedInd=Normalize(obj,VarTrajectoryIndicator,VarName_Ind)
            % Var_Name: a cell array like 1*Var_Num with indicator name that corresponding to the
            % TrajectoryIndicator
            Var_JNDs=zeros(length(VarName_Ind),1);
            Var_IndicatorRange=zeros(length(VarName_Ind),2);% var*2;
            for var_i=1:length(VarName_Ind)
                Var_JNDs(var_i)=obj.JNDs.(VarName_Ind{var_i});
                Var_IndicatorRange(var_i,:)=obj.IndicatorRange.(VarName_Ind{var_i});
                % consistent with the Generate_NormalisedParam_IndDataBase
                % function
                maxtemp=max(VarTrajectoryIndicator(:,var_i));
                if maxtemp>Var_IndicatorRange(var_i,2)
                    % update the range
                    Maxvalue=fix(abs(maxtemp)/Var_JNDs(var_i))*Var_JNDs(var_i);
                    obj.IndicatorRange.(VarName_Ind{var_i})=[Var_IndicatorRange(var_i,1),Maxvalue];
                end
                mintemp=min(VarTrajectoryIndicator(:,var_i));
                if mintemp<Var_IndicatorRange(var_i,1)
                    % update the range
                    minvalue=fix(abs(mintemp)/Var_JNDs(var_i))*Var_JNDs(var_i);
                    obj.IndicatorRange.(VarName_Ind{var_i})=[minvalue,Var_IndicatorRange(var_i,1)];
                end
                
            end
            % Linear
            NormalisedInd=DriverPerception_Linear(Var_IndicatorRange,VarTrajectoryIndicator,Var_JNDs);
            NormalisedInd=NormalisedInd';
        end
                  
       

        % ===============Update Internal Properties==============
        function Update_IndicatorDataBase(obj,trajectory)
            % update the indicator data base according to the new
            % experienced trajectories
            
        end
        
        function ModifyJND(obj,JND_Var,JNDValue)
            obj.JNDs.(JND_Var)=JNDValue;
            if ismember(JND_Var,obj.VarInParam_IndDataBase)
                obj.Initial_NormalisedIndicatorDataBase();
            end
            
        end
   end
   methods
        function Update_LearningRationalCoeficient(obj,NewValue)
            if NewValue>0
                obj.Rational_Coefficient_Learning=NewValue;
            else
                errordlg('Invalid Rational Coefficient!');
            end
        end
        
        function Update_EvaluatedTrajectory(obj,trajectoryind)
            obj.EvaluatedTrajectoryIndBuffer=[obj.EvaluatedTrajectoryIndBuffer;trajectoryind];
            % Update EvaluatedNormalisedTrajectory
%             NormalisedTrajectoryInd=obj.Normalize(trajectoryind,obj.Perception_Var);
%             Diff_TrajectoryInd=obj.NormalisedParam_IndDataBaseTable{:,obj.Perception_Var}-repmat(NormalisedTrajectoryInd',size(obj.NormalisedParam_IndDataBaseTable,1),1);
%             [~,Minimum_No]=min(sum(Diff_TrajectoryInd.^2,2));
%             obj.EValuatedTrajectoryID=[obj.EValuatedTrajectoryID,Minimum_No];          
        end
        
        function Update_CurrentTrajectory(obj)
            TrajectoryTable=obj.OAS_SimulinkModelobj.TrajectoryDataTable;
            perception_var=obj.Perception_Var;
            if ~isempty(perception_var)
                Indicators=obj.Indicator_Calculate(TrajectoryTable,perception_var);
                obj.CurrentTrajectoryInd=Indicators;
            end
        end
        
        function Update_DriverPerceptionVariables(obj,SafetyVar,ComfortVar,EfficiencyVar)
            obj.Safety_PerceptionVar=SafetyVar;
            obj.Comfort_PerceptionVar=ComfortVar;
            obj.Efficiency_PerceptionVar=EfficiencyVar;    
        end
        
        function Update_DEMType(obj,DEMType,DPM_Safety_Type,DPMComfort_Type,DPM_Efficiency_Type)
            obj.DEMPDF_Type=DEMType;
            obj.DPMPDF_Safety_Type=DPM_Safety_Type;
            obj.DPMPDF_Comfort_Type=DPMComfort_Type;
            obj.DPMPDF_Efficiency_Type=DPM_Efficiency_Type;           
        end
          
   end
    
    % ==============Static Methods ================
    methods (Static)
         function Indicators=Indicator_Calculate(TrajectoryTable,Indicator_Varname)
            % Trajectory is a data arrya,each column is a variable data
            % sequence of the vehicle state obtained from the simulink model.
            % Indicator_Varname: a cell array, each element is a indicator
            % name.
            [Available_Ind,Available_IndName]=Indicator_Cal(TrajectoryTable); 
            Indicators=Available_Ind(ismember(Available_IndName,Indicator_Varname));            
        end
        
        function PDF=GeneratePDF(Ndim,PDF_Type)
            % Ndim is the Variable Nums;
            % PDF: Num_Weight*(Ndim+1),the first column is the pdf values.
            switch PDF_Type
                case 'Normal'
                    PDF=PDF_Normal(Ndim);
                case 'Uniform'
                    PDF=PDF_Uniform(Ndim);
                otherwise
                        msgbox([PDF_Type,' Not Ready Yet!']);
                        PDF=[];
                        return
            end

        end
    end
    
     
     %============Set and Get function==========
    methods
        function PDF_WeightTable=get.PDF_WeightTable(obj)
                Num_WeightDEM=size(obj.DEMPDF,1);
                NumIndicator_DPM=[size(obj.DPMPDF_Safety,2)-1,size(obj.DPMPDF_Comfort,2)-1,size(obj.DPMPDF_Efficiency,2)-1];% zeros means only 1 indicator
                Num_WeightDPM=[size(obj.DPMPDF_Safety,1),size(obj.DPMPDF_Comfort,1),size(obj.DPMPDF_Efficiency,1)];
                PDF_WeightTable=zeros(prod(Num_WeightDPM)*Num_WeightDEM,sum(NumIndicator_DPM)+1);
                [X1,X2,X3,X4]=ndgrid((1:Num_WeightDPM(1))',(1:Num_WeightDPM(2))',(1:Num_WeightDPM(3))',(1: Num_WeightDEM)');
                PDF_WeightTable(:,1)=obj.DPMPDF_Safety(X1(:),1).*obj.DPMPDF_Comfort(X2(:),1).*obj.DPMPDF_Efficiency(X3(:),1).*obj.DEMPDF(X4(:),1);
                PDF_WeightTable(:,2:end)=[(obj.DPMPDF_Safety(X1(:),2:end).*obj.DEMPDF(X4(:),2)),(obj.DPMPDF_Comfort(X2(:),2:end).*obj.DEMPDF(X4(:),3)),...
                    (obj.DPMPDF_Efficiency(X3(:),2:end).*obj.DEMPDF(X4(:),4))];% The last three column is the weight of S-C-E
                ID_PDFTable=[X1(:),X2(:),X3(:),X4(:)];
                PDF_WeightTable=[PDF_WeightTable,ID_PDFTable];           
        end
              
        function preferencecoef=get.Preference_Estimation(obj)
            maxpdf_weight=obj.DEMPDF(obj.DEMPDF(:,1)==max(obj.DEMPDF(:,1)),2:end);
            if size(maxpdf_weight,1)>=2
                preferencecoef=mean(maxpdf_weight)';
            else
                preferencecoef=maxpdf_weight';
            end
        end
        
        function preferencecoef_safety=get.PreferenceSafety_Estimation(obj)
            maxpdf_weight=obj.DPMPDF_Safety(obj.DPMPDF_Safety(:,1)==max(obj.DPMPDF_Safety(:,1)),2:end);
            if size(maxpdf_weight,1)>=2
                preferencecoef_safety=mean(maxpdf_weight)';
            else
                preferencecoef_safety=maxpdf_weight';
            end
        end
        
        function preferencecoef_comfort=get.PreferenceComfort_Estimation(obj)
            maxpdf_weight=obj.DPMPDF_Comfort(obj.DPMPDF_Comfort(:,1)==max(obj.DPMPDF_Comfort(:,1)),2:end);
            if size(maxpdf_weight,1)>=2
                preferencecoef_comfort=mean(maxpdf_weight)';
            else
                preferencecoef_comfort=maxpdf_weight';
            end
        end
        
        function preferencecoef_efficiency=get.PreferenceEfficiency_Estimation(obj)
            maxpdf_weight=obj.DPMPDF_Efficiency(obj.DPMPDF_Efficiency(:,1)==max(obj.DPMPDF_Efficiency(:,1)),2:end);
            if size(maxpdf_weight,1)>=2
                preferencecoef_efficiency=mean(maxpdf_weight)';
            else
                preferencecoef_efficiency=maxpdf_weight';
            end
        end
        
        function Perception_Var=get.Perception_Var(obj)
            Perception_Var=[obj.Safety_PerceptionVar,obj.Comfort_PerceptionVar,obj.Efficiency_PerceptionVar];
        end
        
        function Available_PerceptionVar=get.Available_PerceptionVar(obj)
            Available_PerceptionVar=[obj.Available_SafetyVar,obj.Available_ComfortVar,obj.Available_EfficiencyVar];
        end
    end
end
    
    

% ===============Local functions=============
% Calculate trajectory indicators
function [trajectory_indicators,Available_IndicatorName]=Indicator_Cal(TrajectoryTable)
% Trajectory is a Struct with field Varname and data arrya like Length_Sequence*Var_Num(default=12)
% Trajectory_Indicators is a  column for each trajectory;
% note: each time updated the available_IndicatorName, Update the
% properties!
% Safety Var
Available_SafetyVar={'LeftLateralOffset','RightLateralOffset','MinTLC_Left','MaxITLC_Left','MinTLC_Right','MaxITLC_Right'};
LeftLateralOffset=prctile(TrajectoryTable.LateralOffset(TrajectoryTable.LateralOffset>0),98);
if isnan(LeftLateralOffset)
    LeftLateralOffset=max(TrajectoryTable.LateralOffset);
end
RightLateralOffset=prctile(TrajectoryTable.LateralOffset(TrajectoryTable.LateralOffset<0),2);% negative,so pcrtile should be 2
if isnan(RightLateralOffset)
    RightLateralOffset=min(TrajectoryTable.LateralOffset);
end
MinTLC_Left=prctile(TrajectoryTable.TLC_Left,2);
MaxITLC_Left=1./MinTLC_Left;
MinTLC_Right=abs(prctile(TrajectoryTable.TLC_Right,98)); %TLC_Right is negative
MaxITLC_Right=1./MinTLC_Right;
Safety_Ind=[LeftLateralOffset,RightLateralOffset,MinTLC_Left,MaxITLC_Left,MinTLC_Right,MaxITLC_Right];
%Comfort Var
Available_ComfortVar={'MaxLateralAcceleration','MaxLongitudinalAcceleration','MaxYawRate','WARMS','RideComfort_Level'};
MaxLateralAcceleration=prctile(abs(TrajectoryTable.Ay),98);
MaxLongitudinalAcceleration=prctile(abs(TrajectoryTable.Ax),99);
MaxYawRate=prctile(abs(TrajectoryTable.AVz),98);
[WARMS,RideComfort_Level]=Ridecomfort(TrajectoryTable.Ax,TrajectoryTable.Ay);
Comfort_Ind=[MaxLateralAcceleration,MaxLongitudinalAcceleration,MaxYawRate,WARMS,RideComfort_Level];
% Efficiency Var
Available_EfficiencyVar={'MinimumSpeed','Time'};
MinSpeed=prctile(TrajectoryTable.Vx,2);
Time=TrajectoryTable.t(end)-TrajectoryTable.t(1);
Efficiency_Ind=[MinSpeed,Time];

trajectory_indicators=[Safety_Ind,Comfort_Ind,Efficiency_Ind];
Available_IndicatorName=[Available_SafetyVar,Available_ComfortVar,Available_EfficiencyVar];

end

function [NormalisedParam_IndBaseTable,IndicatorRange]=Generate_NormalisedParam_IndDataBase(Param_Indtable,JNDs)
% Convert the Indicators into Normalized 0-1.
% Safety,Comfort,Efficiency is all in 0-1 
% NormalisedParam_IndBase:is a table containing all alternative Indicators
% and corresponding trajectory planning parameters
% IndicatorRange: a struct that containes the indicator maximum and minimum
% value of the 

IndicatorVars=Param_Indtable.Properties.VariableNames;
JND_Vars=fieldnames(JNDs);
id_mem=ismember(JND_Vars,IndicatorVars);
% var_missjnd=JND_Vars(not(id_mem));
% if ~isempty(var_missjnd)
%     msgbox([var_missjnd{:},' are not finded in the IndicatorTable!'])
% end

vars_common=JND_Vars(id_mem);    

if ~isempty(vars_common)
    GridIndicator=zeros(size(Param_Indtable,1),length(vars_common));
    IndicatorRange=struct;
    for Var_i=1:length(vars_common)
        var_temp=vars_common{Var_i};
        indicatortemp=Param_Indtable.(var_temp);
        JND_Temp=JNDs.(var_temp);
        Maxvalue=max(fix(abs(indicatortemp)/JND_Temp))*JND_Temp;
        Minvalue=min(fix(abs(indicatortemp)/JND_Temp))*JND_Temp;
        IndicatorRange.(var_temp)=[Minvalue,Maxvalue]; 
        
        % Assumption 1-Linear, it is a linear perception model
        GridIndicator(:,Var_i)=DriverPerception_Linear([Minvalue,Maxvalue],indicatortemp,JND_Temp);
        % Assumption 2-Logistic 
        % Assumption 3-Exponential  
    end
    [UniqueIndicators,ID_UniqueGrid,~]=unique(GridIndicator,'stable','row');
    NormalisedIndTable=array2table(UniqueIndicators,'VariableNames',vars_common);
    NormalisedParam_IndBaseTable=[Param_Indtable(ID_UniqueGrid,1:9),NormalisedIndTable];
else
    NormalisedParam_IndBaseTable=[];
    IndicatorRange=[];
end

end

function likehood= LikeHood_Func(WeightSet,Query,CompareResult,Rational_Coefficient_Learning)
% calculate each query's likolihood when the result was compare(0/1)
% WeightSet: should be column vecotrs Num_Weight*Num_Indicators,
% Query: should be a data array like Num_query*Num_Indicators
%  CompareREsult: 1, means prefer the second one,0 measn the first one.

% likehood is a data array like Num_Weight* Num_query, 
%     beta=5;% rationality coefficient
%     Query=reshape(Query,[],size(WeightSet,2));
    likehood=1./(1+exp(Rational_Coefficient_Learning*WeightSet*Query'));
    if CompareResult ==1
        return        
    elseif CompareResult==0
        likehood=1-likehood;
    else
        errordlg('False compare result!','modal')
    end
end

function [post_prob,sum_prob]=PDF_NormFunc(PDF,Weight_Set)
    % prob_distrib is a array of the post probability distribution. the row
    % number is the dicrete weight number,and the column number is the query/action.
    % each column is the post prob distribution of the query.
    % this function is to calculate the total prob in the considering
    % weight range to be 1
    % pdf_func: is a function handle.
   if size(PDF,1)==1
        post_prob=1;
        sum_prob=PDF;
    else
        uniqueweight=unique(Weight_Set(:,1));
        Interval_Weight=min(diff(uniqueweight));
        Ndim_ValidWeight=size(Weight_Set,2)-1;
        % considering the truth that the valid range of the independent variable
        if Ndim_ValidWeight==1
            sum_prob=sum(PDF(1:end-1))*Interval_Weight;
        else 
            pos_temp=sum(Weight_Set(:,1:Ndim_ValidWeight),2)<=1-Interval_Weight;
            sum_prob=sum(PDF(pos_temp,:))*Interval_Weight^Ndim_ValidWeight;
        end
        post_prob=PDF./repmat(sum_prob,size(PDF,1),1);
    end        
end

function normlisedGridIndicator=DriverPerception_Linear(IndicatorRange,Indicator,Var_JND)
    % Normalize the trajectory indicators within 0-1,according to the
    % indicator range by linear method.
    % IndicatorRange£ºNum_Var*[minValue,maxValue]
    % Indicator: SequenceLength*NumVar;
    % The Var_JND a double array Num_Var*1, each element is the JND of each
% indicator.
    GridIndicatorstemp=fix(Indicator./Var_JND');
    normlisedGridIndicator=(abs(GridIndicatorstemp)-IndicatorRange(:,1)'./Var_JND')./...
        (IndicatorRange(:,2)'./Var_JND'-IndicatorRange(:,1)'./Var_JND'+eps); % prevent that the maximum 
        % value equals to the minimum one to make the result nan.
    normlisedGridIndicator(normlisedGridIndicator<0)=0;
    normlisedGridIndicator(normlisedGridIndicator>1)=1;
end

function PDF=PDF_Normal(Ndim,GridIntervalWeight)
if Ndim==1
    PDF=[1,1];
    return
end
if nargin==1
    GridIntervalWeight=0.1;
end
Weight_Min=0;
Weight_Max=1;
WeightSet=Discrete_set(Weight_Min,Weight_Max,GridIntervalWeight,Ndim);
Ndim_ValidWeight=Ndim-1;% only Ndim-1 independent dims because the sum of the weight equals to 1.
mu_norm=1/Ndim*ones(1,Ndim_ValidWeight)';
sigma_norm=eye(Ndim_ValidWeight);
NormalFunc=@(Weight_Vector) sum(1/sqrt(det(sigma_norm)*power(2*pi,size(sigma_norm,1)))*exp(-1/2*(Weight_Vector-mu_norm)'/sigma_norm.*(Weight_Vector-mu_norm)'),2);
NormalPDF=NormalFunc(WeightSet(:,1:Ndim_ValidWeight)');
% Normalise the probability density function
[NormalizedPDF,~]=PDF_NormFunc(NormalPDF,WeightSet);% The result
PDF=[NormalizedPDF,WeightSet];

end

function PDF=PDF_Uniform(Ndim,GridIntervalWeight)
if Ndim==1
    PDF=[1,1];
    return
end

if nargin==1
    GridIntervalWeight=0.1;
end
Weight_Min=0;
Weight_Max=1;
WeightSet=Discrete_set(Weight_Min,Weight_Max,GridIntervalWeight,Ndim);
NormalPDF=ones(size(WeightSet,1),1);
% Normalise the probability density function
[NormalizedPDF,~]=PDF_NormFunc(NormalPDF,WeightSet);% The result
PDF=[NormalizedPDF,WeightSet];

end

function WeightSet=Discrete_set(xmin,xmax,interval,Ndim)
% the sum of the Ndim colum or row equals to 1;
    X=(xmin:interval:xmax)';
    if Ndim==2
        theta_set=X;     
    elseif Ndim==3        
        [theta_1,theta_2]=ndgrid(X);
        theta1=theta_1(:); % take care the order 
        theta2=theta_2(:);
        theta_set=[theta1,theta2];            
    elseif Ndim==4        
        [theta_1,theta_2,theta_3]=ndgrid(X);
        theta1=theta_1(:);
        theta2=theta_2(:);
        theta3=theta_3(:);
        theta_set=[theta1,theta2,theta3];
    elseif Ndim==5
        [theta_1,theta_2,theta_3,theta_4]=ndgrid(X);
        theta1=theta_1(:);
        theta2=theta_2(:);
        theta3=theta_3(:);
        theta4=theta_4(:);
        theta_set=[theta1,theta2,theta3,theta4];
    elseif Ndim==6
        [theta_1,theta_2,theta_3,theta_4,theta_5]=ndgrid(X);
        theta1=theta_1(:);
        theta2=theta_2(:);
        theta3=theta_3(:);
        theta4=theta_4(:);
        theta5=theta_5(:);
        theta_set=[theta1,theta2,theta3,theta4,theta5];
    else
        errordlg('Not supported dimension!\n')
    end
    theta_set_valid=theta_set(sum(theta_set,2)<=1+eps,:);
    WeightSet=[theta_set_valid,1-sum(theta_set_valid,2)]; 
end

function [warms,SubFeel_No]=Ridecomfort(Ax,Ay)
%ridecomfort- Weighted acceleration root mean square
%ridecomfort-https://wenku.baidu.com/view/1ccffb2ada38376bae1fae90.html?rec_flag=default
% subjective feeling No:
    Fs=100;%sample frequency 0.01Hz
    %Ax
    [px,fx]=pwelch(Ax,[],[],[],Fs);
    wx=zeros(1,length(fx));%Weight frequency function
    wx(fx>=0.5 & fx<2)=1;
    wx(fx>=2 & fx<80)=2./fx(fx>=2 & fx<80);
    %Ay
    [py,fy]=pwelch(Ay,[],[],[],Fs);
    wy=zeros(1,length(fy));%Weight frequency function
    wy(fy>=0.5 & fy<2)=1;
    wy(fy>=2 & fy<80)=2./fy(fy>=2 & fy<80);

    warms=sqrt(sum(wx.^2*px*(fx(2)-fx(1)))+sum(wy.^2*py*(fy(2)-fy(1))));
    %¿ÉÊÓ»¯
    %plot(f,px);
    if warms<0.315
%         subjective_feeling=cellstr("comfortable");
        SubFeel_No=1;
    elseif warms<0.63
%         subjective_feeling=cellstr("little_uncomfortable");
        SubFeel_No=2;
    elseif warms<1
%         subjective_feeling=cellstr("relative_uncomfortable");
        SubFeel_No=3;
    elseif warms<1.6
%         subjective_feeling=cellstr("uncomfortable");
        SubFeel_No=4;
    elseif warms<2.5
%         subjective_feeling=cellstr("very_uncomfortable");
        SubFeel_No=5;
    else 
%         subjective_feeling=cellstr("extremely_uncomfortable");
        SubFeel_No=6;
    end
end
