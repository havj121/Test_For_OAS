classdef OAS_GUI < handle
    %OAS_GUI  A GUI used to view the OAS system properties and the simulink
    %information, also obtain the driver's feedback.
    
    properties
        ID='OAS_GUI'
        FigureHandle
        SelectFilePanel
        DriverInformationPanel
        InitialOASPanel
        DriverPerceptionModelPanel
        TrajectoryPanel
        DriverEvaluationPanel
        SelectNextPanel
        % DEM Figure
        DEMFigureHandle

        OAS_SystemObj
        Controllerobj
    end
     
    methods
        function obj = OAS_GUI(OAS_SystemObj)
            %OAS_GUI Constructor
            % register this driver object in the oas_handlemanager
            OAS_handlemanager=OAS_HandleManager.getInstance();
            OAS_handlemanager.register(obj.ID,obj);
            obj.build_UI();
            obj.OAS_SystemObj=OAS_SystemObj;  
            obj.attachToOASSystem();
            obj.Controllerobj=GUIController(obj,obj.OAS_SystemObj);
            obj.attachToController(obj.Controllerobj);

            % add listerner for the oas_SimulinkMOdelobj
            obj.OAS_SystemObj.addlistener('TrajectoryChanged',@obj.UpdataCurrentTrajectory);
            obj.OAS_SystemObj.addlistener('LastPreferenceChanged',@obj.UpdateLastTrajectory);
            % This one should be puted in the DEM & DPM view gui
            % obj.OAS_SystemObj.addlistener('DEMUpdated',@obj.UpdateDEMView);
        end
    end
    methods
        function build_UI(obj)
            % Build the GUI
            obj.FigureHandle=figure('Name','OAS_GUI','Position',[1300,100,1200,700]);
            % through the userdata to send the OAS_GUIobj to the simulink
            % model callback.
            %obj.FigureHandle.UserData=obj;
            % Construct the Layout===========
            mainlayout=uiextras.HBoxFlex('Parent',obj.FigureHandle,'Padding',3,'Spacing',5,'Tag','MainLayout');
            firstmaincollayout=uiextras.VBox('Parent',mainlayout,'Spacing',5,'Tag','Main_FirstColumn');
            secondmaincollayout=uiextras.VBoxFlex('Parent',mainlayout,'Spacing',5,'Tag','Main_SecondColumn');
            % The SelectFile Panel
            obj.SelectFilePanel=BuildSelectFilePanel(firstmaincollayout);
            % The Driver Information Panel
            obj.DriverInformationPanel=BuildDriverInformationPanel(firstmaincollayout);
            % The InitialOAS Panel
            obj.InitialOASPanel=BuildInitialOASPanel(firstmaincollayout);
            set(firstmaincollayout,'Sizes',[-3,-2,-5]);
            % The Driver Perception ModelPanel  
            obj.DriverPerceptionModelPanel=BuildDriverPerceptionModelPanel(secondmaincollayout);
            sub2ndmain_layout=uiextras.HBox('Parent',secondmaincollayout,'Padding',2,'Spacing',3);
            % The Driver Evaluation Panel
            obj.DriverEvaluationPanel=BuildDriverEvaluationPanel(sub2ndmain_layout);
            % The Select Next Panel
            obj.SelectNextPanel=BuildSelectNextPanel(sub2ndmain_layout);
            set(sub2ndmain_layout,'Sizes',[-7,-3]);
            % The trajectory Panel
            obj.TrajectoryPanel=BuildTrajectoryPanel(secondmaincollayout); 
            set(secondmaincollayout,'Sizes',[-1,-1,-1]); 
            
            set(mainlayout,'Sizes',[-3 -7]);
        end      
        function attachToOASSystem(obj)
            % Intialize the GUI with the data in the OAS Systemobj
            % the JND panel
            obj.Update_JNDPanel();
            
            % The Param_IndDataBase Panel
            obj.Update_ParamIndPanel();
            
            % the Rational Coefficient Learning
            obj.Update_RationalCoefPanel();
            % The Initial DEM & DPM
            % DEM
            obj.Update_DEMPanel();
            % Perception variable selection
            obj.Intialize_PerceptionVarSelectPanel();
        end       
        function attachToController(obj,Controllerobj)
            % The SelectFile Panel
            panelobj=obj.SelectFilePanel;
            funcH=@Controllerobj.callback_SelectModelButton;
            set(findobj(panelobj,'String','Select'),'callback',funcH);
            funcH=@Controllerobj.callback_OpenSystemButton;
            set(findobj(panelobj,'String','Open'),'callback',funcH);     
            funcH=@Controllerobj.callback_CloseSystemButton;
            set(findobj(panelobj,'String','Close'),'callback',funcH); 
            funcH=@Controllerobj.callback_StartSystemButton;
            set(findobj(panelobj,'String','Start'),'callback',funcH);
            funcH=@Controllerobj.callback_ContinueSystemButton;
            set(findobj(panelobj,'String','Continue'),'callback',funcH);
            funcH=@Controllerobj.callback_PauseSystemButton;
            set(findobj(panelobj,'String','Pause'),'callback',funcH); 
            funcH=@Controllerobj.callback_StopSystemButton;
            set(findobj(panelobj,'String','Stop'),'callback',funcH);
            
            % The Driver Information Panel
            % Initialization OAS panel
            % JND
            jnd_panel=findobj(obj.InitialOASPanel.Children,'Tag','JNDPanel');
            jndpopupmenue=findobj(jnd_panel.Children,'Tag','JNDPopupmenu');
            funcH=@Controllerobj.callback_JNDPopumenue;
            set(jndpopupmenue,'callback',funcH);
            jndvalueedit=findobj(jnd_panel,'Tag','JNDValueEdit');
            funcH=@Controllerobj.callback_JNDValueEdit;
            set(jndvalueedit,'callback',funcH);
            % Select Param_Ind DataBase
            parind_panel=findobj(obj.InitialOASPanel.Children,'Tag','Param_IndDataBasePanel');
            selectbutton=findobj(parind_panel.Children.Children,'Tag','SelectDataBaseButton');
            funcH=@Controllerobj.callback_SelectParam_IndicatorDataBase;
            set(selectbutton,'callback',funcH);
            updatebutton=findobj(parind_panel.Children.Children,'Tag','UpdateDataBaseButton');
            funcH=@Controllerobj.callback_UpdateParam_IndicatorDataBase;
            set(updatebutton,'callback',funcH);
            
            rationalcoef_panel=findobj(obj.InitialOASPanel.Children,'Tag','RatinalCoefficientPanel');
            rationalcoefvalueedit=findobj(rationalcoef_panel.Children,'Tag','RatioanalCoefficientValueEdit');
            funcH=@Controllerobj.callback_UpdateRationalCoefLearningEdit;
            set(rationalcoefvalueedit,'callback',funcH);
            % The Initial DEM & DPM
            dem_panel=findobj(obj.InitialOASPanel.Children,'Tag','DEMPanel');
            dempopupmenue=findobj(dem_panel.Children,'Tag','DEMApplyToggleButton');
            funcH=@Controllerobj.callback_DEMApplyToggleButton;
            set(dempopupmenue,'callback',funcH);

            
            
            
            % The Driver Perception Model Panel
            panelobj=obj.DriverPerceptionModelPanel;
            funcH=@Controllerobj.callback_PerceptionApplyToggleButton;
            set(findobj(panelobj,'Style','togglebutton'),'callback',funcH);
            
            % Driver Evaluation Panel
            panelobj=obj.DriverEvaluationPanel;
            funcH=@Controllerobj.callback_EvaluationTypeSelection;
            set(findobj(panelobj,'Tag','EvaluationSelectPop'),'callback',funcH);
            funcH=@Controllerobj.callback_UpdateOASButton;
            set(findobj(panelobj,'Tag','EvaluationButton'),'callback',funcH);

            % Select Next Panel
            panelobj=obj.SelectNextPanel;
            funcH=@Controllerobj.callback_SelectNext;
            set(findobj(panelobj,'Tag','SelectNextButton'),'callback',funcH);
            
            
        end
        function Build_DEMView(obj)
            % build the DEM visuliazation figure
            obj.DEMFigureHandle=Build_DEMFigure();
            
            
        end
    end
    
    % ====== Update Panels===========
    methods 
        function UpdataCurrentTrajectory(obj,~,~)
            % call the oas sytem method to calculate the trajectory
            % indicators
            % source: the OAS_system obj;
            % eventdata:a object with properties: Data,varname
            Indicators=obj.OAS_SystemObj.CurrentTrajectoryInd;
            %a trajectory panel
            Indicator_row=reshape(Indicators,1,[]);
            trjaecindpanel=findobj(obj.TrajectoryPanel.Children,'Tag','TrajectoryIndicatorPanel');
            set(findobj(trjaecindpanel,'Tag','CurrentIndicatorValueText'),'String',num2str(Indicator_row,'%6.2f'));
            
            % drver perception panel
            dpmcurrentpanel=findobj(obj.DriverPerceptionModelPanel,'Tag','DriverPerceptionPanel');
            safetyperceptoin=obj.OAS_SystemObj.Safety_Perception(Indicators);
            set(findobj(dpmcurrentpanel,'Tag','CurrentSafetyValueText'),'String',num2str(safetyperceptoin,'%6.2f'));
            comfortperceptoin=obj.OAS_SystemObj.Comfort_Perception(Indicators);
            set(findobj(dpmcurrentpanel,'Tag','CurrentComfortValueText'),'String',num2str(comfortperceptoin,'%6.2f'));
            efficiencyperceptoin=obj.OAS_SystemObj.Efficiency_Perception(Indicators);
            set(findobj(dpmcurrentpanel,'Tag','CurrentEfficiencyValueText'),'String',num2str(efficiencyperceptoin,'%6.2f'));
            
            % driver evaluation panel
            % if the next is enable off, enable evaluation on
            selectnextbutton=findobj(obj.SelectNextPanel,'Tag','SelectNextButton');
            if strcmp(selectnextbutton.Enable,'off')
                evaluationbutton=findobj(obj.DriverEvaluationPanel,'Tag','EvaluationButton');
                evaluationbutton.Enable='on';
            end

        end
        function UpdateLastTrajectory(obj,~,~)
            Indicators=obj.OAS_SystemObj.LastPreferenceTrajectoryInd;
            Indicator_row=reshape(Indicators,1,[]);
            %trajectory panel
            trjaecindpanel=findobj(obj.TrajectoryPanel,'Tag','TrajectoryIndicatorPanel');
            set(findobj(trjaecindpanel,'Tag','LastIndicatorValueText'),'String',num2str(Indicator_row,'%6.2f'));
            
            % drver perception panel
            dpmlastpanel=findobj(obj.DriverPerceptionModelPanel,'Tag','DriverPerceptionPanel');
            safetyperceptoin=obj.OAS_SystemObj.Safety_Perception(Indicators);
            set(findobj(dpmlastpanel,'Tag','LastSafetyValueText'),'String',num2str(safetyperceptoin,'%6.2f'));
            comfortperceptoin=obj.OAS_SystemObj.Comfort_Perception(Indicators);
            set(findobj(dpmlastpanel,'Tag','LastComfortValueText'),'String',num2str(comfortperceptoin,'%6.2f'));
            efficiencyperceptoin=obj.OAS_SystemObj.Efficiency_Perception(Indicators);
            set(findobj(dpmlastpanel,'Tag','LastEfficiencyValueText'),'String',num2str(efficiencyperceptoin,'%6.2f'));
        end
        function Update_JNDPanel(obj)
            % Update the edit uicontrol according to the oassystemobj
            jnd_panel=findobj(obj.InitialOASPanel.Children,'Tag','JNDPanel');
            jndpopupmenue=findobj(jnd_panel.Children,'Tag','JNDPopupmenu');
            Available_Var=obj.OAS_SystemObj.Available_PerceptionVar;
            if ~isempty(Available_Var)
                jndpopupmenue.String=Available_Var;
            else
                jndpopupmenue.String=fieldnames(obj.OAS_SystemObj.JNDs);
            end
            jndvalueedit=findobj(jnd_panel,'Tag','JNDValueEdit');
            currentjnd=jndpopupmenue.String{jndpopupmenue.Value};
            if ismember(currentjnd,fieldnames(obj.OAS_SystemObj.JNDs))
                set(jndvalueedit,'String',num2str(obj.OAS_SystemObj.JNDs.(currentjnd)));
            else
                set(jndvalueedit,'String','');
            end
       
        end
        function Update_ParamIndPanel(obj)
            % Update the panel
            ParamInd_panel=findobj(obj.InitialOASPanel.Children,'Tag','Param_IndDataBasePanel');
            selectedbasetext=findobj(ParamInd_panel.Children,'Tag','SelectedDataBaseEdit');
            if ~isempty(obj.OAS_SystemObj.Param_IndBaseFile)
                set(selectedbasetext,'String',obj.OAS_SystemObj.Param_IndBaseFile);
                obj.OAS_SystemObj.Initial_NormalisedIndicatorDataBase();
            end    
        end
        function Update_RationalCoefPanel(obj)
            rationalcoef_panel=findobj(obj.InitialOASPanel.Children,'Tag','RatinalCoefficientPanel');
            rationalcoefvalueedit=findobj(rationalcoef_panel.Children,'Tag','RatioanalCoefficientValueEdit');
            rationalcoefvalueedit.String=obj.OAS_SystemObj.Rational_Coefficient_Learning;
        end
        function Update_DEMPanel(obj)
            dem_panel=findobj(obj.InitialOASPanel.Children,'Tag','DEMPanel');
            dempopupmenue=findobj(dem_panel.Children,'Tag','DEMPopupmenue');
            if isempty(obj.OAS_SystemObj.DEMPDF_Type)
                dempopupmenue.Value=find(ismember('Empty',dempopupmenue.String));
            else
                value=find(ismember(obj.OAS_SystemObj.DEMPDF_Type,dempopupmenue.String),1);
                if isempty(value)
                    dempopupmenue.String=[dempopupmenue.String,obj.OAS_SystemObj.DEMPDF_Type];
                    value=length(dempopupmenue.String);
                end
                dempopupmenue.Value=value;
            end
            % DPM_Safety
            dpmsafetypopupmenue=findobj(dem_panel.Children,'Tag','DPM_SafetyPopupmenue');
            if isempty(obj.OAS_SystemObj.DPMPDF_Safety_Type)
                dpmsafetypopupmenue.Value=find(ismember('Empty',dpmsafetypopupmenue.String));
            else
                value=find(ismember(obj.OAS_SystemObj.DEMPDF_Type,dpmsafetypopupmenue.String),1);
                if isempty(value)
                    dpmsafetypopupmenue.String=[dpmsafetypopupmenue.String,obj.OAS_SystemObj.DEMPDF_Type];
                    value=length(dpmsafetypopupmenue.String);
                end
                dpmsafetypopupmenue.Value=value;
            end
            %DPM_Comfort
            dpmcomfortpopupmenue=findobj(dem_panel.Children,'Tag','DPM_ComfortPopupmenue');
            if isempty(obj.OAS_SystemObj.DPMPDF_Safety_Type)
                dpmcomfortpopupmenue.Value=find(ismember('Empty',dpmcomfortpopupmenue.String));
            else
                value=find(ismember(obj.OAS_SystemObj.DEMPDF_Type,dpmcomfortpopupmenue.String),1);
                if isempty(value)
                    dpmcomfortpopupmenue.String=[dpmcomfortpopupmenue.String,obj.OAS_SystemObj.DEMPDF_Type];
                    value=length(dpmcomfortpopupmenue.String);
                end
                dpmcomfortpopupmenue.Value=value;
            end
            %DPM_Efficiency
            dpmefficiencypopupmenue=findobj(dem_panel.Children,'Tag','DPM_EfficiencyPopupmenue');
            if isempty(obj.OAS_SystemObj.DPMPDF_Safety_Type)
                dpmefficiencypopupmenue.Value=find(ismember('Empty',dpmefficiencypopupmenue.String));
            else
                value=find(ismember(obj.OAS_SystemObj.DEMPDF_Type,dpmefficiencypopupmenue.String),1);
                if isempty(value)
                    dpmefficiencypopupmenue.String=[dpmefficiencypopupmenue.String,obj.OAS_SystemObj.DEMPDF_Type];
                    value=length(dpmefficiencypopupmenue.String);
                end
                dpmefficiencypopupmenue.Value=value;
            end 
        end
        function UpdateDEMTypePanelState(obj)
            dem_panel=findobj(obj.InitialOASPanel.Children,'Tag','DEMPanel');
            dempopupmenue=findobj(dem_panel.Children,'Tag','DEMPopupmenue');
            dpmsafetypopupmenue=findobj(dem_panel.Children,'Tag','DPM_SafetyPopupmenue');
            dpmcomfortpopupmenue=findobj(dem_panel.Children,'Tag','DPM_ComfortPopupmenue');
            dpmefficiencypopupmenue=findobj(dem_panel.Children,'Tag','DPM_EfficiencyPopupmenue');
            applybuttonobj=findobj(dem_panel.Children,'Tag','DEMApplyToggleButton');
            if strcmp(dempopupmenue.Enable,'on')
                dempopupmenue.Enable='off';
                dpmsafetypopupmenue.Enable='off';
                dpmcomfortpopupmenue.Enable='off';
                dpmefficiencypopupmenue.Enable='off';
                applybuttonobj.BackgroundColor=ones(1,3)*0.94;
            else
                dempopupmenue.Enable='on';
                dpmsafetypopupmenue.Enable='on';
                dpmcomfortpopupmenue.Enable='on';
                dpmefficiencypopupmenue.Enable='on';
                applybuttonobj.BackgroundColor='y';
            end  
        end
        function Intialize_PerceptionVarSelectPanel(obj)
            dpmvarselect_panel=findobj(obj.DriverPerceptionModelPanel,'Tag','PerceptionIndicatorSelectionPanel');
            safetylistbox=findobj(dpmvarselect_panel,'Tag','SafetyVarSelectListbox');
            safetylistbox.String=obj.OAS_SystemObj.Available_SafetyVar;
            comfortlistbox=findobj(dpmvarselect_panel,'Tag','ComfortVarSelectListbox');
            comfortlistbox.String=obj.OAS_SystemObj.Available_ComfortVar;
            efficiencylistbox=findobj(dpmvarselect_panel,'Tag','EfficiencyVarSelectListbox');
            efficiencylistbox.String=obj.OAS_SystemObj.Available_EfficiencyVar;                      
        end
        function UpdateDriverPerceptionPanelState(obj)
            DPMpanel=obj.DriverPerceptionModelPanel;
            safelistbox=findobj(DPMpanel,'Tag','SafetyVarSelectListbox');
            comfortlistbox=findobj(DPMpanel,'Tag','ComfortVarSelectListbox');
            efficiencylistbox=findobj(DPMpanel,'Tag','EfficiencyVarSelectListbox');
            applybuttonobj=findobj(DPMpanel,'Style','togglebutton');
            if strcmp(safelistbox.Enable,'on')
                safelistbox.Enable='off';
                comfortlistbox.Enable='off';
                efficiencylistbox.Enable='off';
                applybuttonobj.BackgroundColor=ones(1,3)*0.94;
            else
                safelistbox.Enable='on';
                comfortlistbox.Enable='on';
                efficiencylistbox.Enable='on';
                applybuttonobj.BackgroundColor='y';
            end  
        end
        function Flag=CheckParamInd_PerceptionVarSelected(obj)
                % Check if the selected Var is in the Param_Ind DataBase
                Flag=0;% fail 
                [selectedsafetyind,selectedcomfortind,selectedefficiencyind]=obj.Get_SelectedPerceptionVar();
                % Var-in the DataBase
                AvailableParam_Ind=obj.OAS_SystemObj.VarInParam_IndDataBase;
                safety_common=ismember(selectedsafetyind,AvailableParam_Ind);
                if ~isempty(selectedsafetyind)
                    if ~all(safety_common)
                        var_missjnd=selectedsafetyind(~safety_common);
                        f=errordlg(['Failed!',var_missjnd{:},': are not in the Param_Ind DataBase!'],'modal');
                        return
                    end
                end
                if ~isempty(selectedcomfortind)
                    comfort_common=ismember(selectedcomfortind,AvailableParam_Ind);    
                    if ~all(comfort_common)
                        var_missjnd=selectedcomfortind(~comfort_common);
                        f=errordlg(['Failed!',var_missjnd{:},': are not in the Param_Ind DataBase!'],'modal');
                        return
                    end
                end
                if ~isempty(selectedefficiencyind)
                    efficiency_common=ismember(selectedefficiencyind,AvailableParam_Ind);
                    if ~all(efficiency_common)
                        var_missjnd=selectedefficiencyind(~efficiency_common);
                        f=errordlg(['Failed!',var_missjnd{:},': are not in the Param_Ind DataBase!'],'modal');
                        return
                    end
                end
                Flag=1;
        end
        function Flag=CheckJND_PerceptionVarSelected(obj)
               % Check if the selected ind has been specified JNDs.
               Flag=0;% fail 
               [selectedsafetyind,selectedcomfortind,selectedefficiencyind]=obj.Get_SelectedPerceptionVar();
               AvailableJNDsInd=fieldnames(obj.OAS_SystemObj.JNDs);
               if ~isempty(selectedsafetyind)
                   safety_common=ismember(selectedsafetyind,AvailableJNDsInd);
                    if ~all(safety_common)
                        var_missjnd=selectedsafetyind(~safety_common);
                        f=errordlg(['Failed!',var_missjnd{:},': Specify the JND first!'],'modal');
                        return 
                    else
                        for i=length(selectedsafetyind)
                            jnd=obj.OAS_SystemObj.JNDs.(selectedsafetyind{i});
                            if isempty(jnd)|| isnan(jnd) 
                                f=errordlg(['Failed!',selectedsafetyind{i},': Specify the JND first!'],'modal');
                                return
                            end
                             
                        end
                    end
               end
               if ~isempty(selectedcomfortind)
                    comfort_common=ismember(selectedcomfortind,AvailableJNDsInd);    
                    if ~all(comfort_common)
                        var_missjnd=selectedcomfortind(~comfort_common);
                        f=errordlg(['Failed!',var_missjnd{:},': Specify the JND first!'],'modal');
                        return
                    else
                        for i=length(selectedcomfortind)
                            jnd=obj.OAS_SystemObj.JNDs.(selectedcomfortind{i});
                            if isempty(jnd)|| isnan(jnd) 
                                f=errordlg(['Failed!',selectedcomfortind{i},': Specify the JND first!'],'modal');
                                return
                            end
                        end
                    end
               end
               if ~isempty(selectedefficiencyind)
                    efficiency_common=ismember(selectedefficiencyind,AvailableJNDsInd);
                    if ~all(efficiency_common)
                        var_missjnd=selectedefficiencyind(~efficiency_common);
                        f=errordlg(['Failed!',var_missjnd{:},': Specify the JND first!'],'modal');
                        return
                    else
                        for i=length(selectedefficiencyind)
                            jnd=obj.OAS_SystemObj.JNDs.(selectedefficiencyind{i});
                            if isempty(jnd)|| isnan(jnd) 
                                f=errordlg(['Failed!',selectedefficiencyind{i},': Specify the JND first!'],'modal');
                                return
                            end
                        end
                    end
               end
               Flag=1; 
        end
              
    end
    % ==============Get================
    methods
        function [selectedsafetyind,selectedcomfortind,selectedefficiencyind]=Get_SelectedPerceptionVar(obj)
             % query data from the GUI
            DPMpanel=obj.DriverPerceptionModelPanel;
            safelistbox=findobj(DPMpanel,'Tag','SafetyVarSelectListbox');
            comfortlistbox=findobj(DPMpanel,'Tag','ComfortVarSelectListbox');
            efficiencylistbox=findobj(DPMpanel,'Tag','EfficiencyVarSelectListbox');
            selectedsafetyind=safelistbox.String(safelistbox.Value)';
            selectedcomfortind=comfortlistbox.String(comfortlistbox.Value)';
            selectedefficiencyind=efficiencylistbox.String(efficiencylistbox.Value)';            
        end
        function [DEMPDF_Type,DPMPDF_Safety_Type,DPMPDF_Comfort_Type,DPMPDF_Efficiency_Type]=Get_DEMType(obj)
            dem_panel=findobj(obj.InitialOASPanel.Children,'Tag','DEMPanel');
            dempopupmenue=findobj(dem_panel.Children,'Tag','DEMPopupmenue');
            dpmsafetypopupmenue=findobj(dem_panel.Children,'Tag','DPM_SafetyPopupmenue');
            dpmcomfortpopupmenue=findobj(dem_panel.Children,'Tag','DPM_ComfortPopupmenue');
            dpmefficiencypopupmenue=findobj(dem_panel.Children,'Tag','DPM_EfficiencyPopupmenue');
            DEMPDF_Type=dempopupmenue.String{dempopupmenue.Value};
            DPMPDF_Safety_Type=dpmsafetypopupmenue.String{dpmsafetypopupmenue.Value};
            DPMPDF_Comfort_Type=dpmcomfortpopupmenue.String{dpmcomfortpopupmenue.Value};
            DPMPDF_Efficiency_Type=dpmefficiencypopupmenue.String{dpmefficiencypopupmenue.Value};
            
        end
        function Evaluation_Type=GetEvaluationType(obj)
            evalselpop=findobj(obj.DriverEvaluationPanel,'Tag','EvaluationSelectPop');
            Evaluation_Type=evalselpop.String{evalselpop.Value};            
        end
        function Evaluation=GetEvaluation(obj)
            % Obtain Evaluation,Evaluation is a structor with fields "Type" and "Result".
            DEMpanel=obj.DriverEvaluationPanel;
            Evaluation=struct;
            Evaluation.Type=obj.GetEvaluationType();
            switch Evaluation.Type
                case 'Compare'
                    comparegroup=findobj(DEMpanel,'Tag','Compare');
                    Evaluation.Result=comparegroup.SelectedObject.String;
                case 'Fuzzy Linguistic Instruction'
                    instructionobj=findobj(DEMpanel,'Tag','LinguisticInstruction');
                    Evaluation.Result=instructionobj.String(instructionobj.Value);
                otherwise
                    errordlg('No such evaluatio type','modal')
            end
            % Evaluation Trajectory (or compare group)
            
            
        end
        function SelectCriteria=GetSelectCriteria(obj)
            selectnextpop=findobj(obj.SelectNextPanel,'Tag','SelectPop');
            SelectCriteria=selectnextpop.String{selectnextpop.Value};
        end
    end
end

% Local functions
function obj=BuildSelectFilePanel(parentfigure)
    obj=uipanel('Parent',parentfigure,'Title','Select Model','Tag','SelectFilePanel');
    mainlayout=uiextras.VBoxFlex('Parent',obj,'Padding',2,'Spacing',4);
    firstlayout=uiextras.VBox('Parent',mainlayout,'Spacing',2);
    secondlayout=uiextras.HBox('Parent',mainlayout,'Spacing',2);
    thirdlayout=uiextras.HBox('Parent',mainlayout,'Spacing',2);
    uicontrol('Parent',firstlayout,'Style','text','String','SelectedModel:','Tag','SelectedModeltext');
    uicontrol('Parent',firstlayout,'Style','text','Tag','SelectedModelEdit','BackgroundColor','white');
    uicontrol('Parent',secondlayout,'Style','pushbutton','String','Select','Tag','SelectModelButton');
    uicontrol('Parent',secondlayout,'Style','pushbutton','String','Open','Tag','OpenSystemButton');
    uicontrol('Parent',secondlayout,'Style','pushbutton','String','Close','Tag','CloseSystemButton');
    uicontrol('Parent',thirdlayout,'Style','pushbutton','String','Start','Tag','StartSystemButton');
    uicontrol('Parent',thirdlayout,'Style','pushbutton','String','Continue','Tag','ContinueSystemButton');
    uicontrol('Parent',thirdlayout,'Style','pushbutton','String','Pause','Tag','PauseSystemButton');
    uicontrol('Parent',thirdlayout,'Style','pushbutton','String','Stop','Tag','StopSystemButton');
    set(firstlayout,'Sizes',[20,-1]);
    set(secondlayout,'Sizes',[-1 -1 -1]);
    set(thirdlayout,'Sizes',[-1 -1 -1 -1]);

end
function obj=BuildDriverInformationPanel(parentlayout)
%     obj=uipanel(Figure,'Title','Driver Information','Position',position,'Tag','DriverInformationPanel');
    obj=uipanel('Parent',parentlayout,'Title','Driver Information','Tag','DriverInformationPanel');
end
function obj=BuildInitialOASPanel(parentlayout)
%     obj=uipanel(Figure,'Title','Initialization OAS','Position',position,'Tag','InitializationOASPanel');
    obj=uipanel('Parent',parentlayout,'Title','Initialization OAS','Tag','InitializationOASPanel');    
    mainlayout=uiextras.VBox('Parent',obj,'Padding',2);
    
    JNDpanel=uipanel(mainlayout,'Title','JND','Tag','JNDPanel');
    jndlayout=uiextras.HBox('Parent',JNDpanel,'Padding',2);
    avalable_ind={'Indicator_1'};
    uicontrol(jndlayout,'Style','popupmenu','String',avalable_ind,'Tag','JNDPopupmenu');
    uicontrol(jndlayout,'Style','edit','Tag','JNDValueEdit');
    set(jndlayout,'Sizes',[-1,-1],'Spacing',5);
    
    Param_IndBasePanel=uipanel(mainlayout,'Title','Select Param_Indicator Data Base','Tag','Param_IndDataBasePanel');
    param_indlayout=uiextras.VBoxFlex('Parent',Param_IndBasePanel,'Padding',2);
    uicontrol(param_indlayout,'Style','text','String','Selected DataBase:','Tag','SelectedText');
    uicontrol(param_indlayout,'Style','text','Tag','SelectedDataBaseEdit','BackgroundColor','white');
    subparam_indlayout=uiextras.HBox('Parent',param_indlayout);
    uicontrol(subparam_indlayout,'Style','pushbutton','String','Select DataBase','Tag','SelectDataBaseButton');
    uicontrol(subparam_indlayout,'Style','pushbutton','String','Update DataBase','Tag','UpdateDataBaseButton','Enable','off');
    set(subparam_indlayout,'Sizes',[-1,-1],'Spacing',5);
    set(param_indlayout,'Sizes',[20,-1,-1],'Spacing',5);
    
    rationalcoefpanel=uipanel(mainlayout,'Title','Rational Coefficient','Tag','RatinalCoefficientPanel');
    rationalcoeflayout=uiextras.HBox('Parent',rationalcoefpanel,'Padding',2);
    uicontrol(rationalcoeflayout,'Style','text','String','Ratioanal Coefficient Learing:');
    uicontrol(rationalcoeflayout,'Style','edit','Tag','RatioanalCoefficientValueEdit');
    set(rationalcoeflayout,'Sizes',[-1,-1],'Spacing',5);

    DEMpanel=uipanel(mainlayout,'Title','DEM & DPM','Tag','DEMPanel');
    demlayout=uiextras.VBox('Parent',DEMpanel,'Padding',2);
    topdemlayout=uiextras.Grid('Parent',demlayout,'Padding',2);
    availablePDF={'Empty','Normal','Uniform'};
    % first column
    uicontrol(topdemlayout,'Style','text','String','DEM:');
    uicontrol(topdemlayout,'Style','text','String','DPM_Safety:');
    % second column
    uicontrol(topdemlayout,'Style','popupmenu','String',availablePDF,'Tag','DEMPopupmenue');
    uicontrol(topdemlayout,'Style','popupmenu','String',availablePDF,'Tag','DPM_SafetyPopupmenue');    
    % third column
    uicontrol(topdemlayout,'Style','text','String','DPM_Comfort:');
    uicontrol(topdemlayout,'Style','text','String','DPM_Efficiency:');
    % forth column
    uicontrol(topdemlayout,'Style','popupmenu','String',availablePDF,'Tag','DPM_ComfortPopupmenue');
    uicontrol(topdemlayout,'Style','popupmenu','String',availablePDF,'Tag','DPM_EfficiencyPopupmenue');
    set(topdemlayout,'RowSizes',[-1,-1],'ColumnSizes',[80,-1,80,-1],'Spacing',5);
    bottondemlayout=uiextras.HBox('Parent',demlayout,'Padding',2);
    uiextras.Empty('Parent',bottondemlayout);
    uicontrol(bottondemlayout,'Style','togglebutton','String','Apply','Tag','DEMApplyToggleButton','BackgroundColor','y');
    uiextras.Empty('Parent',bottondemlayout);
    set(bottondemlayout,'Sizes',[-1,100,-1]);
    set(demlayout,'Sizes',[-2,-1],'Spacing',2);
    
    set(mainlayout,'Sizes',[-2,-4,-2,-4],'Spacing',5);
end
function obj=BuildDriverPerceptionModelPanel(parentlayout)
%   obj=uipanel(Figure,'Title','Driver Perception Model','Position',position,'Tag','DriverPerceptionModelPanel');
    obj=uipanel('Parent',parentlayout,'Title','Driver Perception Model','Tag','DriverPerceptionModelPanel');
    mainlayout=uiextras.HBoxFlex('Parent',obj,'Padding',3,'Spacing',5);
    % Driver Perception Indicator Selection
    DriverPerceptionVarSelectionPanel=uipanel('Parent',mainlayout,'Title','Perception Indicator Selection','Tag','PerceptionIndicatorSelectionPanel');
    varselectlayout=uiextras.VBox('Parent',DriverPerceptionVarSelectionPanel,'Padding',2,'Spacing',2);
    
    varselectlayout_top=uiextras.HBox('Parent',varselectlayout,'Padding',2);
    uicontrol(varselectlayout_top,'Style','text','String','Safety');
    uicontrol(varselectlayout_top,'Style','text','String','Comfort');
    uicontrol(varselectlayout_top,'Style','text','String','Efficiency');
    set(varselectlayout_top,'Sizes',[-1,-1,-1]);
    
    varselectlayout_middle=uiextras.HBox('Parent',varselectlayout,'Padding',2);
    uicontrol(varselectlayout_middle,'Style','listbox','Tag','SafetyVarSelectListbox','Max',2);          
    uicontrol(varselectlayout_middle,'Style','listbox','Tag','ComfortVarSelectListbox','Max',2);          
    uicontrol(varselectlayout_middle,'Style','listbox','Tag','EfficiencyVarSelectListbox','Max',2);
    set(varselectlayout_middle,'Sizes',[-1,-1,-1]);
    
    varselectlayout_bottom=uiextras.HBox('Parent',varselectlayout,'Padding',2);
    uiextras.Empty('Parent',varselectlayout_bottom);
    uicontrol(varselectlayout_bottom,'Style','togglebutton','String','Apply','Tag','VarSelectionoggleButton','BackgroundColor','y');
    uiextras.Empty('Parent',varselectlayout_bottom);
    set(varselectlayout_top,'Sizes',[-1,200,-1]);
    set(varselectlayout,'Sizes',[-1,-4,-1]);

    % The Drive Perception Panel
    DriverPerceptionPanel=uipanel('Parent',mainlayout,'Title','Driver Perception','Position',[0.6,0.01,0.38,0.98],'Tag','DriverPerceptionPanel');
    driverperceptionlayout=uiextras.Grid('Parent',DriverPerceptionPanel,'Padding',2);
    % First column
    uiextras.Empty('Parent',driverperceptionlayout);
    uicontrol(driverperceptionlayout,'Style','text','String','Safety','Tag','SafetyPerceptionText');
    uicontrol(driverperceptionlayout,'Style','text','String','Comfort','Tag','ComfortPerceptionText');
    uicontrol(driverperceptionlayout,'Style','text','String','Efficiency','Tag','EfficiencyPerceptionText');
    % second column
    uicontrol(driverperceptionlayout,'Style','text','String','Last','Tag','LastText');   
    uicontrol(driverperceptionlayout,'Style','text','BackgroundColor','white','Tag','LastSafetyValueText');
    uicontrol(driverperceptionlayout,'Style','text','BackgroundColor','white','Tag','LastComfortValueText');
    uicontrol(driverperceptionlayout,'Style','text','BackgroundColor','white','Tag','LastEfficiencyValueText');
    % third column
    uicontrol(driverperceptionlayout,'Style','text','String','Current','Tag','CurrentText');   
    uicontrol(driverperceptionlayout,'Style','text','BackgroundColor','white','Tag','CurrentSafetyValueText');
    uicontrol(driverperceptionlayout,'Style','text','BackgroundColor','white','Tag','CurrentComfortValueText');
    uicontrol(driverperceptionlayout,'Style','text','BackgroundColor','white','Tag','CurrentEfficiencyValueText');
    set(driverperceptionlayout,'RowSizes',[-1,-1,-1,-1],'ColumnSizes',[-1,-1,-1],'Spacing',5);
    set(mainlayout,'Sizes',[-7,-5]);
end
function obj=BuildDriverEvaluationPanel(parentlayout)
%   obj=uipanel(Figure,'Title','Driver Evaluation','Position',position,'Tag','Driver EvaluationPanel');
    obj=uipanel('Parent',parentlayout,'Title','Driver Evaluation','Tag','Driver EvaluationPanel');
    mainlayout=uiextras.HBoxFlex('Parent',obj,'Padding',3,'Spacing',5);
    % Evaluation type selection
    selectlayout=uiextras.VBox('Parent',mainlayout,'Padding',3,'Spacing',5);
    uicontrol(selectlayout,'Style','text','String','Evaluation Type');
    uicontrol(selectlayout,'Style','popupmenu','String',{'Compare','Fuzzy Linguistic Instruction'},'Tag','EvaluationSelectPop');
    uicontrol(selectlayout,'Style','pushbutton','String','OK & Update OAS','BackgroundColor','g','Tag','EvaluationButton','Enable','off');
    uicontrol(selectlayout,'Style','pushbutton','String','View DEM & DPM','BackgroundColor','y','Tag','ViewDEMButton');
    set(selectlayout,'Sizes',[20,30,-1,-1]);
    % Compare Button Group
    DriverCompareGroup=uibuttongroup(mainlayout,'Title','Driver Compare','Tag','Compare');
    uicontrol(DriverCompareGroup,'Style','radiobutton','String','No Ideal','Tag','CompareNoIdealButton','Units','normalized','Position',[0.01,0.76,0.8,0.2]);
    uicontrol(DriverCompareGroup,'Style','radiobutton','String','Almost the same','Tag','CompareTheSameButton','Units','normalized','Position',[0.01,0.51,0.8,0.2]);
    uicontrol(DriverCompareGroup,'Style','radiobutton','String','First','Tag','CompareFirstButton','Units','normalized','Position',[0.01,0.26,0.8,0.2]);
    uicontrol(DriverCompareGroup,'Style','radiobutton','String','Second','Tag','CompareSecondButton','Units','normalized','Position',[0.01,0.01,0.8,0.2]);
    % Fuzzy Linguistic Instruction
    middlelayout=uiextras.VBox('Parent',mainlayout,'Padding',2,'Spacing',3);
    uicontrol(middlelayout,'Style','text','String','Linguistic Instruction','Tag','LinguisticInstructionText');
    AvailabelInstructions={'Faster','Slower','More Close to Left','More Close to Right','More close to Center','...'};
    uicontrol(middlelayout,'Style','popupmenu','String',AvailabelInstructions,'Tag','LinguisticInstruction','Enable','off');
    uiextras.Empty('Parent',middlelayout);
    uiextras.Empty('Parent',middlelayout);
    set(middlelayout,'Sizes',[20,-1,-1,-1]);
    set(mainlayout,'Sizes',[-1,-1,-1]);
end
function obj=BuildSelectNextPanel(parentlayout)
%     obj=uipanel(Figure,'Title','Select Next','Position',position,'Tag','SelectNextPanel');
    obj=uipanel('Parent',parentlayout,'Title','Select Next','Tag','SelectNextPanel');
    mainlayout=uiextras.VBox('Parent',obj,'Padding',2,'Spacing',3);    
    uicontrol(mainlayout,'Style','text','String','Select Criteria','Tag','SelectText');
    uicontrol(mainlayout,'Style','popupmenu','String',{'Maximum Expected Information','Random'},'Tag','SelectPop');
    uicontrol(mainlayout,'Style','pushbutton','String','Next','BackgroundColor','g','Tag','SelectNextButton','Enable','off');
    uiextras.Empty('Parent',mainlayout);
    set(mainlayout,'Sizes',[20,30,-1,-1]);
end
function obj=BuildTrajectoryPanel(parentlayout)
    % The Trajectory Panel
%     obj=uipanel(Figure,'Title','Trajectory','Position',position,'Tag','TrajectoryIndicatorPanel');            
    obj=uipanel('Parent',parentlayout,'Title','Trajectory','Tag','TrajectoryIndicatorPanel');            
    % Trajectory Indicator Panel
    mainlayout=uiextras.HBoxFlex('Parent',obj,'Padding',2,'Spacing',2);
    leftlayout=uiextras.VBox('Parent',mainlayout,'Padding',2,'Spacing',2);
    TrajectoryIndicatorPanel=uipanel(leftlayout,'Title','Trajectory Indicator','Tag','TrajectoryIndicatorPanel');    
    indicatorlayout=uiextras.GridFlex('Parent',TrajectoryIndicatorPanel,'Padding',2);
    uicontrol(indicatorlayout,'Style','text','String','Last','Tag','LastIndicatorText');
    uicontrol(indicatorlayout,'Style','text','String','Current','Tag','CurrentIndicatorText');
    uicontrol(indicatorlayout,'Style','text','Tag','LastIndicatorValueText','BackgroundColor','white');
    uicontrol(indicatorlayout,'Style','text','Tag','CurrentIndicatorValueText','BackgroundColor','white');
    set(indicatorlayout,'RowSizes',[-1,-1],'ColumnSizes',[60,-1],'Spacing',5);
    % View More
    leftbottonlayout=uiextras.HBox('Parent',leftlayout,'Padding',2,'Spacing',2);
    uiextras.Empty('Parent',leftbottonlayout);
    uicontrol(leftbottonlayout,'Style','pushbutton','String','View More','Tag','ViewMoreButton','BackgroundColor','y')
    uiextras.Empty('Parent',leftbottonlayout);
    set(leftbottonlayout,'Sizes',[-1,-1,-1]);
    
    set(leftlayout,'Sizes',[-2,-1]);
    uiextras.Empty('Parent',mainlayout);
    set(mainlayout,'Sizes',[-1,-1]);
end
function obj=Build_DEMFigure()
    % Build the GUI
    obj=figure('Name','OAS_DEM','Position',[1000,100,700,700]);
    
    
    
     
end
