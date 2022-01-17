classdef GUIController < handle
    %This class is used control the gui object callbacks
    
    properties
        ID='GUIController'
        OAS_Systemobj
        OAS_GUIobj
    end
    
    events

    end
    %=====Constructor==========
    methods
        function obj = GUIController(OAS_GUIobj,OAS_Systemobj)
            % Constructor
            obj.OAS_GUIobj = OAS_GUIobj;
            obj.OAS_Systemobj=OAS_Systemobj;

            % register this driver object in the oas_handlemanager
            OAS_handlemanager=OAS_HandleManager.getInstance();
            OAS_handlemanager.register(obj.ID,obj);
        end
    end
    
    
    % the SelectModel Panel
    methods       
        function callback_SelectModelButton(obj,~,~)
            Flag=obj.OAS_Systemobj.OAS_SimulinkModelobj.select_Model();
            if Flag==1
                selectfilepanel=findobj(obj.OAS_GUIobj.SelectFilePanel);
                set(findobj(selectfilepanel,'Tag','SelectedModelEdit'),'String',obj.OAS_Systemobj.OAS_SimulinkModelobj.FilePath);
            end
        end
        function callback_OpenSystemButton(obj,~,~)
            obj.OAS_Systemobj.OAS_SimulinkModelobj.Open_Model();
        end
        function callback_CloseSystemButton(obj,~,~) 
            obj.OAS_Systemobj.OAS_SimulinkModelobj.Close_Model();
        end
        function callback_StartSystemButton(obj,~,~)
            % add listener that listers the event 'escape curve finished',
            % indicated by a block output.
            obj.OAS_Systemobj.OAS_SimulinkModelobj.Start_Model();
        end
        function callback_ContinueSystemButton(obj,~,~)
            obj.OAS_Systemobj.OAS_SimulinkModelobj.Continue_Model();
        end
        function callback_PauseSystemButton(obj,~,~)
            obj.OAS_Systemobj.OAS_SimulinkModelobj.Pause_Model();
        end 
        function callback_StopSystemButton(obj,~,~)
            %clear CurveListenerHandle
            obj.OAS_Systemobj.OAS_SimulinkModelobj.Stop_Model();
        end 
    end
    
    % The Initial OAS Panel
    methods
        function callback_JNDPopumenue(obj,~,~)
            obj.OAS_GUIobj.Update_JNDPanel();
        end
        function callback_JNDValueEdit(obj,src,~)
            jnd_panel=findobj(obj.OAS_GUIobj.InitialOASPanel.Children,'Tag','JNDPanel');
            jndpopupmenue=findobj(jnd_panel.Children,'Tag','JNDPopupmenu');
            JNDVar=jndpopupmenue.String{jndpopupmenue.Value};
            JNDValue=str2double(src.String);
            obj.OAS_Systemobj.ModifyJND(JNDVar,JNDValue);
        end
        function callback_SelectParam_IndicatorDataBase(obj,~,~)
            Flag=obj.OAS_Systemobj.SelectParam_IndicatorDataBase();
            if Flag==1
                parind_panel=findobj(obj.OAS_GUIobj.InitialOASPanel.Children,'Tag','Param_IndDataBasePanel');
                selectedtext=findobj(parind_panel.Children.Children,'Tag','SelectedDataBaseEdit');
                set(selectedtext,'String',obj.OAS_Systemobj.Param_IndBaseFile);
                updatebutton=findobj(parind_panel.Children.Children,'Tag','UpdateDataBaseButton');
                set(updatebutton,'Enable','on');
            end

        end        
        function callback_UpdateParam_IndicatorDataBase(obj,~,~)
            obj.OAS_Systemobj.Initial_NormalisedIndicatorDataBase();
            parind_panel=findobj(obj.OAS_GUIobj.InitialOASPanel.Children,'Tag','Param_IndDataBasePanel');
            updatebutton=findobj(parind_panel.Children.Children,'Tag','UpdateDataBaseButton');
            set(updatebutton,'Enable','off');
        end
        function callback_UpdateRationalCoefLearningEdit(obj,src,~)
            rationalcoef=str2double(src.String);
            obj.OAS_Systemobj.Update_LearningRationalCoeficient(rationalcoef);
        end
        function callback_DEMApplyToggleButton(obj,src,~)
            % the src block is the button
            if src.Value==src.Max 
                % Botton Down
                % Set the OAS_System DEM PDFS type
                [DEMPDF_Type,DPMPDF_Safety_Type,DPMPDF_Comfort_Type,DPMPDF_Efficiency_Type]=obj.OAS_GUIobj.Get_DEMType();
                obj.OAS_Systemobj.Update_DEMType(DEMPDF_Type,DPMPDF_Safety_Type,DPMPDF_Comfort_Type,DPMPDF_Efficiency_Type);
                
                if ~isempty(obj.OAS_Systemobj.Perception_Var)
                    obj.OAS_Systemobj.GeneratePDFs();
                end
            end
            obj.OAS_GUIobj.UpdateDEMTypePanelState();
        end        
    end 
    
    % Driver Informaion Panel
    methods
        
    end  
    
    % Driver Perception Model Panel
    methods 
        function callback_PerceptionApplyToggleButton(obj,src,~)
            % the src is the button.
            if src.Value==src.Max 
                % Botton Down
                % Check if the selected ind has been specified in the
                % indicator range and has been specified JNDs.
                Flag_1=obj.OAS_GUIobj.CheckParamInd_PerceptionVarSelected();
                Flag_2=obj.OAS_GUIobj.CheckJND_PerceptionVarSelected();
                [selectedsafetyind,selectedcomfortind,selectedefficiencyind]=obj.OAS_GUIobj.Get_SelectedPerceptionVar();
                if Flag_1 && Flag_2
                    obj.OAS_Systemobj.Update_DriverPerceptionVariables(selectedsafetyind,selectedcomfortind,selectedefficiencyind);
                    if ~isempty(obj.OAS_Systemobj.DPMPDF_Safety_Type)
                        obj.OAS_Systemobj.GeneratePDFs();
                    end
                end
            end
            obj.OAS_GUIobj.UpdateDriverPerceptionPanelState();     
        end       
    end
    
    % Driver Evaluation Panel
    methods 
        function callback_EvaluationTypeSelection(obj,~,~)
            evaluation_type=obj.OAS_GUIobj.GetEvaluationType();
            DEMpanel=obj.OAS_GUIobj.DriverEvaluationPanel;
            comparegroup=findobj(DEMpanel,'Tag','Compare');
            instructionobj=findobj(DEMpanel,'Tag','LinguisticInstruction');
            switch evaluation_type
                case 'Compare'
                    comparegroup.Visible='on';
                    instructionobj.Enable='off';
                case 'Fuzzy Linguistic Instruction'
                    instructionobj.Enable='on';
                    comparegroup.Visible='off';
                otherwise
                    errordlg('No such evaluatio type','modal')
            end
            
        end
        
        function callback_UpdateOASButton(obj,src,~)
            % call the oas system upodate_oas method
            Evaluation=obj.OAS_GUIobj.GetEvaluation();
            obj.OAS_Systemobj.Update_OAS(Evaluation);
            ed=OAS_EventData(Evaluation);
            obj.OAS_Systemobj.Driverobj.notify('DriverEvaluated');
            % src: the update oas button object
            src.Enable='off';
            % set the selectnext button on
            selectbutton=findobj(obj.OAS_GUIobj.SelectNextPanel,'Tag','SelectNextButton');
            selectbutton.Enable='on';
        end 
        
        function CallOASUpdate(obj,~,~)
            
        end
    end
    
    % select Next Panel
    methods
        function callback_SelectNext(obj,src,~)
            %src: is the Next Select button
            % Determine Select Criteria
            SelectCriteria=obj.OAS_GUIobj.GetSelectCriteria();
            obj.OAS_Systemobj.Select_NextTrajectory(SelectCriteria);
            src.Enable='off';
        end
    end

end

