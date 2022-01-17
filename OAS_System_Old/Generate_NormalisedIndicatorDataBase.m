% Local Function
function [NormalisedParam_IndBaseTable,IndicatorRange]=Generate_NormalisedIndicatorDataBase(Param_Indtable,JNDs)
% Convert the Indicators into Normalized 0-1.
% Safety,Comfort,Efficiency is all in 0-1 
% NormalisedParam_IndBase:is a table containing all alternative Indicators
% and corresponding trajectory planning parameters
% IndicatorRange: a struct that containes the indicator maximum and minimum
% value of the 

IndicatorVars=Param_Indtable.Properties.VariableNames;
JND_Vars=fieldnames(JNDs);
id_mem=ismember(JND_Vars,IndicatorVars);
var_missjnd=JND_Vars(not(id_mem));
if ~isempty(var_missjnd)
    sprintf([var_missjnd{:},' are not finded in the IndicatorTable!'])
end

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
        GridIndicatorstemp=fix(indicatortemp/JND_Temp);
        normlisedGridIndicators=(abs(GridIndicatorstemp)-Minvalue./JND_Temp)./...
            (max(GridIndicatorstemp+eps)-min(GridIndicatorstemp)); % prevent that the maximum 
        % value equals to the minimum one to make the result nan.
        % Assumption 2-Logistic 
        % Assumption 3-Exponential
        GridIndicator(:,Var_i)=normlisedGridIndicators;
        
    end
    [UniqueIndicators,ID_UniqueGrid,~]=unique(GridIndicator,'stable','row');
    NormalisedIndTable=array2table(UniqueIndicators,'VariableNames',vars_common);
    NormalisedParam_IndBaseTable=[Param_Indtable(ID_UniqueGrid,1:9),NormalisedIndTable];
else
    NormalisedParam_IndBaseTable=[];
    IndicatorRange=[];
end

end