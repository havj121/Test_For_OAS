classdef OAS_HandleManager < handle
    %This is a class that manages all the object through a container map.

    properties
        HandleDictionary
    end
    
    methods (Access=private)
        function obj = OAS_HandleManager()
            % Constructor that only be called within a obj
            obj.HandleDictionary = containers.Map();
        end
    end
    methods (Static)
        function obj = getInstance()
            % only be instanced once
            persistent localobj
            if isempty(localobj) || ~isvalid(localobj)
                localobj=OAS_HandleManager();
            end
            obj=localobj;
        end
    end
    
    methods
        function register(obj,ID,Data)
            % add data to the container map
            expr=sprintf('obj.HandleDictionary(\''%s\'')=Data;',ID);
            eval(expr);
        end
        
        function Data=getData(obj,ID)
            % get the Data of the key ID
            if isKey(obj.HandleDictionary,ID)
                Data=obj.HandleDictionary(ID);
            else
                error('ID not exist in the HandleDictionary');
            end
        end
    end
end

