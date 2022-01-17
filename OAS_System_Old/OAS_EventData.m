classdef OAS_EventData < event.EventData
% EXPERIENCERECEVIEDEVENTDATA

% Copyright 2018 The MathWorks, Inc.

    properties (SetAccess = immutable)
        Data
    end
    methods
        function this = OAS_EventData(data)
            this.Data = data;
        end
    end
end