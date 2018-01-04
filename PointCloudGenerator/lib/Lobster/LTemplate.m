
classdef LTemplate < handle
   
    properties
       root;
    end
    
    methods
        function self = LTemplate(template_string)
            if iscell(template_string)
                template_string = strjoin(template_string, '\n');
            end
            assert(ischar(template_string), ...
                'template_string must be a string or a cellarray of strings.');
            
            self.root = LCompiler(template_string).compile();
        end
        
        function str = render(self, context)
            if ~exist('context', 'var')
                context = struct();
            end
            
            str = self.root.render(context);
            str = strrep(str, '{#', '{');
            str = strrep(str, '#}', '}');
        end
    end
    
end