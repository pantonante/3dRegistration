
classdef LNode < handle 
       
    properties
       creates_scope = false;
       children = cell(0); 
    end
    
    methods 
        function self = LNode(fragment)
            if ~exist('fragment', 'var')
                fragment = '';
            end
            
            self.process_fragment(fragment);
        end
        
        function process_fragment(self, fragment)
            error('Lobster:MethodNotImplemented', ...
                '<process_fragment> is not implemented on <LNode>');
        end
        
        function enter_scope(self)
            error('Lobster:MethodNotImplemented', ...
                '<enter_scope> is not implemented on <LNode>');
        end
        
        function exit_scope(self)
            error('Lobster:MethodNotImplemented', ...
                '<exit_scope> is not implemented on <LNode>');
        end
        
        function str = render(self, context)
            error('Lobster:MethodNotImplemented', ...
                '<render> is not implemented on <LNode>');
        end
        
        function str = render_children(self, context, children)
            if ~exist('children', 'var')
                children = self.children;
            end
            
            rendered_children = cellfun(@(x) x.render(context), ...
                children, 'Uniform', false);
            
            str = strjoin(rendered_children, '');
        end
            
        function add_child(self, child)
            if ~isa(child, 'LNode')
                error('Lobster:GenericError', ...
                    'Attempted to add a <%s> to the children of the root node', ...
                    class(child));
            end
            
           self.children{end+1} = child; 
        end
    end
    
    
end