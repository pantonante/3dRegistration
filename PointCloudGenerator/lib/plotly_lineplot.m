function plotly_lineplot(varargin)
% PLOTLY_LINEPLOT(filename, x, y, ...) creates an HTML file with the plot rendered with Plotly.js
% Parameters:
%   output_filename, x, y ['xlabel', xlabel, ['ylabel', ylabel, ['title',
%   title]]]


if nargin<3 || mod(nargin,2)==0
    error('Missing arguments')
end

filename = varargin{1};
context.x = varargin{2};
context.y = varargin{3};
context.xlabel = '';
context.ylabel = '';
context.title = '';
context.legend = '';

% parameters parsing
for i=4:nargin-1
    if strcmpi(varargin{i},'xlabel')
        context.xlabel = varargin{i+1};
    elseif strcmpi(varargin{i},'ylabel')
        context.ylabel = varargin{i+1};
    elseif strcmpi(varargin{i},'title')
        context.title = varargin{i+1};
    elseif strcmpi(varargin{i},'legend')
        context.legend = varargin{i+1};
    end
end
        

template = fileread('lineplot.tpl');
tpl = LTemplate(template);
doc = tpl.render(context);

fileID = fopen(filename,'w');
fprintf(fileID,doc);
fclose(fileID);

end

