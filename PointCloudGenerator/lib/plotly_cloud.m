function plotly_cloud(filename, ptCloud )

if ischar(ptCloud)
    ptCloud = pcread(ptCloud);
end

% Convert pt cloud to CSV
[pathstr, name] = fileparts(filename);
save_csv(ptCloud,[pathstr,name,'.csv']);

% Generate page
context.ptCloud = [pathstr,name,'.csv'];
template = fileread('ptCloud.tpl');
tpl = LTemplate(template);
doc = tpl.render(context);

%Save
fileID = fopen(filename,'w');
fprintf(fileID, doc);
fclose(fileID);

end

function save_csv(ptCloud, filename)
fileID = fopen(filename,'w');
fprintf(fileID,'x,y,z\n');
for i=1:length(ptCloud.Location)
    p = ptCloud.Location(i,:);
    fprintf(fileID,strjoin(arrayfun(@(n) num2str(n),p,'UniformOutput',false),','));
    fprintf(fileID,'\n');
end
fclose(fileID);
end