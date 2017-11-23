classdef DatasetDescriptor
    %DATASETDESCRIPTOR Contains all the information about the auto
    %generated descriptor
    %   Detailed explanation goes here
    
    % <-- Properties
    properties
        Variable
        Diameter
        Container
    end
    % --> end properties
    
    % <-- Methods
    methods
        % Constructor
        function dd = DatasetDescriptor(variable, dataset_diameter)
            if nargin == 2
                if ischar(variable)
                    dd.Variable = variable;
                    dd.Container = containers.Map;
                else
                    error('[DatasetDescriptor] Variable must be a string')
                end
                if isnumeric(dataset_diameter)
                    dd.Diameter = dataset_diameter;
                else
                    error('[DatasetDescriptor] Diameter must be a number')
                end
            else
                error('[DatasetDescriptor] Requires variable and dataset diameter')
            end
        end
         
        function dd = add_PointClouds(dd, value, P_filename, Q_filename, T_filename)
            if nargin<4 || ~isnumeric(value) || ...
                    ~ischar(P_filename) || isempty(P_filename) || ...
                    ~ischar(Q_filename) || isempty(Q_filename) || ...
                    ~ischar(T_filename) || isempty(T_filename) 
                error('[DatasetDescriptor::add_PointClouds] Input must be three non-empty filenames')
            else
                dataset = DatasetDescriptor.PointCloudDataset...
                    (P_filename, Q_filename, T_filename);
                if(~dd.Container.isKey(num2str(value)))
                    dd.Container(num2str(value)) = [];
                end
                dd.Container(num2str(value)) = cat(1, ...
                    dd.Container(num2str(value)), dataset);
            end
        end
        
       function to_JSON(dd, output_filename)
            fileID = fopen(output_filename, 'w');
            fprintf(fileID,'{\n');
            fprintf(fileID,'\t"dataset_variable": "%s",\n',dd.Variable);
            fprintf(fileID,'\t"ptCloud_diameter": "%f",\n',dd.Diameter);
            fprintf(fileID,'\t"dataset": [\n'); 
            
            keyset = keys(dd.Container);
            for k=1:length(keyset)
                fprintf(fileID,'\t\t{\n'); 
                key = keyset{k};
                fprintf(fileID,'\t\t\t"value": "%s",\n',key);
                P='';
                Q='';
                T='';
                valueSet=dd.Container(key);
                for v=1:length(valueSet)
                    value=valueSet(v);
                    if(v<length(valueSet))
                        sep=',';
                    else
                        sep='';
                    end
                    P=strcat(P,'"',value.PtCloud_P,'"',sep);
                    Q=strcat(Q,'"',value.PtCloud_Q,'"',sep);
                    T=strcat(T,'"',value.Transformation,'"',sep);
                end
                fprintf(fileID,'\t\t\t"P": [%s],\n',P);
                fprintf(fileID,'\t\t\t"Q": [%s],\n',Q);
                fprintf(fileID,'\t\t\t"T": [%s]\n',T);
                fprintf(fileID,'\t\t}');
                if(k<length(keyset))
                    fprintf(fileID,',\n');
                else
                    fprintf(fileID,'\n');
                end
            end
            fprintf(fileID,'\t]\n}\n');
            fclose(fileID);
       end
     end
    % --> end public methods   
    
    % <-- Static methods
    methods(Static)
        function s = PointCloudDataset(P_filename, Q_filename, T_filename)
            if nargin<3 || ...
                    ~ischar(P_filename) || isempty(P_filename) || ...
                    ~ischar(Q_filename) || isempty(Q_filename) || ...
                    ~ischar(T_filename) || isempty(T_filename) 
                error('[PointCloudDataset] Input must be three non-empty filenames')
            else
                s = struct('PtCloud_P',P_filename, ...
                'PtCloud_Q', Q_filename, ...
                'Transformation', T_filename);
            end
        end
    end
    % --> end static methods
end

