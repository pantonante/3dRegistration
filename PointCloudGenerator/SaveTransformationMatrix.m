function SaveTransformationMatrix( T, filename )

fileID = fopen(filename,'w');
fprintf(fileID, '%d %d %d %d\n',T');

end

