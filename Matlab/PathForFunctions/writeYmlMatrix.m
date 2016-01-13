function writeYmlMatrix( fileID, matrix, matrixName )
%UNTITLED2 Summary of this function goes here
fprintf(fileID,'%s\n',[matrixName ,': !!opencv-matrix']);
fprintf(fileID, '%s %d \n%s %d\n%s \n%s', '   rows:', size(matrix,1), '   cols:', size(matrix,2), '   dt: d', '   data: [');

for j=1:size(matrix,2)
    for i=1:size(matrix,1)
        fprintf(fileID,'%.16e', matrix(i,j));
        if j==size(matrix,2) && i==size(matrix,1)
            break;
        else
            fprintf(fileID, ', ');
        end
    end
end
fprintf(fileID, '%s\n', ']');

end

