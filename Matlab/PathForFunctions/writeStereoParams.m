function writeStereoParams(name, stereoParamsH)

fileID = fopen([name,'_stero.yml'],'w+t');

fprintf(fileID, '%s\n', '%YAML:1.0');

writeYmlMatrix(fileID,stereoParamsH.RotationOfCamera2, 'mR')
writeYmlMatrix(fileID,stereoParamsH.TranslationOfCamera2'./1000, 'mT')
writeYmlMatrix(fileID,stereoParamsH.EssentialMatrix./1000, 'mE')
writeYmlMatrix(fileID,stereoParamsH.FundamentalMatrix./1000, 'mF')

fclose(fileID);

fileID = fopen([name,'_cam1.yml'],'w+t');

fprintf(fileID, '%s\n', '%YAML:1.0');

writeYmlMatrix(fileID,stereoParamsH.CameraParameters1.IntrinsicMatrix, 'Matrix')
writeYmlMatrix(fileID,[stereoParamsH.CameraParameters1.RadialDistortion, stereoParamsH.CameraParameters1.TangentialDistortion], 'DistCoeffs')

fclose(fileID);

fileID = fopen([name,'_cam2.yml'],'w+t');

fprintf(fileID, '%s\n', '%YAML:1.0');

writeYmlMatrix(fileID,stereoParamsH.CameraParameters2.IntrinsicMatrix, 'Matrix')
writeYmlMatrix(fileID,[stereoParamsH.CameraParameters2.RadialDistortion,stereoParamsH.CameraParameters2.TangentialDistortion], 'DistCoeffs')

fclose(fileID);


end