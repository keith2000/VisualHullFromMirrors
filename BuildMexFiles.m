function BuildMexFiles

    folder = fileparts(mfilename('fullpath'));

    commands = string.empty();

    
    

    commands(end+1) = MexCommand(["MatrixToAxisAngleMex.cpp", "Wm3Math.cpp"], folder);

    commands(end+1) = MexCommand(["MarchDemoMex.cpp", "VisualHullFunction.cpp", "MatlabGreyImage.cpp", "HomogeneousCoord.cpp", "Matrix4x4.cpp",...
        "Matrix4x4_Matlab.cpp", "polygonizer.cpp"], folder);

    commands(end+1) = MexCommand(["PixelEtErrMex.cpp", "VcalCamera.cpp", "SilhouetteSetPankaj.cpp", "SilhouetteViewPankaj.cpp", "SilFunctions.cpp"...
        "Wm3ContConvexHull2.cpp",  "Wm3ContHullEdge2.cpp",    "Wm3Math.cpp", "Wm3Matrix3.cpp", "Wm3Matrix4.cpp", "Wm3Vector2.cpp" ],...
        folder);
    commands(end+1) = MexCommand("GetBoundaryMex.cpp", folder);
    commands(end+1) = MexCommand("SelectObjectMex.cpp", folder);
    


    for commandIndex = 1:numel(commands)
        command = commands(commandIndex);

        %try
        disp(command)
        eval(command)
        %catch mE

        %   disp(mE)
        %end

    end

end

function command = MexCommand(srcFiles, folder)

    command = "mex ";
    for srcFileIndex = 1:numel(srcFiles)
        srcFile = srcFiles(srcFileIndex);

        fullFilename = SanitizeSlashes( folder + "/" + srcFile );

        command = command + "'" + fullFilename + "' ";
    end

    command = command + " -outdir " +  "'" + folder + "'";

end


function fullFilename = SanitizeSlashes(fullFilename)
    fullFilename = strrep(fullFilename, "/", filesep);
    fullFilename = strrep(fullFilename, "\", filesep);
end