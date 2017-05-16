%MirrorDemo4
close all
clear all

imgFilename = [];


switch 2 %change this number to select input image
    case 1
        imgFilename{end+1} = 'jug1.png';
        imgFilename{end+1} = 'jug2.png';
    case 2
        imgFilename{end+1} = 'horse1.png';
        imgFilename{end+1} = 'horse2.png';
    case 3
        imgFilename{end+1} = 'locust1.png';
        imgFilename{end+1} = 'locust2.png';
    otherwise
        error('invalid option')
end




for imgLoop = 1:length(imgFilename)
    imCell{imgLoop} = imread(imgFilename{imgLoop});
    imCell{imgLoop} = double(imCell{imgLoop})/255;
    %  EvalPrint('size(imCell{imgLoop})')
    polyBoundaryVec = ExtractBoundaries( imCell{imgLoop} );
    polyBoundaryVecCell{imgLoop} = polyBoundaryVec;
end

%mergedViewVec = StaticMirrorObjMovingCamFn( polyBoundaryVecCell, 'draw', 1, 'useMiddleOfSilhouettesAsInitPp', 0 );


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%draw = 1;
draw = 0;

mergedViewVec = StaticMirrorObjMovingCamFn( polyBoundaryVecCell, 'draw', draw );


[pMesh, kMesh] = VhMarchTetMirror( mergedViewVec, imCell, 'draw', 1, 'cellsAcross', 100 );


vrmlFilename = 'DoubleMirrorModel.wrl';
fid = fopen( vrmlFilename, 'w' );
fprintf(fid, '#VRML V2.0 utf8\n');
fprintf(fid, 'Background {skyColor [0 0 0]}\n');
cameraPosition = [0;0;5];
cameraTarget = ViewVecCentrePoint( mergedViewVec );
AddVrmlViewpoint( fid, cameraPosition, cameraTarget, 'description', 'visual hull', 'fieldOfView', 0.2 );
AddVrmlIndexedFaceSet( fid, pMesh, kMesh, 'objNameStr', 'DoubleMirror');
fclose(fid);


