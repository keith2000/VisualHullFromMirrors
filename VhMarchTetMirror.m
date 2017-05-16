function [pMesh, kMesh] = VhMarchTetMirror( viewVec, imCell, varargin )

doTet = 0;
%cellWid = 0.01 ;%not used
cellsAcross = 50;
draw = 0;
doCrop = 0;
numSilhouettesPerImage = 5;
centrePoint = NaN;

VararginModifyDefaults( varargin{:} );

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%centrePoint

vhPoints = ConvVh( viewVec );

boundingCubeWidth = 1.25 * max(max(vhPoints') - min(vhPoints'));

%boundingCubeWidth = 99999;

cellWid = boundingCubeWidth/cellsAcross;

for viewLoop = 1:length(viewVec)
    
    [minX, minY, maxX, maxY] = PointSetExtremesXY( viewVec(viewLoop).boundary );    
    minX = floor(minX); %must move integer number of pixels
    minY = floor(minY);
    maxX = ceil(maxX);
    maxY = ceil(maxY);
    
    
    if doCrop > 0
        
        imgCell{viewLoop} = imCell{ floor((viewLoop-1)/numSilhouettesPerImage) + 1}(minY:maxY, minX:maxX);
        
        viewVec(viewLoop).camera.u0 = viewVec(viewLoop).camera.u0 - minX + 1;
        viewVec(viewLoop).camera.v0 = viewVec(viewLoop).camera.v0 - minY + 1;
        
    else
        imgCell{viewLoop} = zeros( size(imCell{ floor((viewLoop-1)/5) + 1}) );        
        imgCell{viewLoop}(minY:maxY, minX:maxX) = SelectObjectMex( imCell{ floor((viewLoop-1)/numSilhouettesPerImage) + 1}(minY:maxY, minX:maxX), 1);   
    end
    
    
    internalsCell{viewLoop} = [viewVec(viewLoop).camera.efl, ...
            viewVec(viewLoop).camera.u0, ...
            viewVec(viewLoop).camera.v0];
    
    RbtCell{viewLoop} = viewVec(viewLoop).camera.pose;
    
    if draw > 1
        figure
        imshow(imgCell{viewLoop})
        
        if doCrop > 0
            ShowPoly( CoordAdd(viewVec(viewLoop).boundary, -[minX;minY]+1 ) , 'FaceColor', 'none', 'EdgeColor', 'r', 'LineWidth', 2)        
        else
            ShowPoly( viewVec(viewLoop).boundary, 'FaceColor', 'none', 'EdgeColor', 'r', 'LineWidth', 2)        
        end
    end
    
end


if isnan(centrePoint)
centrePoint = ViewVecCentrePoint( viewVec );
end

%centrePoint
cellsAcross = 32000; %no need to limit this so choose a large number
[pMesh, kMesh] = MarchDemoMex(imgCell, internalsCell, RbtCell, cellWid, cellsAcross, centrePoint, doTet );

if draw > 0     
    NewFigure3D('light', 1);
    view(0,0)
    patch('Faces', kMesh', 'Vertices', pMesh', 'FaceColor', 'r', 'EdgeColor', 'none')
    lighting phong
end