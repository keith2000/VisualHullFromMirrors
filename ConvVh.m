function vhPoints = ConvVh( viewVec, varargin )

draw = 0;
noiseStd = 0.00001;
VararginModifyDefaults( varargin{:} );

%polygons must be convex
%edges should not be exactly horixontal or vertical -- add a small bit of
%noise to avoid these degenerate configurations

numCams = length(viewVec);

pDualVec = [];


centrePoint = ViewVecCentrePoint( viewVec );

for camLoop = 1:numCams    
    ccPolyCellNi{camLoop} = ImageToNormalisedImage( convexhull( ...
        viewVec(camLoop).boundary + noiseStd*randn(size(viewVec(camLoop).boundary) ) ), ...
        viewVec(camLoop).camera );        
    
    Rbt = inv(viewVec(camLoop).camera.pose);
    Rbt(1:3,4) = Rbt(1:3,4) - centrePoint; %origin must lie inside the convex hull for dual space computation to work    
    RbtCell{camLoop} = inv(Rbt);    
    ccPolyCellNiW = inv(RbtCell{camLoop}) * aug(aug(ccPolyCellNi{camLoop}(1:2,:)));
    camCenW =  inv(RbtCell{camLoop}) * aug([0;0;0]);
    
    pDualVec = ...
        [pDualVec, ...
            DualPointFromThreePointsOnPlane( ccPolyCellNiW(1:3,:), [ccPolyCellNiW(1:3,2:end), ccPolyCellNiW(1:3,1)], camCenW(1:3) )];    
end


vhPoints = ConvexPolyhedronFacesToDualPoints( pDualVec );


vhPoints = CoordAdd( vhPoints, centrePoint );

if draw > 0
    [kMesh, vol] = convhulln(vhPoints'); kMesh=kMesh';
    vol
    if draw>=1,   Newfigure3d, end
    patch('Faces', kMesh', 'Vertices', vhPoints', 'FaceColor', 'y')
end