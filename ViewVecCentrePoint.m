function centrePoint = ViewVecCentrePoint( viewVec )

numViews = length(viewVec);

for viewLoop = 1:numViews,

    cenVecNi{viewLoop} = ImageToNormalisedImage( ...
        PolygonCentroid( convexhull( viewVec(viewLoop).boundary ) ),...
        viewVec(viewLoop).camera );
    RbtCell{viewLoop} = viewVec(viewLoop).camera.pose;
    
end



centrePoint = LinearEuclideanTriangulation( cenVecNi, RbtCell );