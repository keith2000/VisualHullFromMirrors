function orthoViewVec = ApproxOrthoViewVec( viewVec )

centrePoint = ViewVecCentrePoint( viewVec );


for viewLoop = 1:length(viewVec)    
    orthoViewVec(viewLoop).camera.pose = viewVec(viewLoop).camera.pose;    
    centrePointCam = viewVec(viewLoop).camera.pose * aug(centrePoint);    
    depth = centrePointCam(3);        
    orthoViewVec(viewLoop).boundary = viewVec(viewLoop).boundary ;    
    orthoViewVec(viewLoop).boundary(1,:) = orthoViewVec(viewLoop).boundary(1,:) - viewVec(viewLoop).camera.u0;
    orthoViewVec(viewLoop).boundary(2,:) = orthoViewVec(viewLoop).boundary(2,:) - viewVec(viewLoop).camera.v0;        
    orthoViewVec(viewLoop).boundary = orthoViewVec(viewLoop).boundary * depth/viewVec(viewLoop).camera.efl;        
end
