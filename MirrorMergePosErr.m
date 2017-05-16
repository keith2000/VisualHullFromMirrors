function [resVec, mergedViewVec] = MirrorMergePosErr( vec, vecScl, polyBoundaryVecCell, allViews, normalAngle, RR_Rec, wSolnVec )

vec = vec .* vecScl;
numImages = length(polyBoundaryVecCell);
ensure('(numImages-1)*2==length(vec)')

R = [cos(normalAngle) -sin(normalAngle) 0; sin(normalAngle) cos(normalAngle) 0; 0 0 1]; %rotate about Z-axis

m1Base =     [1;0;0];
m2Base = R * [1;0;0];

mergedViewVec = [];



for imgLoop = 1:numImages,        
    clear viewVec;
    
    if imgLoop == 1
        moveAlongJoin = 0; 
        scl = 1;
    else
        moveAlongJoin =  vec((numImages-1)*0 + imgLoop-1);    
        scl = vec((numImages-1)*1 + imgLoop-1);    
    end
    RR = RR_Rec{imgLoop};    
    m1Hat = RR * m1Base;       
    m2Hat = RR * m2Base;
    wSoln = wSolnVec(imgLoop);
    M1_pos = -m1Hat;
    M2_pos = -m2Hat * wSoln;    
    for viewLoop = 1:5,
        viewVec(viewLoop).camera = allViews.camera;
        viewVec(viewLoop).boundary = polyBoundaryVecCell{imgLoop}{viewLoop};
    end    
    viewVec(1).camera.pose = eye(4);
    viewVec(2).camera.pose = ReflectionMatrixFromVecPoint( m1Hat, M1_pos ) * viewVec(1).camera.pose ;
    viewVec(3).camera.pose = ReflectionMatrixFromVecPoint( m2Hat, M2_pos ) * viewVec(1).camera.pose ;
    viewVec(4).camera.pose = ReflectionMatrixFromVecPoint( m1Hat, M1_pos ) * viewVec(3).camera.pose ;
    viewVec(5).camera.pose = ReflectionMatrixFromVecPoint( m2Hat, M2_pos ) * viewVec(2).camera.pose ;    
    piLeft = [m1Hat; -dot(m1Hat, M1_pos)];
    piRight = [m2Hat; -dot(m2Hat, M2_pos)];
    lineDir = cross(m1Hat, m2Hat); lineDir = lineDir/norm(lineDir);
    piCam = [lineDir; 0];    
    [U,Sigma,V] = svd([piLeft, piRight, piCam]');    
    theJoin = V(:,end); %right nullspace is the plane/point in homogeneous coords
    mirrorJoinPointCamRef = theJoin(1:end-1)./theJoin(end);
    mirrorJoinPointMirRef = inv(RR) * mirrorJoinPointCamRef;
    transVec = [ -mirrorJoinPointMirRef(1:2); moveAlongJoin];
    for viewLoop = 1:5,
        viewVec(viewLoop).camera.pose =  inv(  [inv(RR), transVec; [0,0,0,1]] *  inv(viewVec(viewLoop).camera.pose) );
        
        viewVec(viewLoop).camera.pose(1:3,4) = scl * viewVec(viewLoop).camera.pose(1:3,4);
        
    end
    mergedViewVec = [mergedViewVec, viewVec];
end


resVec = PixelEtcErrForImageSet( mergedViewVec );