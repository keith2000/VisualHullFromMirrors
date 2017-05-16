function [sclVec, moveAlongJoinVec, sclArray, moveAlongJoinArray] = ...
    ApproxMergeScaleAndPos( polyBoundaryVecCell, allViews, normalAngle, RR_Rec, wSolnVec )

numImages = length(polyBoundaryVecCell);
R = [cos(normalAngle) -sin(normalAngle) 0; sin(normalAngle) cos(normalAngle) 0; 0 0 1]; %rotate about Z-axis
m1Base =     [1;0;0];
m2Base = R * [1;0;0];

for imgLoop = 1:numImages,        
    clear viewVec;
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
    transVec = [ -mirrorJoinPointMirRef(1:2); 0];
    for viewLoop = 1:5,
        viewVec(viewLoop).camera.pose =  inv(  [inv(RR), transVec; [0,0,0,1]] *  inv(viewVec(viewLoop).camera.pose) );                
    end

    orthoViewVec{imgLoop} = ApproxOrthoViewVec( viewVec );
end



for imgLoop = 2:numImages,        
    %compare with orthoViewVec{1} to get scale and translation    
    for viewLoopA = 1:5,
        for viewLoopB = 1:5,            
            unitMoveDirProj = orthoViewVec{1}(viewLoopA).camera.pose(1:2,1:3) * [0;0;1];            
            originProj = orthoViewVec{1}(viewLoopA).camera.pose(1:2,1:4) * [0;0;0;1];            
            MB_to_MA = orthoViewVec{1}(viewLoopA).camera.pose * inv(orthoViewVec{imgLoop}(viewLoopB).camera.pose);
            epiDir = MB_to_MA(1:2,1:3) * [0;0;1];            
           % [aInd,bInd, pA1, pA2] = OrthoTangencies( orthoViewVec{1}(viewLoopA).boundary, MC_to_MA );            
           thePolyAlignedX = [-epiDir(2),epiDir(1)] * orthoViewVec{1}(viewLoopA).boundary;
           maxA = max(thePolyAlignedX);
           minA = min(thePolyAlignedX);           
           thePolyAlignedX = [-epiDir(2),epiDir(1)] * MB_to_MA(1:2,1:4) * aug(aug(orthoViewVec{imgLoop}(viewLoopB).boundary));
           maxB = max(thePolyAlignedX);
           minB = min(thePolyAlignedX);           
           origComp = [-epiDir(2),epiDir(1)] * originProj;           
           moveComp = [-epiDir(2),epiDir(1)] * unitMoveDirProj;           
            %we are interested specifically in the components perpendicular
            %to the epipolar direction
            k = (maxB*minA-maxB*origComp-origComp*minA-maxA*minB+maxA*origComp+origComp*minB)/(-minA+maxA);
            moveAlongJoin = k/moveComp;
            scl = (maxA-minA)/(maxB-minB);
            sclArray(imgLoop-1, viewLoopA+5*(viewLoopB-1) ) = scl;
            moveAlongJoinArray(imgLoop-1, viewLoopA+5*(viewLoopB-1) ) = moveAlongJoin;
        end    
    end
    
end


sclVec = mean(sclArray, 2);
moveAlongJoinVec =  mean(moveAlongJoinArray, 2);