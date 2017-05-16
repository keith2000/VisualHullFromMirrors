function [resVec, viewVecCell, allViews, normalAngle, RR_Rec, wSolnVec] = MirrorPosFocalPpErrOPT( vec, vecScl, polyBoundaryVecCell )




persistent polyBoundaryVecCell;
persistent vecScl;

%if isempty(polyBoundaryVec)
if 0==nargin
    disp('reset')
    load('temp.mat', 'polyBoundaryVecCell','vecScl')
else
    vec = vec .* vecScl;
    
    numImages = length(polyBoundaryVecCell);
    
    ensure('numImages*5+4==length(vec)')
    
    normalAngle = vec(end-3);
    efl = vec(end-2);
    p0 = [vec(end-1);vec(end)];
    
    R = [cos(normalAngle) -sin(normalAngle) 0; sin(normalAngle) cos(normalAngle) 0; 0 0 1]; %rotate about Z-axis
    
    m1Base =     [1;0;0];
    m2Base = R * [1;0;0];
    
    allViews.camera.efl = efl;
    allViews.camera.u0 = p0(1);
    allViews.camera.v0 = p0(2);
    allViews.camera.kappa1 = 0;
    
    for imgLoop = 1:numImages,
        
        wSoln = vec(imgLoop);
        
        wSolnVec(imgLoop) = wSoln;
        quat  = vec( numImages + 4*(imgLoop-1) + (1:4) );
        RR = QuaternionToRotationMatrix(quat);
        
        %if any( not(isreal(RR)) )
        %RR = eye(3);
        %end
        
        RR_Rec{imgLoop} = RR;
        
        %RR
        
        m1Hat = RR * m1Base;
        m2Hat = RR * m2Base;
        
        M1_pos = -m1Hat;
        M2_pos = -m2Hat * wSoln;
        
        for viewLoop = 1:5,
            viewVecCell{imgLoop}(viewLoop).camera = allViews.camera;
            viewVecCell{imgLoop}(viewLoop).boundary = polyBoundaryVecCell{imgLoop}{viewLoop};
        end
        
        viewVecCell{imgLoop}(1).camera.pose = eye(4);
        %    norm(m1Hat)
        viewVecCell{imgLoop}(2).camera.pose = ReflectionMatrixFromVecPoint( m1Hat, M1_pos ) * viewVecCell{imgLoop}(1).camera.pose ;
        viewVecCell{imgLoop}(3).camera.pose = ReflectionMatrixFromVecPoint( m2Hat, M2_pos ) * viewVecCell{imgLoop}(1).camera.pose ;
        viewVecCell{imgLoop}(4).camera.pose = ReflectionMatrixFromVecPoint( m1Hat, M1_pos ) * viewVecCell{imgLoop}(3).camera.pose ;
        viewVecCell{imgLoop}(5).camera.pose = ReflectionMatrixFromVecPoint( m2Hat, M2_pos ) * viewVecCell{imgLoop}(2).camera.pose ;
        
        for jj = 1:length(viewVecCell{imgLoop})
            viewVecCell{imgLoop}(jj).camera.width = NaN;
            viewVecCell{imgLoop}(jj).camera.height = NaN;
        end
        
    end
    
    
    resVec = [];
    %save
    for imgLoop = 1:numImages,
        
        %  resVec = [ resVec; PixelEtcErrForImageSet( viewVecCell{imgLoop} )  ];
        resVec = [ resVec; PixelEtErrMex( viewVecCell{imgLoop} )  ];
        
    end
    
end