function [resVec, normalAngle, efl, p0, RR_Rec, allViews] = MirrorFocalPpErr( vec, vecScl, polyBoundaryVecCell, draw )

vec = vec .* vecScl;

if not(exist('draw'))
    draw = 0;
end

numImages = length(polyBoundaryVecCell);

%length(vec)
%numImages

%ensure('numImages*4+4==length(vec)')

%The angle between the mirrors is constant -- this provides a further
%constraint on the epipoles. Each epipole pair is represented by a base
%pair of unit vectors separated by the angle between the mirror normals
% this pair is then tumbled into place using a rotation represented by a
% quaternion 

normalAngle = vec(end-3);
efl = vec(end-2);
p0 = [vec(end-1);vec(end)];

allViews.camera.efl = efl;
allViews.camera.u0 = p0(1);
allViews.camera.v0 = p0(2);
allViews.camera.kappa1 = 0;

R = [cos(normalAngle) -sin(normalAngle) 0; sin(normalAngle) cos(normalAngle) 0; 0 0 1]; %rotate about Z-axis

m1Base =     [1;0;0];
m2Base = R * [1;0;0];

for imgLoop = 1:numImages,    
    
    quat  = vec( 4*(imgLoop-1) + (1:4) );
    
    RR = QuaternionToRotationMatrix(quat);
       RR_Rec{imgLoop} = RR;
    
    m1Hat = RR * m1Base;       
    m2Hat = RR * m2Base;
    
    M1Reflect = ReflectionMatrixFromVec( m1Hat );
    M2Reflect = ReflectionMatrixFromVec( m2Hat );
    
    
    m212Hat = M2Reflect * M1Reflect * m2Hat;
    m121Hat = M1Reflect * M2Reflect * m1Hat;
    
    
%    EvalPrint('m1Hat')
    
    eV1Vec(:,imgLoop) = p0 + efl * m1Hat(1:2)/m1Hat(3);
    eV2Vec(:,imgLoop) = p0 + efl * m2Hat(1:2)/m2Hat(3);
    
    
    % eV121Vec(:,imgLoop) = p0 + efl * m121Hat(1:2)/m121Hat(3);
    % eV212Vec(:,imgLoop) = p0 + efl * m212Hat(1:2)/m212Hat(3);
    
    
    
    
    m21Hat = M1Reflect * m2Hat;
    m12Hat = M2Reflect * m1Hat;
    
    eV21 = p0 + efl * m21Hat(1:2)/m21Hat(3);
    eV12 = p0 + efl * m12Hat(1:2)/m12Hat(3);
    
    
    
    eV12Vec(:,imgLoop) = eV12;
    eV21Vec(:,imgLoop) = eV21;
    
    
    
    if  draw > 0
        figure(imgLoop)
        plot2D( eV1Vec(:,imgLoop), 'or', 'MarkerSize', 15, 'LineWidth', 2)
        plot2D( eV2Vec(:,imgLoop), 'og', 'MarkerSize', 15, 'LineWidth', 2)
        %plot2d( eV121Vec(:,imgLoop), 'om', 'MarkerSize', 15, 'LineWidth', 2)
        %plot2d( eV212Vec(:,imgLoop), 'oc', 'MarkerSize', 15, 'LineWidth', 2)
        plot2D( eV21, 'oc', 'MarkerSize', 11, 'LineWidth', 2)
        plot2D( eV12, 'om', 'MarkerSize', 11, 'LineWidth', 2)
        
        
    end
    
    
    
    %     eV1Vec(1,imgLoop) = vec( 4*(imgLoop-1) + 1 );
    %     eV1Vec(2,imgLoop) = vec( 4*(imgLoop-1) + 2 );
    %     eV2Vec(1,imgLoop) = vec( 4*(imgLoop-1) + 3 );
    %     eV2Vec(2,imgLoop) = vec( 4*(imgLoop-1) + 4 );    
end



%The



resVec = [];

for imgLoop = 1:numImages,  
    
    polyBoundaryVec = polyBoundaryVecCell{imgLoop};
    
    eRV1 = eV1Vec(:,imgLoop);
    eRV2 = eV2Vec(:,imgLoop);
    
    eV2V21 = eV12Vec(:,imgLoop);
    eV1V12 = eV21Vec(:,imgLoop);
    
    resVec = [ resVec, ...
            FourErrorDistances( eRV1, polyBoundaryVec{1}, polyBoundaryVec{2} ), ...
            FourErrorDistances( eRV1, polyBoundaryVec{3}, polyBoundaryVec{4} ), ...
            FourErrorDistances( eRV2, polyBoundaryVec{1}, polyBoundaryVec{3} ), ...
            FourErrorDistances( eRV2, polyBoundaryVec{2}, polyBoundaryVec{5} ), ...
            FourErrorDistances( eV2V21, polyBoundaryVec{5}, polyBoundaryVec{3} ), ...
            FourErrorDistances( eV1V12, polyBoundaryVec{4}, polyBoundaryVec{2} )    ];
end