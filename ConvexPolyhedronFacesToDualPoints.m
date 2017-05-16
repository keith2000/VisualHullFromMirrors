function pVec = ConvexPolyhedronFacesToDualPoints( pPolyhedron, doEnsure )

if nargin < 2
    doEnsure = 1;
end

if doEnsure == 1
    %faster to skip
    %but safer to not skip
    ensure('size(pPolyhedron,1)==3');
    ensure('size(pPolyhedron,2)>=4');
end


kPolyhedron = convhulln( pPolyhedron' )';

for faceLoop = 1:size(kPolyhedron,2)
    A = Augment( pPolyhedron(:, kPolyhedron(:,faceLoop)) )'; %form a matrix consisting of the vertices of the current face
    [U,Sigma,V] = svd(A);    
    theJoin = V(:,end); %right nullspace is the plane/point in homogeneous coords
    if 0 == theJoin(4)
        pVec(:,faceLoop) = [inf;inf;inf];
    else
        pVec(:,faceLoop) = theJoin(1:3)/theJoin(4); %convert to inhomogeneous coords
    end
end


% if 1
%     for faceLoop = 1:size(kPolyhedron,2)
%         
%         vA = pPolyhedron(:, kPolyhedron(2,faceLoop) ) -  pPolyhedron(:, kPolyhedron(1,faceLoop) );
%         vB = pPolyhedron(:, kPolyhedron(3,faceLoop) ) -  pPolyhedron(:, kPolyhedron(1,faceLoop) );
%         
%         faceNormal = cross(vA,vB);
%         
%         d = - dot(faceNormal, pPolyhedron(:, kPolyhedron(1,faceLoop) ) );
%         
%         if 0
%             EvalPrint('dot( [faceNormal; d], aug(  pPolyhedron(:, kPolyhedron(1,faceLoop) ) ) )')
%             EvalPrint('dot( [faceNormal; d], aug(  pPolyhedron(:, kPolyhedron(2,faceLoop) ) ) )')
%             EvalPrint('dot( [faceNormal; d], aug(  pPolyhedron(:, kPolyhedron(3,faceLoop) ) ) )')
%         end
%         
%         pVec(:,faceLoop) = faceNormal/d;
%     end
%     
% else


%end