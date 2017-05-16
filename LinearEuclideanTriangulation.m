function [Xw, rmsErr] = LinearEuclideanTriangulation( Xi, KRt)

%given imaged point locations Xi and projection matrix KRt, world coordinates are computed
%using linear least squares
%if normalised image coordinates are used then Xs == Xi and KRt == RBT

numCams = length(KRt);
numPoints = size(Xi{1},2);

for thePoint = 1:numPoints,
    
    for cam = 1:numCams,
        
        A( (cam-1) * 2 + 1, : ) = ( KRt{cam}(1,1:3) - Xi{cam}(1, thePoint) * KRt{cam}(3,1:3) );
        
        
        
        b( (cam-1) * 2 + 1, 1 ) = Xi{cam}(1, thePoint) * KRt{cam}(3,4) - KRt{cam}(1,4);
        
        A( (cam-1) * 2 + 2, : ) = ( KRt{cam}(2,1:3) - Xi{cam}(2, thePoint) * KRt{cam}(3,1:3) );
        b( (cam-1) * 2 + 2, 1 ) = Xi{cam}(2, thePoint) * KRt{cam}(3,4) - KRt{cam}(2,4);
        
    end

    %A
    %b
    
    A = A(find(~isnan(A(:,1))),:);
    b = b(find(~isnan(b(:,1))),:);
    
    
    if ( length(b) >= 4 )
        
        [U,Sigma,V] = svd(A);
        diags = [diag(1./diag(Sigma)), zeros( size(Sigma,2), size(Sigma,1)-size(Sigma,2))];
        
        Xw(1:3, thePoint) = V * diags * ( U.' * b);
    else
        Xw(1:3, thePoint) = NaN;
    end
    
    errSq = [];
    for cam = 1:numCams,
        XiModel = KRt{cam} * [Xw(1:3, thePoint);1];
        XiModel = XiModel/XiModel(3);
       %([ XiModel(1:2),   Xi{cam}])
        errSq = [errSq , (XiModel(1) - Xi{cam}(1, thePoint))^2 +(XiModel(2) - Xi{cam}(2, thePoint))^2];
    end
    
    rmsErr( thePoint ) = sqrt(mean(errSq));
    
    
end