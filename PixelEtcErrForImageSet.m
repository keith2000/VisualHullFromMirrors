function [pixRes, pixResArray] = PixelEtcErrForImageSet( theCameraVec, polyBoundaryVec, draw )


if ( nargin == 1 ), 
    draw = 0; 
    viewVec = theCameraVec; clear theCameraVec;
    numViews = length(viewVec);
    for viewLoop = 1:numViews
        theCameraVec(viewLoop) = structord(viewVec(viewLoop).camera);

        %save
        polyBoundaryVec{viewLoop} = convexhull( viewVec(viewLoop).boundary );
    end    
end


if ( nargin <= 2 ), draw = 0; end


numCams = length(theCameraVec);

if ( length(polyBoundaryVec) ~= numCams ), error('number of boundaries does not match number of cameras'), end


pixRes = [];

for cam = 1:numCams,
    ccPolyNi{cam} = ImageToNormalisedImage( polyBoundaryVec{cam}, theCameraVec(cam) );
end

for camA = 1:numCams,
    for camB = 1:numCams,
        if ( camA ~= camB )
            
            [tUpper, pUpper, tLower, pLower] = EpiTangPoints(...
                ccPolyNi{camA},...
                ccPolyNi{camB},...
                theCameraVec(camA).pose,...
                theCameraVec(camB).pose );
            
            
            pixPointUpper = NormalisedImageToImage(pUpper(1:2), theCameraVec(camA) ); %points in pixel coords
            pixTangUpper = NormalisedImageToImage(tUpper(1:2), theCameraVec(camA) );            
            diffUpper = pixTangUpper - pixPointUpper;            
            
            pixPointLower = NormalisedImageToImage(pLower(1:2), theCameraVec(camA) ); %points in pixel coords
            pixTangLower = NormalisedImageToImage(tLower(1:2), theCameraVec(camA) );            
            diffLower = pixTangLower - pixPointLower;
             
            %pixRes = [pixRes; diffUpper(:); diffLower(:)];
            pixRes = [pixRes; diffUpper(:); diffLower(:)];
            
            pixResArray(camA,camB) = rms( [diffUpper(:); diffLower(:)] );
            
            if ( draw > 0 )
                     colStr = 'rgbcmy';
                figure( camA + 100 * draw );
                set(gca, 'color', [1 1 1] *.8)
                hold on
                
                msShowPoly(  polyBoundaryVec{camA}, 'EdgeColor', colStr(camA), 'FaceColor', 'none')
                
                plot2d( [pixTangUpper, pixPointUpper], ['-', colStr(camB)]  );
                plot2d( pixPointUpper, ['.', colStr(camB)]  );
                plot2d( pixPointUpper, ['o', colStr(camB)]  );

                plot2d( [pixTangLower, pixPointLower], ['-', colStr(camB)]  );
                plot2d( pixPointLower, ['.', colStr(camB)]  );
                plot2d( pixPointLower, ['o', colStr(camB)]  );
                
            end
            
        end
    end
end

