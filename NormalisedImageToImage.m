function polyI = NormalisedImageToImage( polyNi, theCamera )



if ( size(polyNi,1)  ~= 2 ), error('polyI must be a 2d point set'), end


polyU = polyNi * theCamera.efl;
polyD =  acTsaiRadialDistortion( polyU, theCamera.kappa1 );
polyI(1,:) = polyD(1,:) + theCamera.u0;
polyI(2,:) = polyD(2,:) + theCamera.v0;