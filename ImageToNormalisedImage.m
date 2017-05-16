function polyNi = ImageToNormalisedImage( polyI, theCamera )


%if (~IsPoly(polyI)), error('polyI must be a 2d point set'), end
if ( size(polyI,1)  ~= 2 ), error('polyI must be a 2d point set'), end

polyD(1,:) = polyI(1,:) - theCamera.u0;
polyD(2,:) = polyI(2,:) - theCamera.v0; %move centroid to principal point


if ( theCamera.kappa1 == 0)
    polyU = polyD;
else
    polyU(1,:) = polyD(1,:) + polyD(1,:) .* ( theCamera.kappa1 * ( polyD(1,:).^2 + polyD(2,:).^2 ) );
    polyU(2,:) = polyD(2,:) + polyD(2,:) .* ( theCamera.kappa1 * ( polyD(1,:).^2 + polyD(2,:).^2 ) );
end


polyNi = polyU / theCamera.efl; %divide by the effective focal length to get the image on the z=1 plane
%units are the same as for the 1 that the z=1 is measured in
