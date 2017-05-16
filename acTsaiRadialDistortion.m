function  Pd =  acTsaiRadialDistortion(Pu, kappa1)

% Pd =  acTsaiRadialDistortion(Pu, kappa1)
% Performs first order radial lens distortion using Tsai's model of the image formation process.
% The method for solving the closed form solution is known as the Cardan method.
% For a description see:
%
%@inproceedings{devernay95,
%    author={Fr{\'e}d{\'e}ric Devernay and Olivier Faugeras},
%    title={Automatic calibration and removal of distortion from scenes of structured
%        environments},
%    booktitle={SPIE Conference on Investigative and Trial Image Processing},
%    year={1995},
%    address={San Diego, CA},
%    note={Also available at \url{ftp://ftp-robotvis.inria.fr/pub/html/Papers/devernay-faugeras:95b.ps.gz}},
%    volume={2567}
%}

if (size(Pu,1)~=2)
    error('Pu must have 2 rows.')
end

if (prod(size(kappa1))~=1)
    error('kappa1 should be scalar.')
end


if kappa1==0
    Pd=Pu; 
else
    
    Ru = sqrt(Pu(1,:).^2+Pu(2,:).^2);
    c = 1 / kappa1;
    d = -c * Ru;
    Q = c / 3;
    R = -d / 2;
    D = Q^3 + R.^2;
    
    D1=D;
    D3=D;
    
    %if (D >= 0) 	%%%%	%%/* one real root */
    D1 = sqrt(D1);
    S1 =  CBRT(R + D1);
    T1 =  CBRT(R - D1);
    Rd1 = S1 + T1;
    
    %if (Rd < 0) 
    %   Rd = sqrt(-1 / (3 * kappa1));
    %   sprintf ('\nWarning: undistorted image point to distorted image point mapping limited by\n');
    %  sprintf ('         maximum barrel distortion radius of %lf\n', Rd);
    %  end
    
    %else 		%%	%%/* three real roots */
    D3 = sqrt (-D3);
    S3 = (sqrt(R.^2+D3.^2)).^(1/3);
    T3 = atan2 (D3, R) / 3;
    %sinT=sin(T);cosT=cos(T);
    
    %%%%/* the larger positive root is    2*S*cos(T)                   */
    %%/* the smaller positive root is   -S*cos(T) + SQRT(3)*S*sin(T) */
    %%/* the negative root is           -S*cos(T) - SQRT(3)*S*sin(T) */
    
    Rd3 = -S3 .* cos(T3) + sqrt(3) .* S3 .* sin(T3);%%	%%/* use the smaller positive root */
    %end
    
    Rd=(D>=0).*Rd1+(D<0).*Rd3;
    
    lambda = Rd ./ Ru;
    
    Pd = Pu .* repmat(lambda,2,1);
    
end
function y= CBRT(x)
y=(x>=0).*(x.^(1/3))+(x<0).*(-((-x).^(1/3)));

