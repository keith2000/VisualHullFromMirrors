function pOut = Perp( pIn, noCheck )

if nargin < 2
    ms_assert('all(size(pIn)==[2,1])')
end

pOut(1,1) = -pIn(2);
pOut(2,1) = +pIn(1);