% [y1,y2]=unitize(x1,x2);
% Normalize the x's so that their sum of squares is 1

function [y1,y2]=unitize(x1,x2)
   if (nargin==1)
      y1 = x1/norm(x1);
   else
      z = sqrt(x1^2+x2^2);
      y1 = x1/z;
      y2 = x2/z;
   end;
%end;
