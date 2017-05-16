function [t1Upper, pUpper, t1Lower, pLower] = EpiTangPoints( silPoly1, silPoly2, RBT1, RBT2)

if ( not( all( size(RBT1) == [4,4] ) ) )
    error('The rigid body transform must be represented by a 4x4 matrix')
end

if ( not( all( size(RBT2) == [4,4] ) ) )
    error('The rigid body transform must be represented by a 4x4 matrix')
end

if ( size(silPoly1,1)~=2 )
    error('silPoly must be 2xn ')
end

if ( size(silPoly2,1)~=2 )
    error('conePoly must be 2xn ')
end


Rt12 = RBT2  * inv(RBT1); %r.b.t. from view 1 to view 2
E12 = AntiSymmetricMatrixFromVector( Rt12(1:3,4) ) * Rt12(1:3,1:3);
E21 = E12';

[U,Sigma,V] = svd(E12);
e12 = V(1:3,3)/V(3,3); %projection in to view 1 of cam centre 2

e12 =e12/(e12(3));

[U,Sigma,V] = svd(E21);
e21 = V(1:3,3)/V(3,3); %projection in to view 2 of cam centre 1

e21 = e21/(e21(3));

%fprintf('e12: %20.15f %20.15f\ne21: %20.15f %20.15f\n',e12(1),e12(2),e21(1), e21(2))

[t1Upper, t1Lower] = PointPolygonTangentExtremes(e12, silPoly1(1:2,:) );
[t2Upper, t2Lower] = PointPolygonTangentExtremes(e21, silPoly2(1:2,:) );

%fprintf('t120: %20.15f %20.15f\nt121: %20.15f %20.15f\n',t120(1),t120(2),t121(1), t121(2))
%fprintf('t210: %20.15f %20.15f\nt211: %20.15f %20.15f\n',t210(1),t210(2),t211(1), t211(2))

t1Upper = [t1Upper;1];
t1Lower = [t1Lower;1];
t2Upper = [t2Upper;1];
t2Lower = [t2Lower;1];

if  (  abs( t2Lower' * E12 * t1Upper ) < abs( t2Upper' * E12 * t1Upper ) )
    
    temp = t2Upper;
    t2Upper = t2Lower;
    t2Lower = temp;
    
end



lineUpper = E21 * t2Upper; %from view 2 to view 1 E21 = E'


% 00260 vgl_homg_operators_2d<T>::perp_line_through_point(const vgl_homg_line_2d<T>& l,
% 00261                                                   const vgl_homg_point_2d<T>& p)
% 00262 {
% 00263   vgl_homg_point_2d<T> d(l.a(), l.b(), 0);
% 00264   return cross(d, p);
% 00265 }

lineDirection = [lineUpper(1); lineUpper(2); 0];
perpLineThruPoint = cross( lineDirection, t1Upper );

% 0269 //: Get the perpendicular projection of point onto line.
% 00270 template <class T>
% 00271 vgl_homg_point_2d<T>
% 00272 vgl_homg_operators_2d<T>::perp_projection(const vgl_homg_line_2d<T>& line,
% 00273                                           const vgl_homg_point_2d<T>& point)
% 00274 {
% 00275   vgl_homg_line_2d<T> perpline = perp_line_through_point (line, point);
% 00276   vgl_homg_point_2d<T> answer = cross(line, perpline);
% 00277   return answer;
% 00278 }

pUpper = cross( lineUpper, perpLineThruPoint);
pUpper = pUpper./ pUpper(3);

lineLower = E21 * t2Lower; 
lineDirection = [lineLower(1); lineLower(2); 0];
perpLineThruPoint = cross( lineDirection, t1Lower );
pLower = cross( lineLower, perpLineThruPoint);
pLower = pLower./ pLower(3);

