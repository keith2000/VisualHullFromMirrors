function A=SignedArea(p)

% A=SignedArea(p) 
% 2 A(P) = sum_{i=1}^{n} ( x_i (y_{i+1} - y_{i-1}) )

if (length(size(p))~=2)
   error('Too many dimensions')
end

if (size(p,2)==2)
   disp('Forming transpose')
   p=p.';
end

if (size(p,2)<3)
   size(p)
   error('Too few vertices -- not a polygon')
end

A = (  sum(p(1,:) .*  ( [p(2,2:end),p(2,1)]-[p(2,end),p(2,1:end-1)]) )    )/2;


% Subject 2.01: How do I find the area of a polygon?
% 
%     The signed area can be computed in linear time by a simple sum.
%     The key formula is this:
% 
%         If the coordinates of vertex v_i are x_i and y_i,
%         twice the signed area of a polygon is given by
% 
%         2 A( P ) = sum_{i=0}^{n-1} (x_i y_{i+1} - y_i x_{i+1}).
% 
%     Here n is the number of vertices of the polygon, and index
%     arithmetic is mod n, so that x_n = x_0, etc. A rearrangement
%     of terms in this equation can save multiplications and operate on
%     coordinate differences, and so may be both faster and more
%     accurate:
% 
%        2 A(P) = sum_{i=0}^{n-1} ( x_i  (y_{i+1} - y_{i-1}) )
% 
%     Here again modular index arithmetic is implied, with n=0 and -1=n-1.
%     This can be avoided by extending the x[] and y[] arrays up to [n+1]
%     with x[n]=x[0], y[n]=y[0] and y[n+1]=y[1], and using instead
% 
%        2 A(P) = sum_{i=1}^{n} ( x_i  (y_{i+1} - y_{i-1}) )
% 
% 
%     References: [O' Rourke (C)] Thm. 1.3.3, p. 21; [Gems II] pp. 5-6:
%     "The Area of a Simple Polygon", Jon Rokne.  Dan Sunday's explanation:
%        http://GeometryAlgorithms.com/Archive/algorithm_0101/  where
% 
%     To find the area of a planar polygon not in the x-y plane, use:
%     
%        2 A(P) = abs(N . (sum_{i=0}^{n-1} (v_i x v_{i+1})))
%     
%        where N is a unit vector normal to the plane. The `.' represents the
%        dot product operator, the `x' represents the cross product operator,
%        and abs() is the absolute value function.
%     
%     [Gems II] pp. 170-171:
%     "Area of Planar Polygons and Volume of Polyhedra", Ronald N. Goldman.
