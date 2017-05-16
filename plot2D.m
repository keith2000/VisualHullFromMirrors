function plot2D( P, varargin )
% plot2D(P,str) uses a 2xn vector P and plots it using plot

if not(isempty(P))
    
    if ( size(P,1)~=2 ), error('P must have 2 rows'), end
    plot( P(1,:), P(2,:), varargin{:} )
    
end
