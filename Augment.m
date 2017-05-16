function y = Augment( x )

%augments a row of ones to x

y = [x; ones(1, size(x,2) )];