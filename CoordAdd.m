function c=CoordAdd(a,b)

%adds a 3xn to a 3x1 or a 2xn to a 2x1 etc

if iscell(a)
    
    for k=1:length(a),    
        c{k} = CoordAdd( a{k}, b );
    end
    
else

c=a+repmat(b,1,size(a,2));
end


