function ap=aug(p)

%ap=aug(p) augments a 3xn os a 2xn vector p with ones

if (size(p,1)~=3)
   if (size(p,1)~=2)
      error('p must be 3xn or 2xn')
   end
end

ap=[p;p(1,:)*0+1];