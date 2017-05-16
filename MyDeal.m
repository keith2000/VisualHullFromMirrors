function varargout = MyDeal(x)

%ms_assert('thisNargout == length(x)') %breaks in matlab2016 but not matlab2015

for loop = 1:nargout
    varargout{loop} = x(loop);
end