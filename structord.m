function y = structord(x) 
%STRUCTORD - Sort structure fieldnames alphabetically 
% 
%Syntax:  y = structord(x); 
% 
%See also: SORTSTRUCT 


%Author: Nabeel, The Mathworks 
%Newsgroup post: 06-JUN-2000 


Dimensions = size(x);   
x = x(:); 
[f,ix] = sort(fieldnames(x)); 
v = struct2cell(x); 
y = cell2struct(v(ix,:),f,1);   
y = reshape(y,Dimensions); 
