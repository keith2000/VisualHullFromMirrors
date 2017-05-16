function EvalPrint( evalStr )

ms_assert('all(ischar(evalStr))', 'expression must be enclosed in single quotes')
theResult = evalin('caller', evalStr);

if ( length( theResult ) == 1 )
    fprintf('%s: %g\n', evalStr, theResult )
    
else
    
    fprintf('%s: [', evalStr )    
    for loop = 1:length(theResult(:))
        fprintf('%s%g', char(', ' * (loop>1)), theResult(loop) )
    end
    fprintf(']\n')        
    
end