function ensure( callerStr, mssgStr )

%ensures that an expression is true
%use for defensive progamming so that bugs may be found quickly

if not(ischar(callerStr))
    error('"ensure" expression must be a string, since assert uses evalin')
end

assertionIsTrue = evalin( 'caller', callerStr );

if ( not(assertionIsTrue) ),
    
    if ( nargin == 1 ), mssgStr = ''; end
    
    disp(mssgStr)
    error( ['ensure expression is false: ',callerStr] )
end