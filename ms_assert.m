function assert( callerStr, mssgStr )

if not(ischar(callerStr))
    error('assertion expression must be a string, since assert uses evalin')
end

assertionIsTrue = evalin( 'caller', callerStr );

if ( not(assertionIsTrue) ),
    
    if ( nargin == 1 ), mssgStr = ''; end
    
    disp(mssgStr)
    error( ['Assertion failed: ',callerStr] )
end