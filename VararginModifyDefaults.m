function VararginModifyDefaults( varargin )


%example usage
% cellsAcross = 180;
% doDisplayLoop = 1;
% VararginModifyDefaults( varargin{:} );
if (  mod(length( varargin),2) == 1  ), error('need even number of arguments: variable name followed by value'); end
for loop = 1:2:length( varargin)
    
    if evalin('caller', ['exist(''',varargin{loop},''')'])
        
       
        
%         if 0
%             if  ischar(varargin{loop+1})
%                 evalin('caller', sprintf('%s=''%s'';', varargin{loop}, varargin{loop+1} ) );
%             else
%                 evalin('caller', sprintf('%s=%g;', varargin{loop}, varargin{loop+1} ) );
%             end
%         else
%             evalin('caller', sprintf('%s = varargin{%d};', varargin{loop}, loop+1 ) );
%         end
                
         assignin('caller', varargin{loop}, varargin{loop+1} );
         
    else
        error( sprintf('unknown variable %s (specified value: %g)', varargin{loop}, varargin{loop+1} ) )
    end
end