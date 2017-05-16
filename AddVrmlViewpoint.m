function AddVrmlViewpoint( fid, cameraPosition, cameraTarget, varargin )

description  = 'another viewpoint';
fieldOfView  = 0.39;

if (  mod(length( varargin),2) == 1  ), error('need even number of arguments: variable name followed by value'); end
for loop = 1:2:length( varargin)    
    if exist(varargin{loop} )
        eval( sprintf('%s = varargin{loop+1};', varargin{loop} ) );
    else
        error('unknown variable')
    end
end

vec = cameraTarget - cameraPosition;

RR = AlignVectors( vec, [0;0;-1] );
aa = MatrixToAxisAngleMex( RR );

fprintf( fid, 'Viewpoint {\n');
fprintf( fid, '   position %g %g %g\n', cameraPosition(1), cameraPosition(2), cameraPosition(3) );
fprintf( fid, '   orientation %g %g %g %g\n', aa(1),  aa(2),  aa(3),  aa(4) );
fprintf( fid, '   fieldOfView %g\n', fieldOfView );
fprintf( fid, '   description "%s"\n', description );
fprintf( fid, '}\n');
