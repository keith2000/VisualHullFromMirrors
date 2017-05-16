function fig = NewFigure3D( varargin )

% NewFigure3D( 'light', 1, 'renderer', 'OpenGl' )

light = 0; %this makes the rendering faster -- use only the relevant portion of the image
renderer = 'zbuffer';
VararginModifyDefaults( varargin{:} );


%datenum(version('-date'))
%datenum('23 Sep 2000')

if (  datenum(version('-date')) < datenum('23 Sep 2000') )
    warningState = warning;
    
    if ( ~strcmp(warningState, 'off') )
        disp('Turning warning off to prevent irritating unrecognized OpenGL version warnings')
        warning off;
    end
end

hh = figure;
hold on

cameratoolbar
axis off
axis equal
axis vis3d
view(3)
cameratoolbar('ResetCameraAndSceneLight')
axis equal
set(hh, 'renderer', renderer)

if light > 0
cameratoolbar togglescenelight 
end

if nargout > 0
    fig = hh;
end