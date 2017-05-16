function polyBoundaryCell = ExtractBoundaries( im,  varargin )

draw = 0;
numBoundaries = 5;
doRemoveBorderPixels = 1;

VararginModifyDefaults( varargin{:} );


if doRemoveBorderPixels > 0
%    im = RemoveBorderPixels( im );
end

for loop = 1:numBoundaries,    
    cc = SelectObjectMex( im, loop );
    polyBoundaryCell{loop} = GetBoundaryMex(cc)';    
end


if draw > 0
    figure, imshow(im)
    for loop = 1:numBoundaries, 
        ShowPoly( polyBoundaryCell{loop}, 'FaceColor', 'none', 'EdgeColor', MyPalette(loop), 'LineWidth', 2)
    end
end