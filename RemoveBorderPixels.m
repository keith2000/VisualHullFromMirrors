function bw = RemoveBorderPixels( bw )

%removes regions of ones that touch the image border

bw =  1-bwfill(1-bw, ...
    1:size(bw,2),...
    repmat( size(bw,1), 1, size(bw,2))   );

bw =  1-bwfill(1-bw, 1:size(bw,2), repmat( 1, 1, size(bw,2)) );

bw =  1-bwfill(1-bw, repmat( size(bw,2), 1, size(bw,1)), 1:size(bw,1) );

bw =  1-bwfill(1-bw, repmat( 1, 1, size(bw,1)) , 1:size(bw,1));
