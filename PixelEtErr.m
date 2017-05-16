function resVec = PixelEtErr( viewVec )


for jj = 1:length(viewVec)
    
   viewVec(jj).camera.width = NaN; 
   viewVec(jj).camera.height = NaN;
   viewVec(jj).camera.serial = '';
   
end

resVec = PixelEtErrMex( viewVec );