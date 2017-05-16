%CompileMirrorMex1


disp('compiling GetBoundaryMex')
mex   GetBoundaryMex.cpp


disp('compiling SelectObjectMex')
mex   SelectObjectMex.cpp


disp('compiling MatrixToAxisAngleMex')
mex   MatrixToAxisAngleMex.cpp Wm3Math.cpp


disp('compiling PixelEtErrMex')
mex   PixelEtErrMex.cpp SilFunctions.cpp SilhouetteSetPankaj.cpp SilhouetteViewPankaj.cpp VcalCamera.cpp Wm3ContConvexHull2.cpp Wm3ContHullEdge2.cpp ...
    Wm3Math.cpp Wm3Matrix4.cpp Wm3Matrix3.cpp Wm3Vector2.cpp

disp('compiling MarchDemoMex') 
mex MarchDemoMex.cpp ...
    helper.cpp...
    HomogeneousCoord.cpp...
    MatlabGreyImage.cpp...
    Matrix4x4.cpp ...
    Matrix4x4_Matlab.cpp ...
    polygonizer.cpp ...
    VisualHullFunction.cpp
