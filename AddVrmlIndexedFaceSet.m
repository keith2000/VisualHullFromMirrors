function AddVrmlIndexedFaceSet( fid, pp, kk, varargin)


objNameStr = 'MyObject';
VararginModifyDefaults( varargin{:} );

%pp = pp*1000;

ensure('size(pp,1)==3')

numPoints = size(pp, 2);
numFaces = size(kk, 2);
numPolyVertices = size(kk, 1);

fprintf(fid, 'PROTO %s[\n', objNameStr);
fprintf(fid, 'field SFColor faceColour 1 0.5 0\n');
fprintf(fid, ']\n');
fprintf(fid, '{\n');
fprintf(fid, 'Shape {\n');
fprintf(fid, '   appearance Appearance {\n');
fprintf(fid, ' material DEF MAT0 Material{\n');
fprintf(fid, 'diffuseColor IS faceColour\n');
fprintf(fid, '}\n');
fprintf(fid, '}\n');
fprintf(fid, '   geometry IndexedFaceSet {\n');
fprintf(fid, '   coord DEF patch0 Coordinate{\n');
fprintf(fid, '    point [\n');



%fprintf(fid, '    point [\n');

for pointLoop = 1:numPoints,
    fprintf(fid, '%g %g %g,\n', pp(1,pointLoop), pp(2,pointLoop), pp(3,pointLoop)  );
end

fprintf(fid, ']\n');
fprintf(fid, '}\n');
%fprintf(fid, '}\n');
fprintf(fid, 'coordIndex [\n');
%0,1,2,3,-1
for faceLoop = 1:numFaces,
    for vLoop = 1:numPolyVertices
        fprintf(fid, '%d, ', kk(vLoop,faceLoop)-1 );
    end
    fprintf(fid, '-1\n');
end

fprintf(fid, ']\n');
fprintf(fid, '}\n');
fprintf(fid, '}\n');
fprintf(fid, '}\n');


fprintf(fid, '%s{}%use object\n', objNameStr);

