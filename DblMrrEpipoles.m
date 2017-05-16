function [eRV1, eRV2, eV1V12, eV2V21, RV1_indArr, RV2_indArr, V1V12_indArr, V2V21_indArr] = ...
    DblMrrEpipoles( polyBoundaryVec, draw )

%note the ordering of C and D -- this has already caused some problems: C
%is the reflection of A in mirror 2: it's not on the same side as A!

if nargin == 1
    draw = 0;
end

[p1A, p1B, p2A, p2B, ind_1A, ind_1B, ind_2A, ind_2B ] = OuterTangents( polyBoundaryVec{1}, polyBoundaryVec{2} );
eRV1 = ComputeEpipole( p1A, p1B, p2A, p2B );

RV1_indArr(1,1) = ind_1A;
RV1_indArr(2,1) = ind_1B;
RV1_indArr(1,2) = ind_2A;
RV1_indArr(2,2) = ind_2B;


[p1A, p1B, p2A, p2B, ind_1A, ind_1B, ind_2A, ind_2B ] = OuterTangents( polyBoundaryVec{1}, polyBoundaryVec{3} );

eRV2 = ComputeEpipole( p1A, p1B, p2A, p2B );

RV2_indArr(1,1) = ind_1A;
RV2_indArr(2,1) = ind_1B;
RV2_indArr(1,2) = ind_2A;
RV2_indArr(2,2) = ind_2B;


[p1A, p1B, p2A, p2B, ind_1A, ind_1B, ind_2A, ind_2B ] = OuterTangents( polyBoundaryVec{2}, polyBoundaryVec{4} );
eV1V12 = ComputeEpipole( p1A, p1B, p2A, p2B );


V1V12_indArr(1,1) = ind_1A;
V1V12_indArr(2,1) = ind_1B;
V1V12_indArr(1,2) = ind_2A;
V1V12_indArr(2,2) = ind_2B;

[p1A, p1B, p2A, p2B, ind_1A, ind_1B, ind_2A, ind_2B ] = OuterTangents( polyBoundaryVec{3}, polyBoundaryVec{5} );
eV2V21 = ComputeEpipole( p1A, p1B, p2A, p2B );

V2V21_indArr(1,1) = ind_1A;
V2V21_indArr(2,1) = ind_1B;
V2V21_indArr(1,2) = ind_2A;
V2V21_indArr(2,2) = ind_2B;

if draw

    plot2d( [polyBoundaryVec{1}(:,RV1_indArr(1,1)), eRV1], '-k' );
    plot2d( [polyBoundaryVec{1}(:,RV1_indArr(1,2)), eRV1], '-k' );
    plot2d( [eRV1], 'og' );

    plot2d( [polyBoundaryVec{1}(:,RV1_indArr(1,1))], '.', 'color', [1 0.5 0] );
    plot2d( [polyBoundaryVec{2}(:,RV1_indArr(2,1))], '.', 'color', [1 0.5 0] );
    plot2d( [polyBoundaryVec{1}(:,RV1_indArr(1,2))], '.', 'color', [ 0.5 0 1] );
    plot2d( [polyBoundaryVec{2}(:,RV1_indArr(2,2))], '.', 'color', [ 0.5 0 1] );

    
    plot2d( [polyBoundaryVec{1}(:,RV2_indArr(1,1)), eRV2], '-k' );
    plot2d( [polyBoundaryVec{3}(:,RV2_indArr(2,2)), eRV2], '-k' );
    plot2d( [eRV2], 'ob' );
    
    plot2d( [eRV1, eRV2], 'k-' );
    
    plot2d( [polyBoundaryVec{2}(:,V1V12_indArr(1,1)), eV1V12], '-k' );
    plot2d( [polyBoundaryVec{4}(:,V1V12_indArr(2,2)), eV1V12], '-k' );
    plot2d( [eV1V12], 'oc' );

    plot2d( [polyBoundaryVec{3}(:,V2V21_indArr(1,1)), eV2V21], '-k' );
    plot2d( [polyBoundaryVec{5}(:,V2V21_indArr(2,2)), eV2V21], '-k' );
    plot2d( [eV2V21], 'om' );
    
    [p1A, p1B, p2A, p2B, ind_1A, ind_1B, ind_2A, ind_2B ] = OuterTangents( polyBoundaryVec{3}, polyBoundaryVec{4} );
    plot2d( [p1A, eRV1], '-', 'color', [1 1 1] * 0.8  );
    plot2d( [p2A, eRV1], '-', 'color', [1 1 1] * 0.8  );
   
    [p1A, p1B, p2A, p2B, ind_1A, ind_1B, ind_2A, ind_2B ] = OuterTangents( polyBoundaryVec{2}, polyBoundaryVec{5} );
    plot2d( [p1A, eRV2], '-', 'color', [1 1 1] * 0.8 );
    plot2d( [p2A, eRV2], '-', 'color', [1 1 1] * 0.8 );
    
end



function epipole = ComputeEpipole( p1A, p1B, p2A, p2B )

L1 = cross(aug(p1A),aug(p1B));
L2 = cross(aug(p2A),aug(p2B));
epipoleHomog = cross(L1,L2);
epipole = epipoleHomog(1:2)/epipoleHomog(3);

