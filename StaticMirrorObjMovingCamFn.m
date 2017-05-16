function [mergedViewVec, vecOpt, polyBoundaryVecCell] = StaticMirrorObjMovingCamFn( polyBoundaryVecCell, varargin )

doFinalLm = 1;
doLm = 1;
draw = 0;
p0Guess = [NaN;NaN];
mfe = 300;
tol = 1e-3;
doOrderSilhouettes = 1; %set this to zero if the silhouettes are already ordered according to which is which

useMiddleOfSilhouettesAsInitPp = 1;
%set this to zero if you want the principal point to be estimated geometrically
%the camera must roll at least a little between shots for this to work

VararginModifyDefaults( varargin{:} );

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if doLm > 0
    doFinalLm = 1; %if we are to do LM, we must do final LM
end


close all

%options=optimset( 'LevenbergMarquardt','on', 'MaxFunEvals',mfe,  'TolFun',tol,...
%    'LargeScale','on', 'TolX',tol, 'Display','off');

tol = 1e-3;
% options = LMFsolve('default');
% options = LMFsolve...
%     (options,...
%     'XTol',tol,...
%     'FTol',tol,...
%     'ScaleD',1,...
%     'Display',0);


options = LMFnlsq('default');
options = LMFnlsq...
    (options,...
    'XTol',tol,...
    'FTol',tol,...
    'ScaleD',1,...
    'Display',0);


numPoses = length(polyBoundaryVecCell);
for poseLoop = 1:numPoses,
    
    
    if doOrderSilhouettes > 0
        [ polyBoundaryVecCell{poseLoop}, indVec] = OrderMostConsistentEpipoles(  polyBoundaryVecCell{poseLoop} );
    end
    polyBoundaryVec = polyBoundaryVecCell{poseLoop};
    
    
    if draw > 0
        figure(poseLoop)
        for viewLoop = 1:5,
            hold on
            ShowPoly( polyBoundaryVec{viewLoop},...
                'FaceColor', MyPalette(poseLoop),  'EdgeColor',  MyPalette(viewLoop),      'FaceAlpha', 0.15);
            axis ij,    axis equal
        end
    end
    
    
    [eRV1Init, eRV2Init, eV1V12Init, eV2V21Init, RV1_indArr, RV2_indArr, V1V12_indArr, V2V21_indArr] = ...
        DblMrrEpipoles( polyBoundaryVec, 0 );
    
    lineDir = eRV2Init - eRV1Init;
    projPointV1V12 = cross( cross(aug(eRV1Init), aug(eRV2Init)), cross(aug(eV1V12Init), aug(eV1V12Init+Perp(lineDir)) ));
    projPointV1V12 = projPointV1V12(1:2)/projPointV1V12(3);
    projPointV2V21 = cross( cross(aug(eRV1Init), aug(eRV2Init)), cross(aug(eV2V21Init), aug(eV2V21Init+Perp(lineDir)) ));
    projPointV2V21 = projPointV2V21(1:2)/projPointV2V21(3);
    aDist = norm(projPointV2V21-eRV1Init);
    bDist = norm(projPointV1V12-eRV2Init);
    vecScl = [eRV1Init; eRV2Init; aDist; bDist];
    vecInitLm = ones(size(vecScl));
    errInit = DblMrrEpipoleLineErr( vecInitLm, polyBoundaryVec, vecScl, 0);
    EvalPrint('rms(errInit)')
    if doLm > 0
        %vecOpt = lsqnonlin('DblMrrEpipoleLineErr', vecInitLm, [], [], options, polyBoundaryVec, vecScl, 0 );
        
%         save('temp.mat', 'polyBoundaryVec','vecScl')
%         DblMrrEpipoleLineErrOPT(); %call with 0==nargin to reset persistent vars. This is needed when we are in a loop
%         vecOpt=LMFsolve('DblMrrEpipoleLineErrOPT',vecInitLm(:)',options);
%         
        
        vecOpt=LMFnlsq(@(x) MyVectorize(DblMrrEpipoleLineErr(x(:), polyBoundaryVec, vecScl, 0)),vecInitLm,options);
        
        %vecOpt=LMFsolve(@(x) MyVectorize(DblMrrEpipoleLineErr(x(:), polyBoundaryVec, vecScl, 0)),vecInitLm,options);
        
        
    else
        vecOpt = vecInitLm;
    end
    [errOpt, eRV1, eRV2, eV1V12, eV2V21] = DblMrrEpipoleLineErr( vecOpt, polyBoundaryVec, vecScl, draw);
    EvalPrint('rms(errOpt)')
    
    
    %     EvalPrint('rms(  DblMrrEpipoleLineErrOPT(vecOpt)  )')
    %
    %     if 2==poseLoop
    %            errInit = DblMrrEpipoleLineErr( vecInitLm, polyBoundaryVec, vecScl, 0);
    %     EvalPrint('rms(errInit)')
    %        stop
    %     end
    
    lineDir = eRV2 - eRV1;
    lineDirHat = lineDir/norm(lineDir);
    R = [unitize(eRV2 - eRV1)'; Perp(unitize(eRV2 - eRV1))']; %we need this to cope with negative values
    epipoleLine = R * [eRV1,  eV2V21,   eV1V12, eRV2];
    a = epipoleLine(1,2) - epipoleLine(1,1);
    b = epipoleLine(1,3) - epipoleLine(1,2);
    c = epipoleLine(1,4) - epipoleLine(1,3);
    f_pi = 1/2*(3*a*c+4*a*b+4*c*b+4*b^2)^(1/2)*(a+b+c)*a^(1/2)*c^(1/2)/(a^2+a*b+c^2+c*b+a*c);
    pE = eRV1 + 1/2*(2*a+2*b+c)*a*(a+b+c)/(a^2+a*b+c^2+c*b+a*c) * lineDirHat;
    if draw,
        plot2D(pE, 'ok'),
        plot2D([eRV2, eRV1], '-k'),
    end
    peVec(:, poseLoop) = pE;
    perpDirVec(:, poseLoop) = Perp( lineDirHat );
    f_piVec(poseLoop) = f_pi;
    eRV1Vec(:, poseLoop) = eRV1;
    eRV2Vec(:, poseLoop) = eRV2;
end



if numPoses > 1
    combList = nchoosek( 1:numPoses, 2);
    
    for combLoop = 1:size(combList,1),
        p0 = PointDirLineIntersect(  ...
            peVec(:, combList(combLoop,1)), perpDirVec(:, combList(combLoop,1)),...
            peVec(:, combList(combLoop,2)), perpDirVec(:, combList(combLoop,2)) );
        EvalPrint('p0');
        p0Vec(:, combLoop) = p0;
        for poseLoop = 1:numPoses,
            if draw>0
                figure(poseLoop)
                plot2D(p0, '*', 'color', MyPalette(combLoop) );
                plot2D([p0, peVec(:, combList(combLoop,1))], '-k' );
                plot2D([p0, peVec(:, combList(combLoop,2))], '-k' );
            end
        end
    end
    
    p0Avg = mean(p0,2); % use the average as an initial guess
    %EvalPrint('p0Avg');
    
    if useMiddleOfSilhouettesAsInitPp > 0
        p0Avg = mean([polyBoundaryVec{1:5}],2); %estimate the principal point as somewhere in the middle of the silhouettes
    end
    
else
    
    
    p0Avg = mean([polyBoundaryVec{1:5}],2); %estimate the principal point as somewhere in the middle of the silhouettes
    %(if only one image is available)
    
end



%save
if all(not(isnan(p0Guess)))
    %use user provided principal point estimate (if given)
    p0Avg =  p0Guess;%[518/2; 388/2];
end

EvalPrint('p0Avg');

for poseLoop = 1:numPoses,
    distFromPpToEpipoleLine = PerpDist2DPointTo2DLine( eRV1Vec(:, poseLoop), Perp(perpDirVec(:, poseLoop)), p0Avg);
    efl = sqrt( f_piVec(poseLoop)^2 - distFromPpToEpipoleLine^2 );
    EvalPrint('efl');
    m1Hat = -[ eRV1Vec(:, poseLoop) - p0Avg; efl ]; m1Hat = m1Hat/norm(m1Hat);
    m2Hat = -[ eRV2Vec(:, poseLoop) - p0Avg; efl ]; m2Hat = m2Hat/norm(m2Hat);
    normalAngle = acos(dot(m1Hat, m2Hat));
    EvalPrint('(pi-normalAngle) * (180/pi)')
    eflVec(poseLoop) = efl;
    normalAngleVec(poseLoop) = normalAngle;
end



eflAvg = mean(eflVec);
normalAngleAvg = mean(normalAngleVec);

EvalPrint('eflAvg')
EvalPrint('normalAngleAvg * (180/pi)')

R = [cos(normalAngleAvg) -sin(normalAngleAvg) 0; sin(normalAngleAvg) cos(normalAngleAvg) 0; 0 0 1]; %rotate about Z-axis
m1Base =     [1;0;0];
m2Base = R * [1;0;0];
perpDir = cross(m1Base, m2Base); perpDir = perpDir/norm(perpDir);
avgDir = m1Base + m2Base; avgDir = avgDir/norm(avgDir);
baseRot = [avgDir, perpDir, cross(avgDir, perpDir)];
vecInit = [];
for poseLoop = 1:numPoses,
    m1Hat = -[ eRV1Vec(:, poseLoop) - p0Avg; efl ]; m1Hat = m1Hat/norm(m1Hat);
    m2Hat = -[ eRV2Vec(:, poseLoop) - p0Avg; efl ]; m2Hat = m2Hat/norm(m2Hat);
    perpDir = cross(m1Hat, m2Hat); perpDir = perpDir/norm(perpDir);
    avgDir = m1Hat + m2Hat; avgDir = avgDir/norm(avgDir);
    thisRot = [avgDir, perpDir, cross(avgDir, perpDir)];
    quat = RotationMatrixToQuaternion( thisRot * inv(baseRot)  );
    vecInit = [vecInit; quat];
end

vecInit = [vecInit; normalAngleAvg; eflAvg; p0Avg];
vecScl = vecInit;
disp('MirrorFocalPpErr')
resInit = MirrorFocalPpErr( ones(size(vecScl)), vecScl, polyBoundaryVecCell, 0 );
EvalPrint('rms(resInit)');



if doLm > 0
    % vecOpt = lsqnonlin('MirrorFocalPpErr', ones(size(vecScl)), [], [], options, vecScl, polyBoundaryVecCell );
    
    
    %save('temp.mat', 'polyBoundaryVecCell','vecScl')
    %vecOpt=LMFsolve('MirrorFocalPpErrOPT',ones(size(vecScl))',options);
    
    vecOpt=LMFnlsq(@(x) MyVectorize(MirrorFocalPpErr(x(:), vecScl, polyBoundaryVecCell, 0)),ones(size(vecScl)),options);
    
    
else
    vecOpt = ones(size(vecScl));
end


[resOpt, normalAngle, efl, p0, RR_Rec, allViews] = MirrorFocalPpErr( vecOpt, vecScl, polyBoundaryVecCell, draw );
EvalPrint('rms(resOpt)');
EvalPrint('(pi-normalAngle)*180/pi');
EvalPrint('efl');
EvalPrint('p0');

wSolnInitVec = OrthoEstMirrorDistRatio( polyBoundaryVecCell, normalAngle, RR_Rec, allViews );

vecInit = [wSolnInitVec; vecOpt.*vecScl];
vecScl = vecInit;

%options=optimset( 'LevenbergMarquardt','on', 'MaxFunEvals',99,  'TolFun',1e-4, 'LargeScale','on', 'TolX',1e-4, 'Display','iter');
%options=optimset( options, 'Display','iter');

disp('MirrorPosFocalPpErr')
resInit = MirrorPosFocalPpErr( ones(size(vecScl)), vecScl, polyBoundaryVecCell ); EvalPrint('rms(resInit)');
if (doLm > 0) ||  (  (doFinalLm > 0) && (numPoses == 1)  )
    
    %save
    %vecOptSng = fminsearch('MirrorPosFocalPpErr', ones(size(vecScl)), [], [], options, vecScl, polyBoundaryVecCell );
    %vecOptSng = lsqnonlin('MirrorPosFocalPpErr', ones(size(vecScl)), [], [], options, vecScl, polyBoundaryVecCell );
    
    %X = fminsearch(@(x) f(x,c),[0.3;1])
    
    %%save('temp.mat', 'polyBoundaryVecCell','vecScl')
    % MirrorPosFocalPpErrOPT(); %call with 0==nargin to reset persistent vars. This is needed when we are in a loop
    % vecOptSng=LMFsolve('MirrorPosFocalPpErrOPT',vecInitLm(:)',options);
    
    
    %vecScl = vecScl(:).';
    
    
    %save
    %vecOptSng=LMFsolve(@(x) MirrorPosFocalPpErr(x(:),vecScl, polyBoundaryVecCell),ones(size(vecScl)),options);
    
    
    vecOptSng=LMFnlsq(@(x) MyVectorize(MirrorPosFocalPpErr(x(:),vecScl, polyBoundaryVecCell)),ones(size(vecScl)),options);
    
else
    vecOptSng = ones(size(vecScl));
end
[resOpt, viewVecCell, allViews, normalAngle, RR_Rec, wSolnVec] = MirrorPosFocalPpErr( vecOptSng, vecScl, polyBoundaryVecCell );
EvalPrint('rms(resOpt)');
vecOptSng = vecOptSng.*vecScl;


if numPoses > 1
    
    [sclVec, moveAlongJoinVec, sclArray, moveAlongJoinArray] = ...
        ApproxMergeScaleAndPos( polyBoundaryVecCell, allViews, normalAngle, RR_Rec, wSolnVec );
    
    
    vecInit = [moveAlongJoinVec; sclVec];
    vecScl = vecInit;
    disp('MirrorMergePosErr')
    [resInit, mergedViewVec] = MirrorMergePosErr( ones(size(vecScl)), vecScl, polyBoundaryVecCell, allViews, normalAngle, RR_Rec, wSolnVec );    EvalPrint('rms(resInit)');
    
    if doLm > 0
        
        %vecOptPos = lsqnonlin('MirrorMergePosErr', ones(size(vecScl)), [], [], options, ...
         %   vecScl, polyBoundaryVecCell, allViews, normalAngle, RR_Rec , wSolnVec);
         
         
         vecOptPos=LMFnlsq(...
             @(x) MyVectorize(MirrorMergePosErr(x(:), vecScl, polyBoundaryVecCell, allViews, normalAngle, RR_Rec , wSolnVec )),...
             ones(size(vecScl)), options);
         
    else
        vecOptPos = ones(size(vecScl));
    end
    
    resOpt = MirrorMergePosErr( vecOptPos, vecScl, polyBoundaryVecCell, allViews, normalAngle, RR_Rec , wSolnVec); EvalPrint('rms(resOpt)');
    vecOptPos = vecOptPos.*vecScl;
    
    vecInit = [vecOptPos; vecOptSng];
    vecScl = vecInit;
    disp('StaticMirrorObjMoveCamErr')
    resInit = StaticMirrorObjMoveCamErr( ones(size(vecScl)), vecScl, polyBoundaryVecCell );
    EvalPrint('rms(resInit)');
    
    if doFinalLm > 0
        %vecOpt = lsqnonlin('StaticMirrorObjMoveCamErr', ones(size(vecScl)), [], [], options, vecScl, polyBoundaryVecCell );
        
        vecOpt=LMFnlsq(@(x) MyVectorize(StaticMirrorObjMoveCamErr(x(:), vecScl, polyBoundaryVecCell)), ones(size(vecScl)),options);
        
    else
        vecOpt =  ones(size(vecScl));
    end
    
    [resOpt, mergedViewVec] = StaticMirrorObjMoveCamErr( vecOpt, vecScl, polyBoundaryVecCell );
    EvalPrint('rms(resOpt)');
    
    vecOpt = vecScl .* vecOpt;
    
else
    mergedViewVec = viewVecCell{1};
    vecOpt = vecOptSng;
end

% if nargout > 1
%     whos vec*
%     save
%
% end
