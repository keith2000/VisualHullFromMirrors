function R=AlignVectors(v1,v2)

% R=AlignVectors(v1,v2)
% provides the rotation matrix to align v2 with v1 i.e. v1==R*v2
% rotation is in the plane defined by the two vectors

%ensure unit vectors:
v1=v1./norm(v1);
v2=v2./norm(v2);

ensure('all(size(v1)==[3,1])');
ensure('all(size(v2)==[3,1])');

cosTheta=dot(v1,v2);


if (abs(cosTheta)<0.9999999)
    
    w=cross(v2,v1);
   % disp(w)
    sinTheta=norm(w);
    
    
    n=w./norm(w); %unit vector defining axis of rotation
    
    
    
    
    %disp(asin(sinTheta))
    %disp(acos(cosTheta))
    
    %norm(n)
    
    Sn=  [    0, -n(3),  n(2);...
            n(3),      0, -n(1);...
            -n(2),  n(1),     0 ];
    
    R = cosTheta*eye(3) + sinTheta*Sn + (1-cosTheta)*(n*n');
    
    if 0
        close all
        figure(1)
        axis vis3d
        hold on
        plot3d([[0;0;0],v1],'r');
        plot3d([[0;0;0],v2],'g');
        plot3d([[0;0;0],n],'b');
        plot3d([[0;0;0],R*v2],'m*');
        plot3d(v1,'r.');
        plot3d(v2,'g.');
        plot3d(n,'b.');
    end
    
    
    if (abs(dot(v1,R*v2)-1)>1e-7)
        error('could not rotate to align')
    end
    
else
    
    if (cosTheta<0)
        R=[...
                [0;-1;0],...
                [-1;0;0],...
                [0;0;-1]...
            ];
    else
        R=eye(3);    
    end
    
end


%Now, I may be misunderstanding you here, so forgive me if I have.
% What I'm about to write is a formula for calculating the 3D
% rotation matrix (in 3x3 matrix format), for the of an arbitrary
% angle theta around the axis specified by the _unit_ column vector
% n = [nx ny nz]'.
% 
% Thus, to use this, you need to be able to calculate the axis of
% rotation, and the angle of rotation.  Let's go...
% 
% (BTW This is based on something called Rodrigues' formula)
% 
% Construct the matrix Sn = [  0 -nz  ny]
%                          [ nz   0 -nx]
%                          [-ny  nx   0]
% 
% Then, find R by using
% 
% R = cos(theta)*I + sin(theta)*Sn + (1-cos(theta))*n.n'
% 
% where I is a 3x3 identity matrix, and n.n' means the product of n
% and n transpose (which will be a 3x3 matrix as well).
%
%With this formula it is easy to generate the cases of rotation
%around the coordinate system axes by substituting the appropriate
%vector for n.
%
%There is a good introductory discussion on this type of thing in
%the Appendix of "A Framework for Uncertainty and Validation of 3-D
%Registration Methods based on Points and Frames", International
%Journal of Computer Vision 25(3), 203-229, 1997.  Kluwer Academic
%Publishers.
%
%Anyway, I hope this is useful.
%
%Cheers,
%
%John
%--
%          ,---------------------------------------.
%          | John Williams                         |
%          | Space Centre for Satellite Navigation |
%          | QUT, Brisbane, Australia              |
%          | Email: ja.williams@student.qut.edu.au |
%          | Ph   : (+61 7) 3864 2458              |
%          | Fax  : (+61 7) 3864 1361              |
%          `---------------------------------------'


