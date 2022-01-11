%ME579_HW3   
%Student: Chunhua Ying     Date: 10/5/2020
%---------------------Reading and Running Guidance-------------------------
%1. Each problem has a subroutine, uncomment the 'problem' in 
%   the 'main' can run the corresponding problem. 
%2. For detailed information, one can just go to that subroutine and check 
%   subproblem coding. Results can be run to be seen and descriptive answer 
%   are as commented in the coding after each subproblem.
%3. All the functions are coded within this program except the provided 
%   ploting function.
%4. For compact and brief consideration, each subproblem has been plotted
%   in just one figure within the problem and specific label is clarified 
%   in the code after plotting function. The initial points and vectors 
%   used for each problem are same for better understanding.
%       point1=[1;1;2] & point2=[1;2;1], vector=point2-point1
%--------------------------------------------------------------------------
function main
%to run, uncommet corresbonding problem, also can be run simultaneously.
% P1         
% P2
% P3
% P4
% P5
% P6
% P7
% P8
 P9
end

%P1 roll about X axis
function P1
clear all
clc
figure(1)
%P1
phi=pi/2; %rotating 90 degree around x axis
colorspecx = {'r','k','m'}; %rotated axis, x-red, y-black, z-magenta
Rx=ROLLR(phi); %rotation matrix
%in order for ploting in 3D, part a) has been moved to right after part b)

%(b)
point1=[1;1;2];  %first point
point2=[1;2;1];  %second point
point1t=Rx*point1; %rotated first point
point2t=Rx*point2; %rotated second point
plotp3 (point1, 'g') %first point before, green color
plotp3 (point1t, 'b') %first point after, blue color
plotp3 (point2, 'g') %second point before, green color
plotp3 (point2t, 'b') %second point after, blue color
%%Answer: from plot, one can see 2 points has been rotated around x axis
%%with 90 degree. 
%==========

%(a)
plotr (Rx, colorspecx) %plot rotated axis
%%Answer: the rotated axis can also been seen in the figure, with x-red, 
%%y-black, z-magenta. one can see the x axis remain same, while z and y has
%%been rotated 90 degree
%==========

%(c)
vector1=point2-point1; %use the 2 points defined before to form a vector
vector1t=Rx*vector1;   %rotated vector
plotv3 (point1, point2, 'g') %vector before, green
plotv3 (point1t, point2t, 'b') %vector after, after
%%Answer: one can see the green vector has been rotated 90 degree around x
%%axis to the blue vector.
%==========
grid on
axis([-2 2 -2 2 -2 2])
end
function Rx=ROLLR(phi) 
Rx=eye(3); 
Rx(2,2)=cos(phi);
Rx(3,3)=cos(phi);
Rx(2,3)=-sin(phi);
Rx(3,2)=sin(phi);
end

%P2 pitch about Y axis
function P2
clear all
clc
figure(2)
%P2
theta=pi/2; %rotated 90 degree
colorspecx = {'r','k','m'};
Ry=PITCHR(theta); %pitch rotation matrix

%(b)
grid on
point1=[1;1;2]; %first point
point2=[1;2;1]; %second point
point1t=Ry*point1;  %first point rotated
point2t=Ry*point2;  %second point rotated
plotp3 (point1, 'g') %first point before, green
plotp3 (point1t, 'b') %first point after, blue
plotp3 (point2, 'g') %second point before, green
plotp3 (point2t, 'b') %second point after, blue
%%Answer: in the plot, one can see these 2 points has been rotated around Y
%%axis 90 degree to blue points.
%==========

%(a)
plotr (Ry, colorspecx) %plot rotated axis
%%Answer: the rotated axis can also been seen in the figure, with x-red, 
%%y-black, z-magenta. one can see the y axis remain same, while z and x has
%%been rotated 90 degree
%==========

%(c)
vector1=point2-point1; %use the 2 points defined before to form a vector
vector1t=Ry*vector1;   %rotated vector
plotv3 (point1, point2, 'g') %vector before, green
plotv3 (point1t, point2t, 'b') %vector after, after
%%Answer: one can see the green vector has been rotated 90 degree around y
%%axis to the blue vector.
grid on
axis([-2 2 -2 2 -2 2])
end
function Ry=PITCHR(theta)
Ry=eye(3);
Ry(1,1)=cos(theta);
Ry(3,3)=cos(theta);
Ry(1,3)=sin(theta);
Ry(3,1)=-sin(theta);
end

%P3 yaw about Z axis
function P3
clear all
clc
figure(3)
%P3
ksi=pi/2; %rotate 90 degree
colorspecx = {'r','k','M'};
Rz=YAWR(ksi); %Yaw rotation matrix

%(b)
point1=[1;1;2]; %first point
point2=[1;2;1]; %second point
point1t=Rz*point1;  %first point rotated
point2t=Rz*point2;  %second point rotated
plotp3 (point1, 'g') %first point before, green
plotp3 (point1t, 'b') %first point after, blue
plotp3 (point2, 'g') %second point before, green
plotp3 (point2t, 'b') %second point after, blue
%%Answer: in the plot, one can see these 2 points has been rotated around Z
%%axis 90 degree to blue points.
%(a)
plotr (Rz, colorspecx)
%%Answer: the rotated axis can also been seen in the figure, with x-red, 
%%y-black, z-magenta. one can see the z axis remain same, while y and x has
%%been rotated 90 degree
%(c)
vector1=point2-point1; %use the 2 points defined before to form a vector
vector1t=Rz*vector1;   %rotated vector
plotv3 (point1, point2, 'g') %vector before, green
plotv3 (point1t, point2t, 'b') %vector after, after
%%Answer: one can see the green vector has been rotated 90 degree around z
%%axis to the blue vector.
grid on
axis([-2 2 -2 2 -2 2])
end
function Rz=YAWR(ksi)
Rz=eye(3);
Rz(1,1)=cos(ksi);
Rz(2,2)=cos(ksi);
Rz(1,2)=-sin(ksi);
Rz(2,1)=sin(ksi);
end

%P4 roll&pitch&yaw
function P4
clear all
clc
figure(4)
EulerAngle=[pi/4,pi/4,pi/4]; %input roll, pitch, yaw angle
Rxyz=RPYR(EulerAngle);     %rotation matrix
colorspecx={'r','k','m'};  
%%y-black, z-magenta
%%Answer: one can see the rotated axis in the figure, this result make 
%%sence when rotate 45 degree around x,y,z axis.

point1=[1;1;2]; %first point
point2=[1;2;1]; %second point
point1t=Rxyz*point1  %first point rotated
point2t=Rxyz*point2  %second point rotated
plotp3 (point1, 'g') %first point before, green
plotr (Rxyz, colorspecx)  %plot rotated axis, with x-red, y-black, z-magenta
%%Answer: one can see the rotated axis in the figure, this result make 
%%sence when rotate 45 degree around x,y,z axis.
plotp3 (point1t, 'b') %first point after, blue
plotp3 (point2, 'g') %second point before, green
plotp3 (point2t, 'b') %second point after, blue

vector1=point2-point1; %use the 2 points defined before to form a vector
vector1t=Rxyz*vector1;   %rotated vector
plotv3 (point1, point2, 'g') %vector before, green
plotv3 (point1t, point2t, 'b') %vector after, after

grid on
axis([-2 2.5 -2 2.5 -2 2])
end
function Rxyz=RPYR(EulerAngle)
EA=EulerAngle;
Rxyz=YAWR(EA(1))*PITCHR(EA(2))*ROLLR(EA(3));
end

%P5 homogeneous transformation
function P5
clear
clc
figure(5)
colorspec={'r','g','b'};
ksi=[1;0 ; 0; 0;pi/2; 0]; %input [x, y, z, roll, pitch, yaw]
g=XF(ksi)     %homogeneous matrix
plotf (g, colorspec) %plot transformed axis, x-red,y-green,z-blue
point1=[1;1;2;1] %point 1 before transform
point1t=g*point1 %point1 transformed
plotp3 (point1(1:3), 'g')  %point 1 before transform, green
%plotr(g(1:3,1:3))     %plot transformed axis
plotp3 (point1t(1:3), 'b') %point 1 after transform, green

%%Answer: one can see in the figure, the transformed axis has been rotated
%%around Y with 90 degree and move along x direction with 1 unit. The
%%transformed point also makes sense in the figure.
grid on
axis([-2 3.2 -2 2 -2 2])
end
function g=XF(ksi)
g=eye(4);
Rxyz=RPYR(ksi(4:6));
g(1:3,1:3)=Rxyz;
g(1:3,4)=ksi(1:3);

end

%P6 inv of homogeneous transformation
function P6
clear
clc
figure(6)
colorspec={'r','g','b'};
ksi=[1; 0; 0; 0;pi/2; 0]; %input [x, y, z, roll, pitch, yaw]
g=XF(ksi)  %homogeneous matrix
ginv=FINV(g); %inverse of homogeneous matrix
plotf (ginv, colorspec)  %plot transformed axis, x-red,y-green,z-blue
point1=[1;0;0;1]; %point 1 
point1t=g*point1; %point 1 transformed forward, g
point1tinv=ginv*point1t %transformed point 1 transform back, ginv
plotp3 (point1(1:3), 'g') %point 1 , green
plotp3 (point1t(1:3), 'b') %point 1 transformed forward, blue
plotp3 (point1tinv(1:3), 'k') %transformed point 1 transform back, black

%%Answer, one can see only black and blue point in the figure, that is
%%because green has been overlaped with black, they are same points, which
%%is corrct since ginv rotate the points back; The roated axis also make
%%sense.


grid on
axis([-2 2 -2 2 -2 2])
end
function ginv=FINV(g)
ginv=eye(4);
ginv(1:3,1:3)=g(1:3,1:3)';
ginv(1:3,4)=-g(1:3,1:3)'*g(1:3,4);
ginv(4,4)=1;
end

%P7 skew symmetric matrix
function P7
clear
clc
v=[1,2,1]; %input 3*1 vector
vhat=SKEW3(v) %skew tranform to R3
end
function vhat=SKEW3(v)
vhat=zeros(4);
vhat=[0,-v(3),v(2);v(3),0,-v(1);-v(2),v(1),0];
end 

%P8 some common properties
function P8
clear
clc
p=[1;1;1] %p vector chosen
%a)
theta=norm(p) %norm pf vector p

%b)
pbar=p/theta  %unit vector along p

%c)
Jp1=SKEW3(p)  %J(p)
Jp2=SKEW3(pbar)*theta  %J(pbar)theta

%d)
Jp1*p  %J(p)*p=0=0*p, thus p is in the kernel of J(p)

%e)
[vecs vals] = eig(SKEW3(p))
%%Answer:Relationship between p and the eigenvectors
%%and eigenvalues of J(p): in R space, skew3(p) only has eigen-
%%value 0 and corresponding eigen-vector p.

%f)
expJp1=expm(Jp1)  %method 1 EXPM()
expJp1_2=real(vecs*diag(exp(diag(vals)))/vecs) %method 2 diagonalization
%%Answer:From the result, we can see they are equal within floating-point
%%precision

%g)
p-expJp1*p
%%Answer: the subtraction is zeros, thus they are equal

%h)
[vecs vals] = eig(expm(SKEW3(p)))
%%Answer:expJp1*p=p, p is an eigenvector of expJp1 with eigenvalue 1
end

%P9 rotation around arbitrary vector
function P9
clc
clear
figure(9)
e1=[9.047888;  22.377979;  15.673100];
e2=[9.704748;  20.177170;  15.238893];
e=e2-e1 %input [e1 e2 e3]T
Rot=EXPR(e) %rotation matrix from e

%a)
colorspec={'r','g','b'};
%plotr (Rot, colorspec)  %transformed axis
%plotr (eye(3), colorspec) %original axis
%%Answer: the original axis is just the frame axis on which the rotation axis
%%plotted.

%b)
%point1=[1;2;3]; %first point
%point2=[3;2;1]; %second point
pf6=[  7.461269  20.756949  15.202408;
       9.047888  22.377979  15.673100;  
       7.612847  21.624961  17.363336;
       9.704748  20.177170  15.238893; 
       8.270298  19.415281  16.926055;
       9.872681  21.045453  17.390302;
       8.638368  20.884422  16.361788;]
   
pf6_r=(Rot*pf6')'+[28 8 -1]
%point1t=Rot*point1; %first point rotated
%point2t=Rot*point2; %second point rotated


%plotv3(point1,point2,'k')
%plotv3(point1t,point2t,'y')
%plotr (Rot, colorspec)
%%Answer: black line is the vector before been rotated, yellow line is the
%%vector after been roated


%c)
%Roll axis
e1=[1;0;0];
ang=pi/2;
vector=e1*ang;
Rot=EXPR(vector);
Rot-ROLLR(ang);

%Pitch axis
e1=[0;1;0];
ang=pi/2;
vector=e1*ang;
Rot=EXPR(vector);
Rot-PITCHR(ang);

%Pitch axis
e1=[0;0;1];
ang=pi/2;
vector=e1*ang;
Rot=EXPR(vector);
Rot-YAWR(ang);

%%Answer: 

%%1. the corresponding axis-angle matrix are as above, compared to the
%%matrix in previous Roll, Pitch, Yaw matrix, they are equal respectively.

%%2. The axis given to EXPR() function is the unit vector around which Euler
%%angle matrix rotated about, and the angle input is actually the angle
%%value Euler angle matrix rotated.
end
function Rot=EXPR(e)
x=e/norm(e);
theta=0.7*norm(e);
xhat=SKEW3(x);
Rot=expm(xhat*theta);
end
