%rotate 4lys peptide to be parallel to the box surface
function main
clc
clear
pos=[17.949137	6.624518	15.878347;
20.40765	6.966542	16.072707;
19.521746	6.094128	16.92084;
19.458882	7.519898	15.002279;
21.031566	8.024403	16.84157;
21.439631	6.127304	15.385761]

%choose three points to form three axis
point2=[ 17.949137   6.624518  15.878347]';
point1=[ 20.407650   6.966542  16.072707]';
point3=[ 21.439631   6.127304  15.385761]';

 e1=(point2-point1)/norm(point2-point1);%X
 e2=(point3-point1)/norm(point3-point1);
 e2=-cross(e1,e2)/norm(cross(e1,e2));%Y
 e3=cross(e1,e2)/norm(cross(e1,e2));%Z

frame_b=[e3,e2,e1] %first frame

%second frame
% point1=[17.106863   9.762878   8.366100]';
% point2=[16.047911   9.432998   9.609405]';
% point3=[16.151389   9.939033   8.695341]';
% 
%  e1=(point2-point1)/norm(point2-point1);%X
%  e2=(point3-point1)/norm(point3-point1);
%  e2=cross(e1,e2)/norm(cross(e1,e2));%Y
% e3=cross(e1,e2)/norm(cross(e1,e2));%Z

%frame_a=[e1,e2,e3]

frame_a=eye(3);

Rot=inv(rot(frame_a,frame_b)) %from b to a, rotation matrix

%rotate all atoms
pos_2=(Rot*pos')';

%translate
  pos_2(:,1)=pos_2(:,1)+30;
  pos_2(:,2)=pos_2(:,2)+15;
  pos_2(:,3)=pos_2(:,3)+ 35;
 pos_2

%csvwrite('kaaa_rot.csv',pos_2')
%rotate to be parallel
 end
%=========================
function Rot=rot(frame_a,frame_b)
%frame_a or _b contains the 3 unit vectors in column
%rotate from b to a
Rot=frame_a'*frame_b;
end

