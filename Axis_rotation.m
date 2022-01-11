% Rotation a 3D coordinate system (molecular structure, frame etc.) around 
% a chosen axis. 
%
% Chunhua Ying     Date: 01/11/2022
%
%---------------------Reading and Running Guidance-------------------------
%1. Properly define axis vector, initial coordinates, and rotating angle
%
%2. To keep the frame at same position, proper displacement is needed after
%   rotation, you can do two ways:
%   1) define any displacement vector you want.
%   2) define displacement in a way such that certain point is fixed after
%      rotation.
%--------------------------------------------------------------------------

function main
 Rotation
end


function Rotation
clc
clear
% define rotation axis
e1 = [7.200500;  22.428300;  15.584400]; % 1st point along rotating axis
e2 = [7.120400;  19.967700;  15.370200]; % 2nd point along rotating axis
e = e1-e2 % rotating axis vector
Rot = EXPR(e) %rotation matrix from e

% initial 3D coordinate system
system0 = [5.9862   19.5251   17.9355	;
7.1921	21.1296	16.2518	;
4.5972	19.6101	15.7388	;
3.9287	18.1351	17.1987	;
3.5226	20.2516	17.5293	;
9.7662	21.0702	15.9711	;
9.0749	22.0308	17.8024	;
9.0047	19.8555	17.6138	;
6.8277	18.5156	17.2994	;
5.6134	19.2539	19.321	;
7.2005	22.4283	15.5844	;
7.1204	19.9677	15.3702	;
6.336	21.0409	17.5922	;
4.4519	19.3793	17.0671	;
8.8182	21.0168	16.9396	;
]

% rotating
system1 =(Rot*system0')' % initial coordinates after rotation
displacement1 = system0(12,:)-system1(12,:); % define dis. s.t. No.12 point fixed
displacement2 = [1 0 -0.3]; % user defined displacement
system3 = system1 + displacement1+displacement2

end
function Rot=EXPR(e)
x=e/norm(e);
theta=0.2*norm(e);
xhat=SKEW3(x);
Rot=expm(xhat*theta);
end

function vhat=SKEW3(v)
vhat=zeros(4);
vhat=[0,-v(3),v(2);v(3),0,-v(1);-v(2),v(1),0];
end 