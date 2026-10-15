function R = eul2rotm2(z, y, x)
% ZYX
% Rz = [cos(a(1)) -sin(a(1)) 0; 
%     sin(a(1)) cos(a(1)) 0; 
%     0 0 1];
% Ry = [cos(a(2)) 0 sin(a(2)); 
%     0 1 0;
%     -sin(a(2)) 0 cos(a(2))];
% Rx = [1 0 0;
%     0 cos(a(3)) -sin(a(3)); 
%     0 sin(a(3)) cos(a(3))];
% Rx, Ry, Rz
% R = Rz*Ry*Rx;
c1 = cos(z);
c2 = cos(y);
c3 = cos(x);
s1 = sin(z);
s2 = sin(y);
s3 = sin(x);

R = [c1*c2, c1*s2*s3 - c3*s1, s1*s3 + c1*c3*s2;
     c2*s1, c1*c3 + s1*s2*s3, c3*s1*s2 - c1*s3;
     -s2,   c2*s3,            c2*c3           ];
end