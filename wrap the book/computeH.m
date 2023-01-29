function [ H2to1 ] = computeH( x1, x2 )
%COMPUTEH Computes the homography between two sets of points

A=[];
for i=1:size(x1,1)
    a1=x1(i,1);
    b1=x1(i,2);
    a2=x2(i,1);
    b2=x2(i,2);
    A = [A;  -a2   -b2   -1   0   0   0   a2*a1   b2*a1   a1; 
             0    0    0   -a2   -b2   -1   a2*b1   b1*b2  b1];
    
end
[u,d,v] = svd(A);
h=v(:,end);

H2to1 = transpose(reshape(h,3,3));
H2to1=H2to1./H2to1(3,3);
end
