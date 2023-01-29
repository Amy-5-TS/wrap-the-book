function [H2to1] = computeH_norm(x1, x2)

total= size(x1,1);
%% Compute centroids of the points
c1 = [(sum(x1(:,1))/size(x1,1)),(sum(x1(:,2))/size(x1,1))];%size(a,1)#column
c2 = [(sum(x2(:,1))/size(x2,1)),(sum(x2(:,2))/size(x2,1))];
%% Shift the origin of the points to the centroid

n1(:,1)=x1(:,1)-c1(1);
n1(:,2)=x1(:,2)-c1(2);
n2(:,1)=x2(:,1)-c2(1);
n2(:,2)=x2(:,2)-c2(2);
%% Normalize the points so that the average distance from the origin is equal to sqrt(2).

dis1=sqrt(n1(:,1).^2+n1(:,2).^2);
dis2=sqrt(n2(:,1).^2+n2(:,2).^2);
mean1=mean(dis1);
mean2=mean(dis2);
scale1=sqrt(2)/mean1;
scale2=sqrt(2)/mean2;

% for the original Nx2 matrix x1 and x2, add an additional 1 column to the end-> Nx3
addit=ones(total,1);
n1=[n1 addit];
n2=[n2 addit];
%% similarity transform 1
t1=[scale1   0   -scale1*c1(1)
     0     scale1 -scale1*c1(2)
     0       0      1      ];


%% similarity transform 2
t2=[scale2   0   -scale2*c2(1)
     0     scale2 -scale2*c2(2)
     0       0      1      ];


n1=transpose(t1*transpose(n1));
n2=transpose(t2*transpose(n2));
n1(:,1)=n1(:,1)./n1(:,3);
n1(:,2)=n1(:,2)./n1(:,3);
n2(:,1)=n2(:,1)./n2(:,3);
n2(:,2)=n2(:,2)./n2(:,3);
n1(:,3)=[];
n2(:,3)=[];%n1 and n2 are new points after normalizing the coordinates

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%randomly choose 4 points and compute homography
randindx =transpose(randperm(total,4));
p1 = n1(randindx,:);
p2 = n2(randindx,:);
%in case there is a NaN
while (abs(p1(1,1)) < 1e-3 )||(abs(p1(1,2)) < 1e-3 )||(abs(p2(1,1)) < 1e-3 )||(abs(p2(1,2)) < 1e-5 )||   (abs(p1(2,1)) < 1e-3 )||(abs(p1(2,2))< 1e-3 )||(abs(p2(2,1)) < 1e-3 )||(abs(p2(2,2)) < 1e-3 )||    (abs(p1(3,1)) < 1e-3 )||(abs(p1(3,2)) < 1e-3 )||(abs(p2(3,1)) < 1e-3 )||(abs(p2(3,2)) < 1e-3 )   ||(abs(p1(4,1)) < 1e-3 )||(abs(p1(4,2)) < 1e-3 )||(abs(p2(4,1)) < 1e-3 )||(abs(p2(4,2)) < 1e-3 )
     randindx = transpose(randperm(total,4));
     p1 = n1(randindx,:);
     p2 = n2(randindx,:);
end


%% Compute Homography
H= computeH( p1, p2 );
% ransac->norm and disp(H)
H2to1 = inv(t1)*H*t2;  %please convert to homogeneous coordinate before using
end



% I11 = find(n1(:,1) < 1e-5);
% I12 = find(n1(:,2) < 1e-5);
% I21 = find(n2(:,1) < 1e-5);
% I22 = find(n2(:,2) < 1e-5);

% randindx =transpose(randperm(total,4));

% while (size(intersect(I11,randindx))~=0& size(intersect(I12,randindx))~=0&size(intersect(I21,randindx))~=0&size(intersect(I22,randindx))~=0)
%      randindx =transpose(randperm(total,4));
% end


% disp("randindx");
% disp(randindx);
% p1 = n1(randindx,:);
% p2 = n2(randindx,:);