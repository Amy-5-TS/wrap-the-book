function [ bestH2to1, inliers] = computeH_ransac( locs1, locs2)
  %COMPUTEH_RANSAC A method to compute the best fitting homography given a
  %list of matching points.
  total = size(locs1,1);
  threshold=20;
  numSam=4;
  e=0.7;
  prob=0.99;
  time = round(log10(1-prob)/log10(1-(1-e)^numSam)); 
  maxNumInliers=0;
  inliers=zeros(total);
  maison=size(locs1,1);
  addit=ones(maison,1);%addict is a Nx1 vector and locs1 is a Nx2 matrix
  
  for i=1:time
      
      %compute the current corresponding H
      %H=computeH(locs1,locs2);
      h2to1 = computeH_norm(locs1,locs2);
      % apply the homography transformation to locs1 and compute the error
      % to apply the transform, first need to add an extra dimension to Nx2
      
      %alert: keep the locs1 and locs2 unchanged!
  
      
      
 %%%%%%%%%%%%%%%%%version one is compare the gradient difference of point pairs%%%%%%%%%
 %%%%%%%%%%%%%%5%%version two is do it in 8-dimensional space%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%         version 1      %%%%%%%%%%%%%%%%%%%%%%
      conc=[locs2 addit];%Nx3
      fakelocs1=transpose(h2to1*transpose(conc));% Nx3 * 3x3 = Nx3 still need to abandon the 3rd column
      fakelocs1(:,1)=fakelocs1(:,1)./fakelocs1(:,3);
      fakelocs1(:,2)=fakelocs1(:,2)./fakelocs1(:,3);
      fakelocs1(:,3)=fakelocs1(:,3)./fakelocs1(:,3);
      fakelocs1(:,3)=[];% now it is a Nx2 matrix again
      
      %compute the gradient difference and count the inliers
      contTotalIn=0;
      coninliers=zeros(total,1);
      diffe=zeros(total,1);
      for k=1:total
        diffe(k,1)=sqrt((fakelocs1(k,1)-locs1(k,1))^2+(fakelocs1(k,2)-locs1(k,2))^2);
        if diffe(k,1)<threshold
          contTotalIn=contTotalIn+1;
          coninliers(k,1)=1;
        end
      end 
      %%%%%%%%%%%%%%%%%%%%%%  end of v1   %%%%%%%%%%%%%%%%%%%%%
%  %%%%%%%%%%%%%%%%%%%%%%%    version 2   %%%%%%%%%%%%%%%%%%%%%
%       %compute the gradient difference and count the inliers
%       contTotalIn=0;
%       coninliers=zeros(total,1);
%       vector1=[];
%       vector2=[];
%     for i=1:size(locs1,1)
%         a1=locs2(i,1);
%         b1=locs2(i,2);
%         a2=locs1(i,1);
%         b2=locs1(i,2);
%         vector1=[vector1; -a1   -b1   -1   0   0   0   a1*a2   b1*a2   a2]; %Nx9
%          vector2=[vector2; 0    0    0   -a1   -b1   -1   a1*b2   b1*b2  b2];
%     end
%      h=transpose(reshape(transpose(h2to1),[],9));
%       for k=1:total
%           
%         if vector1(k,:)*h<threshold 
%           contTotalIn=contTotalIn+1;
%           coninliers(k,1)=1;
%         end
%         if vector2(k,:)*h<threshold
%           contTotalIn=contTotalIn+1;
%           coninliers(k,1)=1;
%         end
%       end 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%   end of v2  %%%%%%%%%%%%%%%%
    
       if contTotalIn>maxNumInliers
        maxNumInliers=contTotalIn;
          inliers=coninliers;
          bestH2to1=h2to1;
      end
  end
  disp("final result");
  disp(fakelocs1);
  disp(locs1);
  disp(bestH2to1);
end
  
  %Q2.2.3

  
  