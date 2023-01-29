function [ locs1, locs2] = matchPics( I1, I2 )
%MATCHPICS Extract features, obtain their descriptors, and match them!

%% Convert images to grayscale, if necessary
if (ndims(I1) == 3)
    I1 = rgb2gray(I1);
end
if (ndims(I2) == 3)
    I2 = rgb2gray(I2);
end

I1 = double(I1) / 255;
I2 = double(I2) / 255;
%% Detect features in both images
threshold1=0.12;
coords1=fast_corner_detect_9(I1, threshold1);
coords2=fast_corner_detect_9(I2, threshold1);
%% Obtain descriptors for the computed feature locations
[desc1, L1] = computeBrief(I1, coords1);
[desc2, L2] = computeBrief(I2, coords2);
%% Match features using the descriptors
threshold2=20;
indexPairs = customMatchFeatures(desc1, desc2, threshold2);
locs1 = L1(indexPairs(:, 1), :);
locs2 = L2(indexPairs(:, 2), :);
%show the pic
customShowMatchedFeatures(I1, I2, locs1, locs2);
disp("points in loc1");
disp(locs1);
disp("points in loc2");
disp(locs2);
end

    