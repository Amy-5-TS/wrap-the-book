function [ composite_img ] = compositeH( H2to1, template, img )
%COMPOSITE Create a composite image after warping the template image on top
%of the image using the homography

% Note that the homography we compute is from the image to the template;
% x_template = H2to1*x_photo
% For warping the template to the image, we need to invert it.

H_template_to_img = inv(H2to1);

%% Create mask of same size as template
mask=true(size(template));%size as the desk

%% Warp mask by appropriate homography
wrap_mask=warpH(mask,H_template_to_img,size(img));

%% Warp template by appropriate homography
img_wrap=warpH(template,H_template_to_img,size(img));

%% Use mask to combine the warped template and the image
mask_the_desk=true(size(img));
mask_the_desk=mask_the_desk-wrap_mask;% in the size of img(desk), but the place where hp cover shows is nw masked(corresponding value=0), and the other places not covered by hp_cover is 1 in the mask matrix
maskedRgbImage = bsxfun(@times, img, cast(mask_the_desk, 'like', img));
composite_img=maskedRgbImage+img_wrap;
end