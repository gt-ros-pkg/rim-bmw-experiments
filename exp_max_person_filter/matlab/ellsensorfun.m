clear
ell_c = [300.0; 200.0];
ell_r = 0;
ell_scale = [100.0, 50.0];
ell_coeffs = 1 ./ ell_scale.^2;
ell_R = [cos(ell_r), -sin(ell_r);
         sin(ell_r),  cos(ell_r)];
ell_shape = diag(1 ./ ell_scale) * ell_R;
sigma = 0.1;

cam_pt = [100; 300];
cam_pt_ell_frame = cam_pt - ell_c;
[tangent1, tangent2] = ellipse_tangent_fun(ell_scale, cam_pt_ell_frame)

% pts = [ell_c, ell_c, ell_c+30];
imgx = 600;
imgy = 600;
[ptsx ptsy] = meshgrid(1:imgx, 1:imgy);
pts = [ptsx(:)'; ptsy(:)'];
pts_trans = bsxfun(@minus, pts, ell_c);
pts_rot_trans = ell_R * pts_trans;
pts_dists = ell_coeffs(1) * pts_rot_trans(1,:).^2 + ell_coeffs(2) * pts_rot_trans(2,:).^2 - 1;
% pts_rot_trans = ell_shape * pts_trans;
% pts_dists = pts_rot_trans(1,:).^2 + pts_rot_trans(2,:).^2 - 1;
pts_weights = exp(-pts_dists.^2/sigma^2);
in_between = vector_between_fun(tangent1, tangent2, pts_rot_trans);
pts_weights(~in_between) = 0;
img_weights = reshape(pts_weights, imgx, imgy);
figure(22)
clf
imagesc(img_weights)
hold on
plot(cam_pt(1),cam_pt(2), 'rx')
