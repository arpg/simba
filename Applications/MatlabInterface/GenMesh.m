function [mesh] = GenMesh(tau, granularity, scale, smoothness, is_recorded)
% This function gives us a detailed mesh to simulate over.
% It's an easy way to generate a heightmap from our tau set of bits.
% Ordering of the mesh:
% [00 01 02]
% [10 11 12]
% [20 21 22]
% We always assume that the car starts at (0, 0). There's a compensating term 
% on the x and y axes to make sure that, no matter what the scale, we can start
% at the origin safely. 
% Smoothness goes from 0 - 3, with 3 being the smoothest map.

[X, Y] =  meshgrid(0:2, 0:2);
z00 = bi2de(bitget(tau, 1:2), 'left-msb');
z01 = bi2de(bitget(tau, 3:4), 'left-msb');
z02 = bi2de(bitget(tau, 5:6), 'left-msb');
z10 = bi2de(bitget(tau, 7:8), 'left-msb');
z11 = bi2de(bitget(tau, 9:10), 'left-msb');
z12 = bi2de(bitget(tau, 11:12), 'left-msb');
z20 = bi2de(bitget(tau, 13:14), 'left-msb');
z21 = bi2de(bitget(tau, 15:16), 'left-msb');
z22 = bi2de(bitget(tau, 17:18), 'left-msb');
Z = real([z00 z01 z02; z10 z11 z12; z20 z21 z22])/4;
[yy, xx] =  meshgrid(0:1/granularity:2, 0:1/granularity:2);
a = size(xx, 1)/size(Z, 2);
zz = imresize(Z, a, 'bilinear');

% Model the smoothness of the curve as well. 
% If you add two bits to the end of tau, you can generate a
% 'smoothness' term to make the meshes much more verbose.
if smoothness > 0,
  H = fspecial('disk', smoothness);
  zz = imfilter(zz, H, 'replicate');
end
mesh.X = X;
mesh.Y = Y;
mesh.Z = Z;
mesh.xx = xx;
mesh.yy = yy;
mesh.zz = zz;
% Extend the end of the mesh out so that the car has a place to set down.
y_extend = repmat(mesh.yy(1,:), 10, 1);
x_extend = repmat((-1:.1:-.1)', 1, numel(mesh.xx(1,:)));
z_extend = repmat(mesh.zz(1,:), 10, 1);
mesh.xx = [x_extend; mesh.xx];
mesh.yy = [y_extend; mesh.yy];
mesh.zz = [z_extend; mesh.zz];
mesh.row_count = numel(mesh.xx(:,1));
mesh.col_count = numel(mesh.xx(1,:));
% xx, yy, and zz are going to be passed into the LocalSim, 
% and will be what we use to create the mesh
% Offsetting to get a nice start at (0, 0)
mesh.xx(:) = mesh.xx(:) + (0.5);
mesh.yy(:) = mesh.yy(:) - (1);
mesh.xx(:) = scale*mesh.xx(:);
mesh.yy(:) = scale*mesh.yy(:);
mesh.zz_row = mesh.zz(:)';

% This is just to get a good .csv file, if we needed it.
if is_recorded,
    dlmwrite('X', mesh.xx(:)');             
    dlmwrite('Y', mesh.yy(:)');
    dlmwrite('Z', mesh.zz(:)');
end

end