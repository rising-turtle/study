%interp2_example2 p204
[x,y] = meshgrid(-3:0.8:3);
z=peaks(x,y);
[xi,yi] = meshgrid(-3:0.2:3);
zi_near = interp2(x,y,z,xi,yi,'nearset');
zi_liner = interp2(x,y,z,xi,yi);
zi_spline = interp2(x,y,z,xi,yi,'spline');
zi_cubic = interp2(x,y,z,xi,yi,'cubic');
hold on;
subplot(2,3,1);
surfc(x,y,z);
subplot(2,3,2);
surfc(xi,yi,zi_near);
subplot(2,3,3);
surfc(xi,yi,zi_liner);
subplot(2,3,4);
surfc(xi,yi,zi_spline);
subplot(2,3,5);
surfc(xi,yi,zi_cubic);