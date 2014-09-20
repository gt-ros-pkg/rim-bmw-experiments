clear
a = 6;
b = 3;
px = 0;
py = 12;

Ax = (py^2 + px^2*(b^2/a^2));
Bx = (-2*px*b^2);
Cx = (a^2*b^2-py^2*a^2);

xsol1x = (-Bx + sqrt(Bx^2 - 4*Ax*Cx)) / (2*Ax)
xsol2x = (-Bx - sqrt(Bx^2 - 4*Ax*Cx)) / (2*Ax)
ysol1x = (-px/py*b^2/a^2) * xsol1x + b^2/py
ysol2x = (-px/py*b^2/a^2) * xsol2x + b^2/py

Ay = (px^2 + py^2*(a^2/b^2));
By = (-2*py*a^2);
Cy = (b^2*a^2-px^2*b^2);

ysol1y = (-By + sqrt(By^2 - 4*Ay*Cy)) / (2*Ay)
ysol2y = (-By - sqrt(By^2 - 4*Ay*Cy)) / (2*Ay)
xsol1y = (-py/px*a^2/b^2) * ysol1y + a^2/px
xsol2y = (-py/px*a^2/b^2) * ysol2y + a^2/px

py == 0
[xsol1x ysol1x]
[xsol2x ysol2x]

px == 0
% use if py == 0
[xsol2y ysol2y]
[xsol1y ysol1y]
