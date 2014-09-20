function [sol1, sol2] = ellipse_tangent_fun(ell_scale, pt)
% For an unrotated ellipse centered at (0,0) with scale ell_scale, this
% returns the two points on the ellipse where the tangent lines intersect
% the point given
a = ell_scale(1);
b = ell_scale(2);
px = pt(1);
py = pt(2);
if py ~= 0
    Ax = (py^2 + px^2*(b^2/a^2));
    Bx = (-2*px*b^2);
    Cx = (a^2*b^2-py^2*a^2);

    xsol1 = (-Bx + sqrt(Bx^2 - 4*Ax*Cx)) / (2*Ax);
    xsol2 = (-Bx - sqrt(Bx^2 - 4*Ax*Cx)) / (2*Ax);
    ysol1 = (-px/py*b^2/a^2) * xsol1 + b^2/py;
    ysol2 = (-px/py*b^2/a^2) * xsol2 + b^2/py;

else
    Ay = (px^2 + py^2*(a^2/b^2));
    By = (-2*py*a^2);
    Cy = (b^2*a^2-px^2*b^2);

    ysol1 = (-By + sqrt(By^2 - 4*Ay*Cy)) / (2*Ay);
    ysol2 = (-By - sqrt(By^2 - 4*Ay*Cy)) / (2*Ay);
    xsol1 = (-py/px*a^2/b^2) * ysol1 + a^2/px;
    xsol2 = (-py/px*a^2/b^2) * ysol2 + a^2/px;
end
sol1 = [xsol1; ysol1];
sol2 = [xsol2; ysol2];
