
vA = randn(2, 1)
vB = randn(2, 1)
vC = randn(2, 1)

figure(23)
clf
hold on
axis([-2 2 -2 2])
mA = [zeros(2,1), vA];
mB = [zeros(2,1), vB];
mC = [zeros(2,1), vC];
line(mA(1,:), mA(2,:),'Color','b')
line(mB(1,:), mB(2,:),'Color','k')
AxB = vA(1)*vB(2) - vA(2)*vB(1)
AxC = vA(1)*vC(2) - vA(2)*vC(1)
CxB = vC(1)*vB(2) - vC(2)*vB(1)
if AxB > 0
    is_between = AxC > 0 && CxB > 0
else
    is_between = AxC < 0 && CxB < 0
end
if is_between
    line(mC(1,:), mC(2,:),'Color','g')
else
    line(mC(1,:), mC(2,:),'Color','r')
end
