function [is_between] = vector_between_fun(vA, vB, vC)
% returns true if vC is between vA and vB
AxBgcmp = vA(1)*vB(2)   >   vA(2)*vB(1);
AxCgcmp = vA(1)*vC(2,:) > vA(2)*vC(1,:);
CxBgcmp = vC(1,:)*vB(2) > vC(2,:)*vB(1);
is_between = zeros(1,size(vC,2));
if AxBgcmp
    is_between = AxCgcmp & CxBgcmp;
else
    is_between = ~AxCgcmp & ~CxBgcmp;
end

% if AxB > 0
%     is_between = AxC > 0 && CxB > 0;
% else
%     is_between = AxC < 0 && CxB < 0;
% end
