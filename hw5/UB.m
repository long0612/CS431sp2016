function [ind,out,U] = UB(C,P,B,D,S)
% [ind,out,U] = UB(C,P,B,D,S)
% Utilization bound
%
% Long Le
% University of Illinois
%

N = numel(C);

U = zeros(N,1);
out = zeros(N,1);
ind = false(N,1);
for i = 1:N
    U(i) = i*(2^(1/i)-1);
    out(i) = (C(i)+2*S+B(i)+P(i)-D(i))/P(i);
    for j = 1:i-1
        out(i) = out(i) + (C(j)+2*S)/P(j);
    end
    ind(i) = out(i) <= U(i);
end
