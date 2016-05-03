function [ind,out,DD] = ET(C,P,B,D,S)
% [ind,out,DD] = ET(C,P,B,D,S)
% Exact schedulability test
%
% Long Le
% University of Illinois
%

N = numel(C);
DD = D;

out = zeros(N,1);
ind = false(N,1);
for i = 1:N
    out(i) = C(i)+2*S+B(i);
    for j = 1:i-1
        out(i) = out(i) + C(j)+2*S;
    end
    
    while 1
        newOut = C(i)+2*S+B(i);
        for j = 1:i-1
            newOut = newOut + ceil(out(i)/P(j))*(C(j)+2*S);
        end
        
        if newOut == out(i) && out(i) <= D(i)
            ind(i) = 1;
            break;
        end
        if newOut > D(i)
            ind(i) = 0;
            out(i) = newOut; % output the latest
            break;
        end
        out(i) = newOut;
    end
end