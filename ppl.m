function [ Kx ] = ppl( A, B, P0)

N = length(A);

T1 = [];
for i=1:N
   T1 = [T1, (A^(i-1))*B];
end


P = poly(eig(A));
T2 = [];
for i=1:N
    T2 = [T2; zeros(1,i-1), P(1:N-i+1)];
end

T3 = zeros(N,N);
for i=1:N
    T3(i, N+1-i) = 1;
end

T = T1*T2*T3;

Pd = poly(P0);

dP = Pd - P;

Flip = [N+1:-1:2]';
Kz = dP(Flip);
Kx = Kz*inv(T);

end
