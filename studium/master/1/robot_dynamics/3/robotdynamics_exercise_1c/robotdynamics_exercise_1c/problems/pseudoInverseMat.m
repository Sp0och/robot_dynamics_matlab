function [ pinvA ] = pseudoInverseMat(A, lambda)
% Input: Any m-by-n matrix, and a damping factor.
% Output: An n-by-m pseudo-inverse of the input according to the Moore-Penrose formula

% Get the number of rows (m) and columns (n) of A
[m, n] = size(A);

% TODO: complete the computation of the pseudo-inverse.
% Hint: How should we account for both left and right pseudo-inverse forms?
%test

AT = A.';
    if m > n
        inside = AT*A;
        inside_regulated = inside + lambda * lambda * eye(n);
        iri = inv(inside_regulated);
        pinvA = iri*AT;
    else 
        inside = A*AT;
        inside_regulated = inside + lambda * lambda * eye(m);
        iri = inv(inside_regulated);
        pinvA = AT*iri;
    end 
end
