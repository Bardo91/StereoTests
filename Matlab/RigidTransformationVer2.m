% From is the matrix of orignial 3 dimensional points as columns and
% To is the matrix of 3 dimensional points as corresponding columns into which it
% transforms

function H = RigidTransformation(From, To)
%average of the entire point set
p1 = mean(From,2);
p2 = mean(To, 2);

%the entire pont set with the average substracted
q1 = From - repmat(p1,1,size(From,2));
q2 = To - repmat(p2,1,size(To,2));


% matrix of sums of product
H = zeros(3,3);

for i=1:size(From,2)
    H = H + q1(:,i)*q2(:,i)';
end

Sxx = H(1,1); Sxy = H(1,2); Sxz = H(1,3);
Syx = H(2,1); Syy = H(2,2); Syz = H(2,3);
Szx = H(3,1); Szy = H(3,2); Szz = H(3,3);

N = [ Sxx+Syy+Szz, Syz-Szy, Szx-Sxz, Sxy-Syx;
      Syz-Szy, Sxx-Syy-Szz, Sxy+Syx, Szx+Sxz;
      Szx-Sxz, Sxy+Syx, -Sxx+Syy-Szz, Syz+Szy;
      Sxy-Syx, Szx+Sxz, Syz+Szy, -Sxx-Syy+Szz];
  
[q,lambda] = eigs(N,1);
Ra = q2r(q)

[U,S,V] = svd(H);


R = V*U';
det(R)
R



T = p2 - R * p1

H=[R,T];


