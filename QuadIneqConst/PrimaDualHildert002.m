function [lambda] = PrimaDualHildert002(H, K)

[n,m] = size(K);
x_ini = zeros(n,m);
lambda = x_ini;
al = 10; 
for loop = 1:100
    lambda_p = lambda; 
    for i = 1:n
        omg = H(i,:)*lambda-H(i,i)*lambda(i,1);
        omg = omg +K(i,1);
        la = -omg/(H(i,i));
        lambda(i,1)= max(0,la);
    end
        al = (lambda-lambda_p)'*(lambda-lambda_p);
        if al<10e-3
            break;
        end
end