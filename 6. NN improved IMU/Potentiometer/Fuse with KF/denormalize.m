function Y = denormalize(X,sigma,mu)
    Y = X.*sigma + mu;
end