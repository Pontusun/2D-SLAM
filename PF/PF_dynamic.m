function [ particles_new ] = PF_dynamic( particles, move, sigma )
%PF_DYNAMIC Summary of this function goes here
%   Detailed explanation goes here

[n, m] = size(particles);
particles_new = zeros(n,m);

for i = 1:m
    particles_new(:,i) = particles(:,i) + move(i) + normrnd(0,sigma(i),[n,1]);
end

end

