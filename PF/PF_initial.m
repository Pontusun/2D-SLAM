function [ particles, w ] = PF_initial( n, state )
%PF_INITIAL Summary of this function goes here
%   Detailed explanation goes here

particles = zeros(n, length(state));
w = 1/n * ones(n,1);

for i = 1:length(state)
    particles(:,i) = state(i);
end

end

