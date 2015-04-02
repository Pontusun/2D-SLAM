function [ index ] = PF_resample( w, n )
% Resample n samples with replacement from weights w,
% and return corresponding indices

nw = length(w);
if nargin < 2,
  n = nw;
end

% Construct normalized cumulative sum vector:
wsum = cumsum(w(:)');
wsum = wsum./wsum(end);

% Random samples with cumsum:
r = rand(1, n);
[ignore, isort] = sort([wsum r]);

% Use sum of 1's to get indices:
s = [ones(1,nw) zeros(1,n)];
s = s(isort);
ssum = cumsum(s);

% Return indices:
index = ssum(s == 0) + 1;

end

