% Determine if a graph is connected
% INPUTS: adjacency matrix
% OUTPUTS: Boolean variable {0,1}
% Note: this only works for undirected graphs
% Idea by Ed Scheinerman, circa 2006, source: http://www.ams.jhu.edu/~ers/matgraph/
%                                     routine: matgraph/@graph/isconnected.m

function S = isconnected(adj)

if not(isempty(find(sum(adj)==0))); S = false; return; end

n = length(adj);
x = [1; zeros(n-1,1)]; % [1,0,...0] nx1 vector 

while 1
     y = x;
     x = adj*x + x;
     x = x>0;
     
     if x==y; break; end

end

S = true;
if sum(x)<n; S = false; end