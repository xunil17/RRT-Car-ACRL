function d = dist(q1,q2)
%%measures distance between two coordinates or states
% d = sqrt((q1.x-q2.x)^2 + (q1.y-q2.y)^2);
d = abs(q1.x - q2.x) + abs(q1.y - q2.y);
end