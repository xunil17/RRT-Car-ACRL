%% Creates a map_struct from csv file

a = csvread('100.csv');

numrows = size(a,1);
numcols = size(a,2);

for i = 1:numrows*numcols
   a(i) = ~a(i);
   
   if i < numrows
      a(i) = 0; 
   elseif i > numrows*numcols - numrows
      a(i) = 0; 
   elseif mod(i,numcols) == 0
      a(i) = 0; 
   elseif mod(i-1,numcols) == 0
      a(i) = 0; 
   end
   
end

start.x = 5;
start.y = 5;

goal.x = 99;
goal.y = 5;

map_struct.map_name = 'rover';
map_struct.bridge_locations = [27;27];
map_struct.bridge_probabilities = 0;
map_struct.seed_map = a;
map_struct.start = start;
map_struct.goal = goal;