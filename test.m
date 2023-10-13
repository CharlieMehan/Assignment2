% Define points
point1OnLine = [1, 2, 3];
point2OnLine = [4, 5, 6];
pointOnPlane = [2, 2, 2];

% Calculate u and w
u = point2OnLine - point1OnLine;
w = point1OnLine - pointOnPlane;

% Display the values of u and w
disp('u:');
disp(u);
disp('w:');
disp(w);