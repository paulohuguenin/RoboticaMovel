clear all;


a1 = [2, 6];
a2 = [4, 6];
a3 = [3, 3];

b1 = [5, 8];
b2 = [7, 8];
b3 = [8, 7];
b4 = [7, 5];
b5 = [5, 6];

c1 = [6, 4];
c2 = [8, 2];
c3 = [5, 2];

i1 = [1, 3];
i2 = [1, 1];

f1 = [9, 8];
f2 = [9, 5];
f3 = [9, 2];


A = polyshape([a1; a2; a3]);
B = polyshape([b1; b2; b3; b4; b5]);
C = polyshape([c1; c2; c3]);

[centro_A_x, centro_A_y] = centroid(A);
[centro_B_x, centro_B_y] = centroid(B);
[centro_C_x, centro_C_y] = centroid(C);

% test scale
plot([A, B, C])
hold on
A_escalado = scale(A, 1.2, [centro_A_x, centro_A_y]);
plot(A_escalado)