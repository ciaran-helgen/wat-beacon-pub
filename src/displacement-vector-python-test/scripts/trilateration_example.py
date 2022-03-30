import numpy as np
from trilateration import trilateration

print('Trilateration example: A')
 
#Reference points
P1 = np.array([	27.297,	-4.953,	1.47])
P2 = np.array([	25.475,	-6.124,	2.36])
P3 = np.array([	22.590,	0.524,	1.2])
#distances
s1 = 3.851  # distance to P1
s2 = 3.875 # distance to P2
s3 = 3.514 # distance to P3

P = np.array([P1, P2, P3] ) # Reference points matrix
P = np.column_stack([P1, P2, P3])
S = np.array([s1, s2, s3]) # Distance vector
W = np.eye(3)  # Weigths matrix


N1, N2 = trilateration(P,S,W)

print('Solution:')
print(N1[1:].flatten())
#print(f"N1 = {N1[1:].flatten()}")
print(N2[1:].flatten())
#print(f"N2 = {N2[1:].flatten()}")

print('Quality factor:')
q = N1[0]-(N1[1]**2 + N1[2]**2 + N1[3]**2)
print(q)
#print(f"q = {q}")