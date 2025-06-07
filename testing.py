import numpy as np
from lie_theory_lib import SO3_rotation as SO3, S3_rotation as S3


qwb = np.array([1.0, 2.0, 3.0, 4.0])
qwb /= np.linalg.norm(qwb)
Rwb = S3.Rq_mat(qwb)

omegab = 2. * np.pi * np.array([0.1, 0.2, 0.3])
N = 1000
dt = 0.01
for i in range(N):
    Rwb = SO3.plus_right(Rwb, omegab * dt)
    qwb = S3.plus_right(qwb, omegab * dt)
    print(np.linalg.norm(Rwb - S3.Rq_mat(qwb)))
