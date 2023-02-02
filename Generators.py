import random
from Models import ALP

def generate_random_alp_instance(n, m):
    I_1 = list(range(1, n + 1))
    R_1 = list(range(1, m + 1))
    T_1 = [random.randint(90, 90 + 17 * n) for _ in range(n)]
    E_1 = [item - random.randint(9, 75) for item in T_1]
    L_1 = [item + random.randint(350, 500) for item in T_1]
    c_minus_1 = [random.randint(10, 30) for _ in range(n)]
    c_plus_1 = [random.randint(10, 30) for _ in range(n)]

    s_1 = dict()
    for i in range(len(I_1)):
        for j in range(len(I_1)):
            if i != j:
                s_1[(i + 1, j + 1)] = random.randint(3, 15)

    return ALP(I_1, R_1, T_1, s_1, E_1, L_1, c_plus_1, c_minus_1)