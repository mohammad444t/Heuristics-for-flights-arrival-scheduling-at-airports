"""
The earliest target landing time (ETLT) construction heuristic (Salehipour et al., 2013).

Input: An instance of the ALP, where I and R are sets of aircraft and runways, w = {} and pi_2×n = {}.
Output: A feasible landing sequence pi for the ALP.
"""
from Models import *
from Utilities import *
from docplex.mp.model import Model


def etlt(alp_instance: ALP):

    # Initialization
    r = 1  # runway number
    k = 0  # aircraft index
    I = alp_instance.I
    R = alp_instance.R
    T = alp_instance.T
    s = alp_instance.s
    n = len(I)  # number of aircraft
    m = len(R)  # number of runways
    w = sort_x_based_on_y(I, T)  # Sort aircraft based on their target landing times
    pi = [[0 for _ in range(n)], [0 for _ in range(n)]]
    pi[0][k] = w[0]  # The first aircraft (i.e., w[0]) appears first in pi
    pi[1][k] = r  # Assign the first aircraft (i.e., w[1]) to runway 1
    w.pop(0)

    # Allocation
    while w:
        k += 1
        if len(w) == 1:
            pi[0][k] = w[0]
            pi[1][k] = r
            w.pop(0)
        else:
            if T[w[1] - 1] < T[w[0] - 1] + s[(w[0], w[1])]:
                # Assign the first two aircraft in w to two different runways
                pi[0][k] = w[0]
                pi[1][k] = r
                k += 1
                r += 1
                if r > m:
                    r = 1
                pi[0][k] = w[1]
                pi[1][k] = r
                w.pop(0)
                w.pop(0)
            else:
                # Assign the first two aircraft in w to the same runway
                pi[0][k] = w[0]
                pi[1][k] = r
                k += 1
                pi[0][k] = w[1]
                pi[1][k] = r
                w.pop(0)
                w.pop(0)
            r += 1
            if r > m:
                r = 1

    return pi




