import random
from Models import ALP
import matplotlib.pyplot as plt
import matplotlib as mpl
from Utilities import *


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


def generate_comparison_bar_plot(x, y1, y2, title, xlabel, ylabel, y1label, y2label, name):

    mpl.rc('font', family='Times New Roman')
    mpl.rc('font', size=12)

    fig, ax = plt.subplots(figsize=(8, 6))

    # Numbers of pairs of bars you want
    N = 3

    # Data on X-axis

    # Specify the values of blue bars (height)
    blue_bar = y1
    # Specify the values of orange bars (height)
    orange_bar = y2

    # Position of bars on x-axis
    ind = [i * 10 for i in range(1, len(x) + 1)]

    # Width of a bar
    width = 2

    # Plotting
    ax.bar(ind, blue_bar, width, label='y1label')
    ax.bar(sum_list_num(ind, width), orange_bar, width, label='y2label')

    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(title)

    # xticks()
    # First argument - A list of positions at which ticks should be placed
    # Second argument -  A list of labels to place at the given locations
    ax.set_xticks(sum_list_num(ind, width / 2), list_to_str_tuple(x))

    # Finding the best position for legends and putting it
    ax.legend(loc='best')
    fig.show()
    fig.savefig(f'{name}.svg', format='svg')
