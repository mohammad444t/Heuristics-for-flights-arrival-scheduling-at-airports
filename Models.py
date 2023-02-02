import time
from Utilities import *
from docplex.mp.model import Model
import math


class ALP:
    def __init__(self, I: list, R: list, T: list, s: dict, E: list, L: list, c_plus: list, c_minus: list):
        # I: list of aircrafts (usually [1,...,n])
        # R: list of runways (usually [1,...,m])
        # T: list of target landing times
        # s: dict of separation (setup) times {(i, j): s_ij}, i,j: aircraft numbers
        # E: Earliest landing times of aircraft
        # L: Latest landing times of aircraft
        # c_plus = Costs of late landings of aircraft
        # c_minus = Costs of early landings of aircraft
        self.I = I
        self.R = R
        self.T = T
        self.s = s
        self.E = E
        self.L = L
        self.c_plus = c_plus
        self.c_minus = c_minus

    def etlt(self):
        # Initialization
        r = 1  # runway number
        k = 0  # aircraft index
        I = self.I
        R = self.R
        T = self.T
        s = self.s
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

    def solve_cplex(self, time_limit=False):

        # Exact P1 Solver using IBM ILOG CPLEX
        model = Model('P1')
        if time_limit:
            model.set_time_limit(self.compute_time_limit())

        # Inputs
        I = self.I
        R = self.R
        T = self.T
        s = self.s
        c_plus = self.c_plus
        c_minus = self.c_minus
        n = len(I)  # number of aircraft
        m = len(R)  # number of runways
        E = self.E
        T = self.T
        L = self.L
        M = 10000

        # Variable indices
        x_indices = [i for i in range(1, n + 1)]
        alpha_indices = x_indices.copy()
        beta_indices = x_indices.copy()
        y_indices = [(i, j) for i in range(1, n + 1) for j in range(1, n + 1) if i != j]
        delta_indices = y_indices.copy()
        gamma_indices = [(i, r) for i in range(1, n + 1) for r in range(1, m + 1)]

        # Define Variables
        x = model.continuous_var_dict(x_indices, lb=0, name='x')
        alpha = model.continuous_var_dict(alpha_indices, lb=0, name='alpha')
        beta = model.continuous_var_dict(beta_indices, lb=0, name='beta')
        y = model.binary_var_dict(y_indices, name='y')
        delta = model.binary_var_dict(delta_indices, name='delta')
        gamma = model.binary_var_dict(gamma_indices, name='gamma')

        # Define Objective Function
        objective_function = model.sum(c_minus[i - 1] * alpha[i] for i in alpha_indices) + \
                             model.sum(c_plus[i - 1] * beta[i] for i in beta_indices)
        model.set_objective("min", objective_function)

        # Define Constraints
        for i in x_indices:
            # Constraint 1.
            model.add_constraint(E[i - 1] <= x[i])
            model.add_constraint(x[i] <= L[i - 1])
            model.add_constraint(x[i] - T[i - 1] == alpha[i] - beta[i])
            model.add_constraint(model.sum(gamma[(i, r)] for r in R) == 1)
            for j in x_indices:
                if i != j:
                    model.add_constraint(x[j] - x[i] >= s[(i, j)] * delta[(i, j)] - M * y[(j, i)])
                    model.add_constraint(y[(i, j)] + y[(j, i)] == 1)
                    model.add_constraint(delta[(i, j)] == delta[(j, i)])
                    for r in R:
                        model.add_constraint(delta[(i, j)] >= gamma[(i, r)] + gamma[(j, r)] - 1)

        sol = model.solve()
        # model.print_solution()
        pi_star = None
        z_star = None
        final_landing_times = None
        if sol is not None:
            landing_times = []
            final_runways = []
            for aircraft in I:
                landing_times.append(sol[x[aircraft]])
                for r in R:
                    if sol[gamma[aircraft, r]] > 0.5:
                        final_runways.append(r)
            pi_star = [sort_x_based_on_y(I, landing_times), sort_x_based_on_y(final_runways, landing_times)]
            z_star = model.objective_value
            final_landing_times = sorted(landing_times)

        return model, sol, pi_star, final_landing_times, z_star

    def compute_d(self):
        n = len(self.I)
        m = len(self.R)

        if m == 1:
            if n <= 20:
                return 4
            elif 20 < n <= 150:
                return 25
            elif 150 < n <= 250:
                return 50
            else:
                return 180
        else:
            if n <= 20:
                return 4
            elif 20 < n <= 100:
                return 25
            elif 100 < n <= 250:
                return 40
            else:
                return 80

    def compute_RR(self):
        n = len(self.I)
        m = len(self.R)

        if m == 1:
            if n <= 150:
                return max(4, n // 25)
            elif 150 < n <= 150:
                return max(4, n // 50)
            else:
                return max(4, n // 150)
        else:
            return 6

    def compute_AR(self):
        n = len(self.I)

        if n <= 250:
            return 5
        else:
            return 6

    def compute_time_limit(self):
        n = len(self.I)
        m = len(self.R)

        if m == 1:
            if n <= 100:
                return 1
            else:
                return 2
        else:
            if n <= 250:
                return 1
            else:
                return 2

    def solve_p1_given_pi(self, pi: list):

        model = Model('P1_given_pi')

        # Inputs
        I = self.I
        R = self.R
        s = self.s
        c_plus = self.c_plus
        c_minus = self.c_minus
        n = len(I)  # number of aircraft
        m = len(R)  # number of runways
        E = self.E
        T = self.T
        L = self.L
        M = 10000
        sequence = pi[0]
        runways = pi[1]

        # Variable indices
        x_indices = [i for i in range(1, n + 1)]
        alpha_indices = x_indices.copy()
        beta_indices = x_indices.copy()
        y_indices = [(i, j) for i in range(1, n + 1) for j in range(1, n + 1) if i != j]
        delta_indices = y_indices.copy()
        gamma_indices = [(i, r) for i in range(1, n + 1) for r in range(1, m + 1)]

        # Define Variables
        x = model.continuous_var_dict(x_indices, lb=0, name='x')
        alpha = model.continuous_var_dict(alpha_indices, lb=0, name='alpha')
        beta = model.continuous_var_dict(beta_indices, lb=0, name='beta')

        # The following variables are now parameters!
        y = dict()
        delta = dict()
        gamma = dict()
        for i in x_indices:
            for j in x_indices:
                if i != j:
                    y[(i, j)] = 1 if sequence.index(i) < sequence.index(j) else 0
                    delta[(i, j)] = 1 if runways[sequence.index(i)] == runways[sequence.index(j)] else 0
        for gamma_index in gamma_indices:
            aircraft_index = sequence.index(gamma_index[0])
            aircraft_runway = runways[aircraft_index]
            gamma[gamma_index] = 1 if gamma_index[1] == aircraft_runway else 0

        # Define Objective Function
        objective_function = model.sum(c_minus[i - 1] * alpha[i] for i in alpha_indices) + \
                             model.sum(c_plus[i - 1] * beta[i] for i in beta_indices)
        model.set_objective("min", objective_function)

        # Define Constraints
        for i in x_indices:
            model.add_constraint(E[i - 1] <= x[i])
            model.add_constraint(x[i] <= L[i - 1])
            model.add_constraint(x[i] - T[i - 1] == alpha[i] - beta[i])
            for j in x_indices:
                if i != j:
                    model.add_constraint(x[j] - x[i] >= s[(i, j)] * delta[(i, j)] - M * y[(j, i)])

        sol = model.solve()
        # model.print_solution()

        z_star = model.objective_value

        return model, sol, z_star

    def relax_and_solve(self):
        # Inputs
        I = self.I
        R = self.R
        T = self.T
        s = self.s
        c_plus = self.c_plus
        c_minus = self.c_minus
        n = len(I)  # number of aircraft
        m = len(R)  # number of runways
        E = self.E
        T = self.T
        L = self.L
        M = 10000
        RR = self.compute_RR()
        AR = self.compute_AR()
        d = self.compute_d()
        time_limit = self.compute_time_limit()
        RC = RR + 1
        l = 1
        f = 1
        step = math.ceil((n - 2 * RR - 1) / (d - 1))

        start_time = time.time()
        pi = self.etlt()
        model, sol, z_pi = self.solve_p1_given_pi(pi)
        z_star = z_pi
        pi_star = pi.copy()
        final_landing_times = []
        if z_star == 0:
            return z_star
        else:
            for _ in range(d):
                sequence = pi_star[0]
                runways = pi_star[1]
                relaxed_aircraft = sequence[RC - RR - 1: min(n, RC + RR)]
                non_relaxed_left = sequence[: RC - RR - 1]
                non_relaxed_right = sequence[min(n, RC + RR):]
                left_indices = list(range(len(non_relaxed_left)))
                relaxed_indices = [len(left_indices) + i for i in range(len(relaxed_aircraft))]
                right_indices = [len(left_indices) + len(relaxed_indices) + i for i in range(len(non_relaxed_right))]
                relaxed_runways = runways[RC - RR - 1: min(n, RC + RR)]
                left_runways = runways[: RC - RR - 1]
                right_runways = runways[min(n, RC + RR):]
                left_last_aircraft_in_runways = [None for _ in range(m)]
                right_first_aircraft_runways = [None for _ in range(m)]
                left_aircraft_in_runways = dict()
                right_aircraft_in_runways = dict()
                for runway in R:
                    left_aircraft_in_runways[runway] = [aircraft for index, aircraft in enumerate(non_relaxed_left) if
                                                        left_runways[index] == runway]
                    right_aircraft_in_runways[runway] = [aircraft for index, aircraft in enumerate(non_relaxed_right) if
                                                         right_runways[index] == runway]
                # Find last aircraft in left runways and first aircraft in right runway (may or may not be useful)
                for index, runway in enumerate(R):
                    left_last_aircraft_in_runways[index] = None if runway not in left_runways else non_relaxed_left[
                        len(left_runways) - list(reversed(left_runways)).index(runway) - 1]
                    right_first_aircraft_runways[index] = None if runway not in right_runways else non_relaxed_right[
                        right_runways.index(runway)]

                # print(f'left: {non_relaxed_left}')
                # print(f'relaxed: {relaxed_aircraft}')
                # print(f'right: {non_relaxed_right}')
                # for runway in R:
                #     print(f'runway {runway} aircraft left: {left_aircraft_in_runways[runway]}')
                #     print(f'runway {runway} aircraft right: {right_aircraft_in_runways[runway]}')
                # print(f'last left: {left_last_aircraft_in_runways}')
                # print(f'first right: {right_first_aircraft_runways}')
                # print('\n')

                # Form and Solve the Relaxed Problem (P2):
                feasibility_flag = True
                for f in range(1, 4):  # Look back parameter: at least 1, at most 3

                    model = Model('P2')
                    model.set_time_limit(self.compute_time_limit())

                    # Variable indices
                    x_indices = [i for i in range(1, n + 1)]
                    alpha_indices = x_indices.copy()
                    beta_indices = x_indices.copy()
                    gamma_indices = [(i, r) for i in range(1, n + 1) for r in range(1, m + 1)]  # Some gammas are fixed
                    relaxed_x_indices = relaxed_aircraft.copy()
                    left_x_indices = non_relaxed_left.copy()
                    right_x_indices = non_relaxed_right.copy()

                    # Define Variables
                    x = model.integer_var_dict(x_indices, lb=0, name='x')
                    alpha = model.continuous_var_dict(alpha_indices, lb=0, name='alpha')
                    beta = model.continuous_var_dict(beta_indices, lb=0, name='beta')
                    gamma = model.binary_var_dict(gamma_indices, name='gamma')

                    # Define Objective Function
                    objective_function = model.sum(c_minus[i - 1] * alpha[i] for i in alpha_indices) + \
                                         model.sum(c_plus[i - 1] * beta[i] for i in beta_indices)
                    model.set_objective("min", objective_function)

                    # Define Constraints
                    for i in x_indices:
                        # Constraint 1.
                        model.add_constraint(E[i - 1] <= x[i])
                        model.add_constraint(x[i] <= L[i - 1])
                        model.add_constraint(x[i] - T[i - 1] == alpha[i] - beta[i])
                        model.add_constraint(model.sum(gamma[(i, r)] for r in R) == 1)

                    # Black dashed arrows
                    for r in R:
                        for i in relaxed_x_indices:
                            for j in relaxed_x_indices:
                                if i != j:
                                    model.add_constraint(
                                        model.logical_or(
                                            x[j] - x[i] >= (s[(i, j)] * (gamma[(i, r)] + gamma[(j, r)] - 1)),
                                            x[i] - x[j] >= (s[(j, i)] * (gamma[(i, r)] + gamma[(j, r)] - 1))))

                    # Colored dashed arrows from the left
                    for r in R:
                        for j in relaxed_x_indices:
                            for i in left_x_indices:
                                model.add_constraint(x[j] - x[i] >= s[(i, j)] * (gamma[(i, r)] + gamma[(j, r)] - 1))

                    # Colored dashed arrows to the right
                    for r in R:
                        for j in right_x_indices:
                            for i in relaxed_x_indices:
                                model.add_constraint(x[j] - x[i] >= s[(i, j)] * (gamma[(i, r)] + gamma[(j, r)] - 1))

                    # Solid arrows + Precedences among the non-relaxed aircraft
                    for runway in R:
                        must_satisfy_s_aircraft = list(
                            reversed(left_aircraft_in_runways[runway] + right_aircraft_in_runways[runway]))
                        for aircraft in must_satisfy_s_aircraft:
                            for look_back in range(f):
                                if must_satisfy_s_aircraft.index(aircraft) + f < len(must_satisfy_s_aircraft):
                                    prev_aircraft = must_satisfy_s_aircraft[must_satisfy_s_aircraft.index(aircraft) + f]
                                    model.add_constraint(x[aircraft] - x[prev_aircraft] >= s[(prev_aircraft, aircraft)])

                    # Fixed gammas
                    for aircraft, runway in zip(left_x_indices, left_runways):
                        model.add_constraint(gamma[(aircraft, runway)] == 1)
                    for aircraft, runway in zip(right_x_indices, right_runways):
                        model.add_constraint(gamma[(aircraft, runway)] == 1)

                    sol = model.solve()
                    # model.print_solution()

                    # Now check the feasibility of the solution of P2
                    x_in_runways = dict()
                    aircraft_in_runways = dict()
                    for r in R:
                        x_in_runways[r] = []
                        aircraft_in_runways[r] = []
                        for i in x_indices:
                            if sol[gamma[(i, r)]] > 0.5:
                                x_in_runways[r].append(sol[x[i]])
                                aircraft_in_runways[r].append(i)

                    feasibility_flag = True
                    for r in R:
                        for aircraft_1, aircraft_1_time in zip(aircraft_in_runways[r], x_in_runways[r]):
                            for aircraft_2, aircraft_2_time in zip(aircraft_in_runways[r], x_in_runways[r]):
                                if aircraft_1 != aircraft_2 and aircraft_2_time > aircraft_1_time and round(
                                        aircraft_2_time) < round(aircraft_1_time) + s[(aircraft_1, aircraft_2)]:
                                    feasibility_flag = False
                                    break
                            if not feasibility_flag:
                                break
                        if not feasibility_flag:
                            break

                    if feasibility_flag:
                        # Skip other look backs
                        landing_times = []
                        final_runways = []
                        for aircraft in I:
                            landing_times.append(sol[x[aircraft]])
                            for r in R:
                                if sol[gamma[aircraft, r]] > 0.5:
                                    final_runways.append(r)

                        if model.objective_value < z_star:
                            pi_star = [sort_x_based_on_y(I, landing_times), sort_x_based_on_y(final_runways, landing_times)]
                            z_star = model.objective_value
                            final_landing_times = landing_times.copy()
                        break

                if not feasibility_flag:
                    # Solve P1 with a time limit
                    model, sol, pi_star1, final_landing_times, z_star1 = self.solve_cplex(time_limit=True)
                    if sol is not None:
                        if model.objective_value < z_star:
                            pi_star = pi_star1
                            z_star = z_star1

                RC += step

        elapsed_time = time.time() - start_time
        return pi_star, z_star, final_landing_times, elapsed_time


