from Models import *
from Utilities import *
from docplex.mp.model import Model
from Inputs import *
import math


def solve_cplex(alp_instance: ALP, time_limit=False):
    # Exact P1 Solver using IBM ILOG CPLEX
    model = Model('P1')

    if time_limit:
        model.set_time_limit(alp_instance.compute_time_limit())

    # Inputs
    I = alp_instance.I
    R = alp_instance.R
    T = alp_instance.T
    s = alp_instance.s
    c_plus = alp_instance.c_plus
    c_minus = alp_instance.c_minus
    n = len(I)  # number of aircraft
    m = len(R)  # number of runways
    E = alp_instance.E
    T = alp_instance.T
    L = alp_instance.L
    M = 100

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
    return model, sol

# model, sol = solve_cplex(ex_alp_instance)


def relax_and_solve(alp_instance: ALP):

    # Inputs
    I = alp_instance.I
    R = alp_instance.R
    T = alp_instance.T
    s = alp_instance.s
    c_plus = alp_instance.c_plus
    c_minus = alp_instance.c_minus
    n = len(I)  # number of aircraft
    m = len(R)  # number of runways
    E = alp_instance.E
    T = alp_instance.T
    L = alp_instance.L
    M = 10000
    RR = ex_alp_instance.compute_RR()
    AR = ex_alp_instance.compute_AR()
    d = ex_alp_instance.compute_d()
    time_limit = ex_alp_instance.compute_time_limit()
    RC = RR + 1
    l = 1
    f = 1
    step = math.ceil((n - 2 * RR - 1) / (d - 1))
    pi = ex_alp_instance.etlt()
    model, sol = ex_alp_instance.solve_p1_given_pi(pi)
    z_pi = model.objective_value
    z_star = z_pi
    pi_star = pi.copy()
    # print(f'sequence: {sequence}')
    # print(f'runways: {runways}\n')
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
                left_aircraft_in_runways[runway] = [aircraft for index, aircraft in enumerate(non_relaxed_left) if left_runways[index] == runway]
                right_aircraft_in_runways[runway] = [aircraft for index, aircraft in enumerate(non_relaxed_right) if
                                                    right_runways[index] == runway]
            # Find last aircraft in left runways and first aircraft in right runway (may or may not be useful)
            for index, runway in enumerate(R):
                left_last_aircraft_in_runways[index] = None if runway not in left_runways else non_relaxed_left[len(left_runways) - list(reversed(left_runways)).index(runway) - 1]
                right_first_aircraft_runways[index] = None if runway not in right_runways else non_relaxed_right[right_runways.index(runway)]

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
                model.set_time_limit(alp_instance.compute_time_limit())

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
                                    model.logical_or(x[j] - x[i] >= (s[(i, j)] * (gamma[(i, r)] + gamma[(j, r)] - 1)),
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
                    must_satisfy_s_aircraft = list(reversed(left_aircraft_in_runways[runway] + right_aircraft_in_runways[runway]))
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
                    break

            if not feasibility_flag:
                # Solve P1 with a time limit
                model, sol = ex_alp_instance.solve_cplex(time_limit=True)

            if sol is not None:
                if model.objective_value < z_star:
                    landing_times = []
                    final_runways = []
                    for aircraft in I:
                        landing_times.append(sol[x[aircraft]])
                        for r in R:
                            if sol[gamma[aircraft, r]] > 0.5:
                                final_runways.append(r)
                    pi_star = [sort_x_based_on_y(I, landing_times), sort_x_based_on_y(final_runways, landing_times)]
                    z_star = model.objective_value

            RC += step

    return z_star


print(ex_alp_instance.relax_and_solve())