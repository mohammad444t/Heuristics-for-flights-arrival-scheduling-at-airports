from Generators import *

m = 2
ns = [50, 100, 150, 200, 250, 300, 350]
avg_times_cplex = []
avg_times_r_and_s = []
avg_percentages_of_error = []
num_of_instances = 5

for n in ns:
    print(n)
    avg_time_cplex = 0
    avg_time_r_and_s = 0
    avg_percentage_of_error = 0
    for instance in range(num_of_instances):
        alp_instance = generate_random_alp_instance(n, m)
        _, _, _, _, z_star_cplex, elapsed_time_cplex = alp_instance.solve_cplex()
        _, z_star_r_and_s, _, elapsed_time_r_and_s = alp_instance.relax_and_solve()
        avg_percentage_of_error += 100 * (1 / num_of_instances) * (z_star_r_and_s - z_star_cplex) / (z_star_cplex)
        avg_time_r_and_s += elapsed_time_r_and_s / num_of_instances
        avg_time_cplex += elapsed_time_cplex / num_of_instances
    avg_times_cplex.append(avg_time_cplex)
    avg_times_r_and_s.append(avg_time_r_and_s)
    avg_percentages_of_error.append(avg_percentage_of_error)

generate_bar_plot(ns, avg_percentages_of_error, f'Average percentage of error in objective value', 'Number of Aircraft', 'Average percentage of error', f'error {m}')
generate_comparison_bar_plot(ns, avg_times_r_and_s, avg_times_cplex, f'Average Elapsed Time (m = {m})', 'Number of Aircraft', 'Elapsed Time', 'R & S', 'CPLEX', f'Time {m}')

