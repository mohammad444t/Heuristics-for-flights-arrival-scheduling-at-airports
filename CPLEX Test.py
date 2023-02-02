from docplex.mp.model import Model

model = Model('Test')


# Define decision variables
y_indices = [i for i in range(1, 6)]
y = model.continuous_var_dict(y_indices, name='y')

objective_function = model.sum(y[i] for i in y_indices)
model.set_objective("min", objective_function)

model.add_constraint(model.logical_or(y[1] >= 2, y[3] >=4))

if __name__ == '__main__':
    model.print_information()
    sol = model.solve()
    model.print_solution()
    print(sol[y[1]])
    if sol is None:
        print("Infeasible")
