from Models import *
from Inputs import *
from Generators import *

print(ex_alp_instance.etlt())

ex1 = generate_random_alp_instance(150, 1)


print(ex1.solve_cplex())
print(ex1.relax_and_solve())
