from ETLT import *
from Models import *
from Inputs import *
from Generators import *

my_p = etlt(ex_alp_instance)
print(ex_alp_instance.etlt())

ex1 = generate_random_alp_instance(400, 2)


print(ex1.solve_cplex())
print(ex1.relax_and_solve())