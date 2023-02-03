def sort_x_based_on_y(x, y):
    return [item for _, item in sorted(zip(y, x))]


def sum_list_num(l1, num):
    return [x + num for x in l1]


def list_to_str_tuple(l1):
    return tuple([str(item) for item in l1])