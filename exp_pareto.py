# coding:utf-8

from config import PARAMETERS, EXP_PARAMETERS
from io_operator import get_objective_value, get_solution_stats_detail
from run import make_model_and_solve


def solve_instances():
    for instance_name in EXP_PARAMETERS['originally_optimized']:
        for ratio in EXP_PARAMETERS['exp_pareto_list']:
            make_model_and_solve(instance_name, PARAMETERS, "PA_DE", "benders", ratio=ratio)


def filter_optimized():
    optimized_instances = []
    for instance_name in EXP_PARAMETERS['originally_optimized']:
        instance_optimized = True
        for ratio in EXP_PARAMETERS['exp_pareto_list']:
            status = get_solution_stats_detail(instance_name, "PA_DE", 'Status', ratio=ratio)
            if status not in PARAMETERS['gurobi_optimal']:
                print(instance_name, ratio, status)
                # instance_optimized = False
                # break
        if instance_optimized:
            optimized_instances.append(instance_name)
    return optimized_instances


def get_pareto_frontiers_data():
    # optimized_instances = EXP_PARAMETERS['originally_optimized']
    optimized_instances = filter_optimized()
    result = []
    for instance_name in optimized_instances:
        x_list, y_list = [], []
        for ratio in EXP_PARAMETERS['exp_pareto_list']:
            obj = get_objective_value(instance_name, "PA_DE", ratio=ratio)
            x_list.append(ratio)
            y_list.append(obj)
        result.append((instance_name, x_list, y_list))
    result.sort(key=lambda a: a[2][0] if type(a[2][0]) == float else float('inf'), reverse=True)
    return result


if __name__ == "__main__":
    solve_instances()
