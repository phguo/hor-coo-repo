# coding:utf-8
import random
from itertools import product
from math import ceil

from config import EXP_PARAMETERS, PARAMETERS, RANDOM_SEED
from instance import Instance
from io_operator import get_objective_value
from run import make_model_and_solve

random.seed(RANDOM_SEED)


def make_and_save_aux_instances():
    for instance_name in EXP_PARAMETERS['originally_optimized']:
        for uct_level in EXP_PARAMETERS['uncertainty_level']:
            # use saved instance
            instance = Instance(instance_name)
            if uct_level == 1.0:
                pass
            elif uct_level in [2.0, 3.0, 4.0, 5.0]:
                distribution, uct_parameters = EXP_PARAMETERS['uct_types'][int(uct_level - 2)]
                if distribution == "triangular":
                    dst = random.triangular
                elif distribution == "uniform":
                    dst = random.uniform
                else:
                    dst = None
                for s, j in product(instance.scenarios, instance.demand_set):
                    instance.scenarios[s][j][0] = ceil(instance.points_population[j] * dst(*uct_parameters[s]))
            else:
                raise Exception("Unknown uncertain level (uct_level): {}.".format(uct_level))
            instance.save_instance("STC", uct_level=uct_level)


def solve_aux_instances():
    for instance_name in EXP_PARAMETERS['originally_optimized']:
        for uncertainty_level in EXP_PARAMETERS['uncertainty_level']:
            make_model_and_solve(
                instance_name, PARAMETERS, 'STC', solution_method="benders", uct_level=uncertainty_level, p_type='AUX')


def solve_expected_value_problems():
    for instance_name in EXP_PARAMETERS['originally_optimized']:
        exp_sets = [
            {
                'uct_level': uct_level,
                'p_type': "EEV",
                'ev_p': e
            }
            for uct_level, e in product(
                EXP_PARAMETERS['uncertainty_level'],
                [True, False]
            )
        ]
        for exp_set in exp_sets:
            make_model_and_solve(instance_name, PARAMETERS, "STC", **exp_set)


def solve_wait_and_see_problems():
    for instance_name in EXP_PARAMETERS['originally_optimized']:
        scenario_num = int(instance_name.split('_')[2])
        exp_sets = [
            {
                'uct_level': uct_level,
                'p_type': "WS",
                's_id': s
            }
            for uct_level, s in product(
                EXP_PARAMETERS['uncertainty_level'],
                [a for a in range(scenario_num)]
            )
        ]
        for exp_set in exp_sets:
            make_model_and_solve(instance_name, PARAMETERS, "STC", **exp_set)


def get_expected_value_data():
    result = []
    for instance_name in EXP_PARAMETERS['originally_optimized']:
        exp_sets = [
            {
                'uct_level': uct_level,
                'p_type': "EEV",
                'ev_p': e
            }
            for uct_level, e in product(
                EXP_PARAMETERS['uncertainty_level'],
                [False]
            )
        ]
        for exp_set in exp_sets:
            baseline_obj = get_objective_value(instance_name, "STC", p_type="AUX", uct_level=exp_set['uct_level'])
            obj = get_objective_value(instance_name, "STC", **exp_set)
            if obj != None and baseline_obj != None:
                value_of_stochastic_solution = (obj - baseline_obj) / baseline_obj
            else:
                value_of_stochastic_solution = None
            line = (instance_name, value_of_stochastic_solution)
            result.append(line)

    return result


def get_wait_and_see_data():
    result = []
    for instance_name in EXP_PARAMETERS['originally_optimized']:
        scenario_num = int(instance_name.split('_')[2])
        exp_sets = [
            {
                'uct_level': uct_level,
                'p_type': "WS"
            }
            for uct_level in EXP_PARAMETERS['uncertainty_level']
        ]
        for exp_set in exp_sets:
            average_ws_objective = 0
            for s_id in [a for a in range(scenario_num)]:
                exp_set.update({'s_id': s_id})
                obj = get_objective_value(instance_name, "STC", **exp_set)
                if obj == None:
                    average_ws_objective = None
                    break
                else:
                    average_ws_objective += obj
            baseline_obj = get_objective_value(instance_name, "STC", p_type="AUX", uct_level=exp_set['uct_level'])
            if average_ws_objective != None:
                average_ws_objective /= scenario_num
            if average_ws_objective != None and baseline_obj != None:
                expected_value_of_perfect_info = (baseline_obj - average_ws_objective) / baseline_obj
            else:
                expected_value_of_perfect_info = None
            line = (instance_name, expected_value_of_perfect_info)
            result.append(line)

    return result


if __name__ == '__main__':

    # make auxiliary instances
    make_and_save_aux_instances()
    # solve auxiliary instances
    solve_aux_instances()

    # solve
    solve_expected_value_problems()
    solve_wait_and_see_problems()
