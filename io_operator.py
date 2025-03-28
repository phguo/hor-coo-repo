# coding:utf-8

import json
from itertools import product
from os import listdir
from warnings import warn

from gurobipy.gurobipy import tupledict, read

from config import FILE_DIR, PARAMETERS, EXP_PARAMETERS


# ''' # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # '''
# ''' # # # # # # # # # # # # instance processing # # # # # # # # # # # # # '''
# ''' # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # '''


def instance_file_name_process(instance_name, instance_type=None, **info):
    instances_dir = FILE_DIR['instances']
    file_name = instance_name

    uct_level = info.get('uct_level', None)
    if instance_type == 'STC' and uct_level != None:
        instances_dir = FILE_DIR['exp_stc_aux_instances']
        file_name = '{}_{}'.format(instance_name, uct_level)
    return instances_dir, file_name


def write_instance(data, instance_name, instance_type=None, **info):
    instance_dir, file_name = instance_file_name_process(instance_name, instance_type, **info)
    json_str = json.dumps(data, sort_keys=True, indent=4)
    with open(instance_dir + file_name + '.json', 'w') as f:
        f.write(json_str)


def read_instance(instance_name, instance_type=None, **info):
    instance_dir, file_name = instance_file_name_process(instance_name, instance_type, **info)
    with open(instance_dir + file_name + '.json', 'r') as f:
        lines = f.read()
    dict_obj = json.loads(
        lines,
        object_hook=lambda d: {int(k) if k.lstrip('-').isdigit() else k: v for k, v in d.items()}
    )
    return dict_obj


# ''' # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # '''
# ''' # # # # # # # # # # # solver readable model file  # # # # # # # # # # '''
# ''' # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # '''

def write_model(model, instance_name, instance_type=None, **info):
    instance_dir, file_name = solution_file_name_process(instance_name, instance_type, **info)
    model.write(instance_dir + file_name + '.mps')


def read_model(instance_name, instance_type=None, **info):
    instance_dir, file_name = instance_file_name_process(instance_name, instance_type, **info)
    model = read(instance_dir + file_name + '.mps')
    return model


# ''' # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # '''
# ''' # # # # # # # # # # # # # # solution file # # # # # # # # # # # # # # '''
# ''' # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # '''


def solution_file_name_process(instance_name, solution_type, **exp_set):
    if solution_type == 'DE':
        solutions_dir = FILE_DIR['de_solutions']
        file_name = '{}'.format(instance_name)

    elif solution_type == 'DE_NO_VI':
        solutions_dir = FILE_DIR['de_solutions_no_vi']
        file_name = '{}'.format(instance_name)

    elif solution_type == 'LBBD':
        solutions_dir = FILE_DIR['LBBD_solutions']
        file_name = '{}'.format(instance_name)

    elif solution_type == 'BAC':
        solutions_dir = FILE_DIR['BAC_solutions']
        file_name = '{}'.format(instance_name)

    elif solution_type == 'PA_DE':
        solutions_dir = FILE_DIR['exp_pareto']
        ratio = exp_set.get('ratio', False)
        assert ratio, Exception("'ratio' is required for a 'PA_DE' instance.")
        file_name = '{}_{}'.format(instance_name, ratio)

    elif solution_type == "LT":
        solutions_dir = FILE_DIR['exp_lateral_trans']
        allow_lt = exp_set.get('allow_lt', None)
        assert allow_lt != None
        file_name = "{}_{}".format(instance_name, "_".join([str(a) for a in exp_set.values()]))

    elif solution_type == "NO_VRP":
        solutions_dir = FILE_DIR['exp_vehicle_routing']
        problem_type = exp_set.get('p_type', None)
        unit_trans_cost = exp_set.get('ut_cost', None)
        uc_times = exp_set.get('ut_times', None)
        fc_times = exp_set.get('fc_times', None)
        vc_times = exp_set.get('vc_times', None)

        assert problem_type != None
        if problem_type != "AUX":
            assert unit_trans_cost != None
            assert uc_times != None
            assert fc_times != None

        if vc_times == None:
            if problem_type == "AUX":
                file_name = '{}_{}'.format(instance_name, 'A')
            else:
                file_name = '{}_{}_{:.4}_{:.2}_{:04d}'.format(
                    instance_name,
                    'T' if problem_type == "TRANS" else 'R',
                    unit_trans_cost, uc_times, fc_times)
        else:
            if problem_type == "AUX":
                file_name = '{}_{}_{:.2}'.format(instance_name, 'A', vc_times)
            else:
                file_name = '{}_{}_{:.2}_{:.4}_{:.2}_{:04d}'.format(
                    instance_name,
                    'T' if problem_type == "TRANS" else 'R',
                    vc_times, unit_trans_cost, uc_times, fc_times)


    elif solution_type == "STC":
        solutions_dir = FILE_DIR['exp_stochasticity']
        uct_level = exp_set.get('uct_level', None)
        p_type = exp_set.get('p_type', None)
        assert uct_level != None
        assert p_type != None
        if p_type == "AUX":
            ad = ''
        elif p_type == "EEV":
            expected_value_problem = exp_set.get('ev_p', None)
            assert expected_value_problem != None
            ad = '{}_{}'.format(1 if expected_value_problem else 0, uct_level)
        elif p_type == "WS":
            scenario_id = exp_set.get('s_id', None)
            assert scenario_id != None
            ad = scenario_id
        else:
            raise Exception("Unknown p_type ({}) for STC instance".format(p_type))
        file_name = "{}_{}_{:.2}_{}".format(instance_name, p_type, uct_level, ad)
    else:
        raise Exception('Unknown solution_type=\'{}\'!'.format(solution_type))

    return solutions_dir, file_name


def write_gurobi_results(model, instance_name, solution_type, **exp_set):
    solutions_dir, file_name = solution_file_name_process(instance_name, solution_type, **exp_set)

    # if int(model.getAttr('Status')) not in PARAMETERS['gurobi_infeasible_code']:
    if model.getAttr('SolCount') >= 1:
        model.write(solutions_dir + file_name + '.sol')
        write_non_zero_gurobi_sol(instance_name, solution_type, **exp_set)

    model.write(solutions_dir + file_name + '.json')
    runtime = model.getAttr('Runtime')  # For fixing '0 Runtime Output Issue' in Gurobi 9.0
    with open(solutions_dir + file_name + '.json', 'r') as f:
        json_content = json.load(f)
        json_content['SolutionInfo']['Runtime'] = str(runtime)
    with open(solutions_dir + file_name + '.json', 'w') as f:
        f.write(json.dumps(json_content, indent=4))


def write_lbbd_json(instance_name, json_content, solution_type, **exp_set):
    solution_dir, file_name = solution_file_name_process(instance_name, solution_type, **exp_set)
    with open(solution_dir + file_name + '.json', 'w') as f:
        f.write(json.dumps(json_content, indent=4))


def write_lbbd_sol(instance_name, sol_content, solution_type, **exp_set):
    solution_dir, file_name = solution_file_name_process(instance_name, solution_type, **exp_set)
    obj, sol = sol_content[0], sol_content[1:]
    lines = ['# Objective value = {}\n'.format(obj)]
    for row in sol:
        if len(row) == 4:
            var_name = row[0:-1]
            var_value = row[-1]
            if '[' in var_name[-1]:
                # p,q,h,i,s
                var_name = "{},{},{}]".format(var_name[-1][:-1], var_name[0], var_name[1])
                # p,q,s,i,h
                var_name = var_name.split(',')
                var_name = "{},{},{},{},{}]".format(var_name[0], var_name[1], var_name[4][:-1], var_name[3],
                                                    var_name[2])
            else:
                var_name = "{}[{},{}]".format(var_name[-1], var_name[0], var_name[1])
        elif len(row) == 2:
            var_name = row[0]
            var_value = row[-1]
        else:
            raise Exception("Unexpected row length: {}".format(len(row)))

        for keyword in [
            "assign", "location", "stock", "PMP_vehicle",
            "direct_trans", "lateral_trans", "routing", "flow"
        ]:
            if keyword in var_name:
                var_name = var_name.replace("PMP_", '').replace("RASP_", '').replace("VDSP_", '')
                line = "{} {}\n".format(var_name, var_value)
                lines.append(line)

        with open(solution_dir + file_name + '.sol', 'w') as f:
            f.writelines(lines)
        write_non_zero_gurobi_sol(instance_name, solution_type, **exp_set)


def write_non_zero_gurobi_sol(instance_name, solution_type, **exp_set):
    solutions_dir, file_name = solution_file_name_process(instance_name, solution_type, **exp_set)
    with open(solutions_dir + file_name + '.sol', 'r') as f:
        f_content = [a for a in f if
                     a.split(' ')[-1] != '0\n' and
                     a.split(' ')[-1] != '0.0\n' and
                     a.split(' ')[-1] != '-0.0\n']
    with open(solutions_dir + file_name + '.nonzero', 'w') as f:
        f.writelines(f_content)


def read_gurobi_sol(instance_name, solution_type, **exp_set):
    # def nround(a, abs_tol=0.0001):
    #     if abs(a - round(a)) <= abs_tol:
    #         return round(a)
    #     else:
    #         return (a)

    solutions_dir, file_name = solution_file_name_process(instance_name, solution_type, **exp_set)
    solution = dict()
    non_zero_solution = dict()
    with open(solutions_dir + file_name + '.sol', 'r') as f:
        f_content = [a.strip('\n') for a in f]
        lines = f_content[1:]
    lines = [(a.split(' ')[0], float(a.split(' ')[1])) for a in lines]
    # rounding 'vars_value' to avoid numerical issues
    # may cause assertion excepting in 'unit_test.py' by round() (solved)
    lines = [(a[0], round(a[1]) if 'flow' not in a[0] else a[1]) for a in lines]
    lines = [(a[0].strip(']').split('[')[0], a[0].strip(']').split('[')[1], a[1]) for a in lines]
    lines = [(a[0], tuple(int(b) for b in a[1].split(',')), a[2]) for a in lines]
    for vars, vars_key, vars_value in lines:
        if vars not in solution:
            solution[vars] = tupledict()
        if vars not in non_zero_solution:
            non_zero_solution[vars] = tupledict()
        solution[vars][vars_key] = vars_value
        if vars_value != 0:
            non_zero_solution[vars][vars_key] = vars_value
    return solution, non_zero_solution


def get_solution_stats(instance_name, solution_type, **exp_set):
    solutions_dir, file_name = solution_file_name_process(instance_name, solution_type, **exp_set)
    try:
        with open(solutions_dir + file_name + '.json', 'r') as f:
            solution_stats = json.load(f)
    except:
        # warn("The {}.json file does not exist.".format(instance_name))
        return dict()

    return solution_stats


def get_solution_stats_detail(instance_name, solution_type, key_str, **exp_set):
    solution_status = get_solution_stats(instance_name, solution_type, **exp_set)
    res = solution_status.get('SolutionInfo', dict()).get(key_str, None)
    return float(res) if res != None else None


def get_objective_value(instance_name, solution_type, **exp_set):
    return get_solution_stats_detail(instance_name, solution_type, 'ObjVal', **exp_set)


def get_objective_bound(instance_name, solution_type, **exp_set):
    return get_solution_stats_detail(instance_name, solution_type, 'ObjBound', **exp_set)


def get_gap(instance_name, solution_type, **exp_set):
    return get_solution_stats_detail(instance_name, solution_type, 'MIPGap', **exp_set)


def get_runtime(instance_name, solution_type, **exp_set):
    return get_solution_stats_detail(instance_name, solution_type, 'Runtime', **exp_set)


def get_sol_count(instance_name, solution_type, **exp_set):
    return get_solution_stats_detail(instance_name, solution_type, "SolCount", **exp_set)


# ''' # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # '''
# ''' # # # # # # # # # # # # # # for unit test # # # # # # # # # # # # # # '''
# ''' # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # '''


def solved_instance_classify(instance_type, resolve_all=False, show_to_console=False):
    if instance_type in ["DE", "DE_NO_VI", "LBBD", "BAC"]:
        exp_set_li = [dict()]
        instance_li = [a.strip('.json') for a in listdir(FILE_DIR['instances']) if '.json' in a]
    elif instance_type == "PA_DE":
        # exp_set_li = EXP_PARAMETERS['exp_pareto_list']
        exp_set_li = [{'ratio': r} for r in EXP_PARAMETERS['exp_pareto_list']]
        instance_li = EXP_PARAMETERS['originally_optimized']
    elif instance_type == "LT":
        exp_set_li = [{'allow_lt': lt, 'ac_times': tm}
                      for tm, lt in product(EXP_PARAMETERS['exp_lateral_trans_list'], [True, False])]
        instance_li = EXP_PARAMETERS['originally_optimized']
    elif instance_type == "NO_VRP":
        exp_set_li = [
            {
                'vc_times': d,
                'ut_cost': EXP_PARAMETERS['exp_unit_transportation_cost'],
                'ut_times': a, 'fc_times': b, 'p_type': c
            }
            for a, b, c, d in product(
                EXP_PARAMETERS['exp_routing_cost_times'],
                EXP_PARAMETERS['exp_facility_cost_times'],
                ["ROUTE", "AUX"],
                EXP_PARAMETERS['exp_vehicle_capacity_times']
            )
        ]
        # problem without routing sub problem != need to test
        instance_li = EXP_PARAMETERS['originally_optimized']
    elif instance_type == "STC":
        for p_type in ["EEV", "AUX"]:
            exp_set_li = [{'uct_level': uct_level, 'p_type': p_type, 'ev_p': False}
                          for uct_level in EXP_PARAMETERS['uncertainty_level']]
            instance_li = EXP_PARAMETERS['originally_optimized']

    else:
        raise Exception("Unknown instance type='{}'!".format(instance_type))

    if resolve_all:
        optimized = []
        feasible_sol_found = []
        no_feasible_sol_found = [(a, None) for a in instance_li]
        proved_to_be_infeasible = []
    else:
        optimized = []
        feasible_sol_found = []
        no_feasible_sol_found = []
        proved_to_be_infeasible = []
        for instance, exp_set in product(instance_li, exp_set_li):
            # exp_set = {'ratio': ratio}
            pair = (instance, exp_set)
            status = get_solution_stats_detail(instance, instance_type, 'Status', **exp_set)
            if status == None:
                no_feasible_sol_found.append(pair)
            elif status in PARAMETERS['gurobi_infeasible_code']:
                proved_to_be_infeasible.append(pair)
            elif status in PARAMETERS['gurobi_optimal']:
                optimized.append(pair)
            else:
                # if get_gap(instance, instance_type, **exp_set) == float("1e+100"):
                #     no_feasible_sol_found.append(pair)
                # else:
                #     feasible_sol_found.append(pair)
                if get_sol_count(instance, instance_type, **exp_set) == 0:
                    no_feasible_sol_found.append(pair)
                else:
                    feasible_sol_found.append(pair)

    optimized = sorted(optimized, key=lambda a: a[0])
    feasible_sol_found = sorted(feasible_sol_found, key=lambda a: a[0])
    no_feasible_sol_found = sorted(no_feasible_sol_found, key=lambda a: a[0])
    proved_to_be_infeasible = sorted(proved_to_be_infeasible, key=lambda a: a[0])

    total = len(optimized) + len(feasible_sol_found) + len(no_feasible_sol_found) + len(proved_to_be_infeasible)

    if show_to_console:
        print("collecting ({:0>3d})              : {}\n"
              "OPTIMIZED* ({:0>3d})              : {}\n"
              "feasible solution found ({:0>3d}) : {}\n"
              "no feasible sol found ({:0>3d})   : {}\n"
              "proved to be infeasible ({:0>3d}) : {}\n\n"
              .format(total, instance_type,
                      len(optimized), optimized,
                      len(feasible_sol_found), feasible_sol_found,
                      len(no_feasible_sol_found), no_feasible_sol_found,
                      len(proved_to_be_infeasible), proved_to_be_infeasible)
              )

    return optimized, feasible_sol_found, no_feasible_sol_found, proved_to_be_infeasible


if __name__ == '__main__':
    pass
