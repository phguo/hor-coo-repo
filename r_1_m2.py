# coding:utf-8
# See exp_vehicle_routing.py
import csv
import json
from copy import deepcopy

import pandas as pd
from gurobipy import tupledict

from config import PARAMETERS, META_DATA_TEMPLATE
from solver import StochasticModels

# Estimated based on optimized instances with |D| = 10.
UNIT_TRANS_COST = 6.5874

# Instances with |D| = 10 are optimized, and instances with |D| = 20 can be not optimized.
VRP_IMPACT_INSTANCES = [
    '03_10_02_0_0', '03_10_02_0_1', '03_10_04_0_0', '03_10_04_0_1',
    '05_10_02_0_0', '05_10_02_0_1', '05_10_04_0_0', '05_10_04_0_1',
    '07_10_02_0_0', '07_10_02_0_1', '07_10_04_0_0', '07_10_04_0_1',

    '03_20_02_0_0', '03_20_02_0_1', '03_20_04_0_0', '03_20_04_0_1',
    '05_20_02_0_0', '05_20_02_0_1', '05_20_04_0_0', '05_20_04_0_1',
    '07_20_02_0_0', '07_20_02_0_1', '07_20_04_0_0', '07_20_04_0_1',

    # '03_40_02_0_0', '03_40_02_0_1', '03_40_02_1_0', '03_40_02_1_1',
    # '03_40_04_0_0', '03_40_04_0_1', '03_40_04_1_0', '03_40_04_1_1',
    # '05_40_02_0_0', '05_40_02_0_1', '05_40_02_1_0', '05_40_02_1_1',
    # '05_40_04_0_0', '05_40_04_0_1', '05_40_04_1_0', '05_40_04_1_1',
    # '07_40_02_0_0', '07_40_02_0_1', '07_40_02_1_0', '07_40_02_1_1',
    # '07_40_04_0_0', '07_40_04_0_1', '07_40_04_1_0', '07_40_04_1_1'
]

FIRST_STAGE_KWS = [
    # 'vehicle',
    'assign', 'location', 'stock'
]


def solve_trans_model(instance):
    # Make and solve model with transportation problems as subproblems.
    parameters = deepcopy(PARAMETERS)
    exp_set = {'ut_cost': UNIT_TRANS_COST, 'ut_times': 1, 'fc_times': 1, 'vc_times': 1, 'p_type': "TRANS"}
    SM = StochasticModels(instance, parameters, instance_type="NO_VRP", **exp_set)
    model = SM.deterministic_equivalent_problem()
    model.optimize()

    # Obtain first-stage solutions.
    first_stage_solution = dict()
    for var in model.getVars():
        if any(kw in var.varName for kw in FIRST_STAGE_KWS):
            first_stage_solution[var.varName] = var.x

    # Save solution and stat of the model to file.
    model.write(f"./r_revision/r.1.m2/solutions/{instance}_t.sol")
    model.write(f"./r_revision/r.1.m2/solutions/{instance}_t.json")

    return first_stage_solution, model


def solve_model_with_fixed_first_stage(instance, first_stage_solution):
    # Make and solve model with first-stage decisions fixed to that yielded by the transportation model.
    parameters = deepcopy(PARAMETERS)
    SM = StochasticModels(instance, parameters, instance_type="DE")
    model = SM.deterministic_equivalent_problem()
    for var_name, var_value in first_stage_solution.items():
        model.getVarByName(var_name).setAttr("ub", var_value)
        model.getVarByName(var_name).setAttr("lb", var_value)
    model.optimize()

    # Save solution and stat of the model to file.
    model.write(f"./r_revision/r.1.m2/solutions/{instance}_r.sol")
    model.write(f"./r_revision/r.1.m2/solutions/{instance}_r.json")

    return model


def run_all():
    for instance in VRP_IMPACT_INSTANCES:
        print(f"Running instance {instance}...")
        first_stage_solution, m_t = solve_trans_model(instance)
        m_r = solve_model_with_fixed_first_stage(instance, first_stage_solution)


def make_solution_table():
    def load_sol(dir):
        non_zero_solution = dict()
        with open(dir, 'r') as f:
            f_content = [a.strip('\n') for a in f]
            lines = f_content[1:]
        lines = [(a.split(' ')[0], float(a.split(' ')[1])) for a in lines]
        for var_name, var_value in lines:
            if (var_value > 0.00001 or var_value < -0.00001) and any(
                    kw in var_name for kw in FIRST_STAGE_KWS) and "aux" not in var_name:
                var_name = var_name.replace("PMP_", '')
                var_name, var_key = var_name.split('[')[0], var_name.split('[')[1].strip(']')
                var_key = tuple(int(a) for a in var_key.split(','))
                if var_name not in non_zero_solution:
                    non_zero_solution[var_name] = dict()
                non_zero_solution[var_name][var_key] = var_value
        return non_zero_solution

    def pretty_first_stage_sol(non_zero_solution):
        location, stock, assign = non_zero_solution['location'], non_zero_solution['stock'], non_zero_solution['assign']
        result = dict()

        # facility capacity
        for facility, facility_type in location.keys():
            if facility not in result:
                result[facility] = [None, None, []]
            capacity = META_DATA_TEMPLATE["facility_types_info"][facility_type]["cap"]
            result[facility][0] = capacity

        # facility stock
        for facility, stock in stock.items():
            facility = facility[0]
            assert facility in result
            result[facility][1] = stock

        # facility assigned demand points
        for facility, demand_point in assign.keys():
            assert facility in result
            result[facility][2].append(demand_point)

        # Reviewer: number of facilities, total initial inventory, # number of vehicles
        large_facility_num = len([v for v in result.values() if v[0] == 4000])
        small_facility_num = len([v for v in result.values() if v[0] == 1000])
        number_of_facility = len(result)
        total_capacity = sum([v[0] for v in result.values()])
        total_initial_inventory = sum([v[1] for v in result.values()])
        inventory_rate = total_initial_inventory / total_capacity * 100
        return [
            # f"{number_of_facility} ({large_facility_num}, {small_facility_num})",
            number_of_facility,
            total_capacity, total_initial_inventory, inventory_rate, ]

    def load_json(dir):
        with open(dir, 'r') as f:
            lines = f.read()
            data = json.loads(
                lines, object_hook=lambda d: {int(k) if k.lstrip('-').isdigit() else k: v for k, v in d.items()})
        return data

    rows = []
    for instance in VRP_IMPACT_INSTANCES:
        real_sol = load_sol(f"./r_revision/r.2.6/BAC_solutions/{instance}.sol")
        real_json = load_json(f"./r_revision/r.2.6/BAC_solutions/{instance}.json")
        trans_sol = load_sol(f"./r_revision/r.1.m2/solutions/{instance}_t.sol")
        trans_json = load_json(f"./r_revision/r.1.m2/solutions/{instance}_t.json")
        # fixed_sol = load_sol(f"./r_revision/r.1.m2/solutions/{instance}_r.sol")
        fixed_json = load_json(f"./r_revision/r.1.m2/solutions/{instance}_r.json")

        real_obj = real_json["SolutionInfo"]["ObjVal"]
        obj_t = trans_json["SolutionInfo"]["ObjVal"]
        obj_r = fixed_json["SolutionInfo"]["ObjVal"]

        real_facility_number, real_total_capacity, real_total_inventory, real_rate = pretty_first_stage_sol(real_sol)
        t_facility_number, t_total_capacity, t_total_inventory, t_rate = pretty_first_stage_sol(trans_sol)

        dif_t = (obj_t - real_obj) / real_obj * 100
        dif_r = (obj_r - real_obj) / real_obj * 100

        rows.append([
            instance.replace('_', '-'),
            real_facility_number, real_total_capacity, round(real_total_inventory), round(real_rate, 2),
            # real_obj,
            t_facility_number, t_total_capacity, round(t_total_inventory), round(t_rate, 2),
            # obj_t,
            round(dif_t, 2),
            # obj_r,
            round(dif_r, 2)
        ])

    df = pd.DataFrame(rows, columns=[
        "Instance",
        "Number_f", "Total_cap", "Total_inv", "Inv_rate",
        # "Obj.",
        "Number_f_t", "Total_cap_t", "Total_inv_t", "Inv_rate_t",
        # "Obj_t",
        "dif_t",
        # "Obj_r",
        "dif_r"
    ])

    # df.loc['mean'] = df.mean()

    print(df.to_latex(index=False))


if __name__ == '__main__':
    # run_all()

    make_solution_table()
