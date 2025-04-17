# coding:utf-8
# See r_1_m2.py, r_2_6.py
import json

import pandas as pd

from config import META_DATA_TEMPLATE

# Instances with |D| = 10 are optimized, and instances with |D| = 20 can be not optimized.
VRP_IMPACT_INSTANCES = [
    '03_10_02_0_0', '03_10_02_0_1', '03_10_04_0_0', '03_10_04_0_1',
    '05_10_02_0_0', '05_10_02_0_1', '05_10_04_0_0', '05_10_04_0_1',
    '07_10_02_0_0', '07_10_02_0_1', '07_10_04_0_0', '07_10_04_0_1',

    '03_20_02_0_0', '03_20_02_0_1', '03_20_04_0_0', '03_20_04_0_1',
    '05_20_02_0_0', '05_20_02_0_1', '05_20_04_0_0', '05_20_04_0_1',
    '07_20_02_0_0', '07_20_02_0_1', '07_20_04_0_0', '07_20_04_0_1',
]

FIRST_STAGE_KWS = [
    # 'vehicle',
    'assign', 'location', 'stock'
]

SOL_DIR_DIC = {
    "de": "./de_solutions_no_vi",
    "de_vi": "./de_solutions",
    "lbbd": "./LBBD_solutions/backup210815_8c16g",
    "bac": "./BAC_solutions/backup210815_8c16g",
    # "lbbd_pp": "./r_revision/r.2.6/LBBD_solutions",
    "bac_pp": "./r_revision/r.2.6/BAC_solutions"
}


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
        total_capacity = int(round(sum([v[0] for v in result.values()])))
        total_initial_inventory = int(round(sum([v[1] for v in result.values()])))
        inventory_rate = total_initial_inventory / total_capacity * 100
        return [
            f"{number_of_facility} ({large_facility_num}, {small_facility_num})",
            # number_of_facility,
            total_capacity, total_initial_inventory, inventory_rate]

    def load_json(dir):
        with open(dir, 'r') as f:
            lines = f.read()
            data = json.loads(
                lines, object_hook=lambda d: {int(k) if k.lstrip('-').isdigit() else k: v for k, v in d.items()})
        return data

    rows = []
    for instance in VRP_IMPACT_INSTANCES:

        # Select the best solution
        real_sol, real_json, real_obj = None, None, float('inf')
        for _, dir in SOL_DIR_DIC.items():
            new_real_sol = load_sol(f"{dir}/{instance}.sol")
            new_real_json = load_json(f"{dir}/{instance}.json")
            new_obj = float(new_real_json["SolutionInfo"]["ObjVal"])
            if new_obj < real_obj:
                real_sol, real_json, real_obj = new_real_sol, new_real_json, new_obj

        trans_sol = load_sol(f"./r_revision/r.1.m2/solutions/{instance}_t.sol")
        trans_json = load_json(f"./r_revision/r.1.m2/solutions/{instance}_t.json")
        # fixed_sol = load_sol(f"./r_revision/r.1.m2/solutions/{instance}_r.sol")
        fixed_json = load_json(f"./r_revision/r.1.m2/solutions/{instance}_r.json")

        # real_obj = float(real_json["SolutionInfo"]["ObjVal"])
        obj_t = trans_json["SolutionInfo"]["ObjVal"]
        obj_r = fixed_json["SolutionInfo"]["ObjVal"]

        real_facility_number, real_total_capacity, real_total_inventory, real_rate = pretty_first_stage_sol(real_sol)
        t_facility_number, t_total_capacity, t_total_inventory, t_rate = pretty_first_stage_sol(trans_sol)

        dif_t = (obj_t - real_obj) / real_obj * 100
        dif_r = (obj_r - real_obj) / real_obj * 100

        rows.append([
            instance.replace('_', '-'),
            real_facility_number, real_total_capacity, real_total_inventory, real_rate,
            t_facility_number, t_total_capacity, t_total_inventory, t_rate,
            round(dif_t, 2),
            round(dif_r, 2)
        ])

    df = pd.DataFrame(rows, columns=[
        "Instance",
        "Number_f", "Total_cap", "Total_inv", "Inv_rate",
        "Number_f_t", "Total_cap_t", "Total_inv_t", "Inv_rate_t",
        "dif_t", "dif_r"
    ])

    # df.loc['mean'] = df.mean(numeric_only=True)

    print(df.to_latex(index=False, float_format=lambda x: f"{x:.2f}"))


if __name__ == '__main__':
    # run_all()

    make_solution_table()
