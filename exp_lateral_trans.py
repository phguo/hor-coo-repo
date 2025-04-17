# coding:utf-8
import math
from itertools import product

from config import EXP_PARAMETERS, PARAMETERS
from io_operator import get_objective_value, get_gap
from run import make_model_and_solve

from rich import print as print

from solution import GurobiSolution


# def solve_lateral_trans_instances():
#     for instance_name, fc, lt, in product(
#             EXP_PARAMETERS['originally_optimized'],
#             EXP_PARAMETERS['exp_lateral_trans_facility_cap_times'],
#             [True, False],
#     ):
#         make_model_and_solve(instance_name, PARAMETERS, "LT", allow_lt=lt, f_cap_times=fc)

def solve_lateral_trans_instances():
    for instance_name, fc, lt, in product(
            ["07_10_04_0_0", "07_10_04_0_1"],
            EXP_PARAMETERS['exp_lateral_trans_facility_cap_times'],
            [False],
    ):
        make_model_and_solve(instance_name, PARAMETERS, "LT", allow_lt=lt, f_cap_times=fc)


def get_exp_lateral_trans_data():
    neos_server_results = {
        "07_10_04_0_1": {
            2: {'obj': 351684.392, 'gap': 2.35, 'lb': 343409.206},
            1.2: {'obj': 382418.938, 'gap': 4.21, 'lb': 366301.722},
            1: {'obj': 396569.384, 'gap': 2.62, 'lb': 386163.746}
        },
        "07_10_04_0_0": {
            2: {'obj': 296254.951, 'gap': 3.55, 'lb': 285741.467},
            1.2: {'obj': 326989.496, 'gap': 7.05, 'lb': 303920.882},
            1: {'obj': 339946.350, 'gap': 4.13, 'lb': 325912.964}
        },
    }

    result = []
    for instance_name, fc in product(
            EXP_PARAMETERS['originally_optimized'],
            EXP_PARAMETERS['exp_lateral_trans_facility_cap_times']
    ):
        obj_lt = get_objective_value(instance_name, "LT", allow_lt=True, f_cap_times=fc)
        obj_no_lt = get_objective_value(instance_name, "LT", allow_lt=False, f_cap_times=fc)

        if obj_lt == None:
            try:
                obj_lt = neos_server_results[instance_name][fc]['obj']
            except:
                pass

        dif = (obj_no_lt - obj_lt) / obj_lt * 100 if obj_lt != None and obj_no_lt != None else None

        # for lt in [True, False]:
        #     try:
        #         solution = GurobiSolution(instance_name, "LT", allow_lt=lt, f_cap_times=fc)
        #         maximum_demand = max(solution.scenarios_total_demand.values())
        #         large_cap_base = math.ceil(maximum_demand / solution.facility_num)
        #         small_cap_base = math.ceil(large_cap_base * 0.25)
        #         large_facility_cap = math.ceil(large_cap_base * fc)
        #         small_facility_cap = math.ceil(small_cap_base * fc)
        #         facility_total_cap = {key:large_facility_cap if value[0] == 1 else small_facility_cap
        #                               for key, value in solution.opened_facility_type.items()}
        #         facility_cap_used = solution.facility_stock
        #         used_ratio = sum(facility_cap_used.values()) / sum(facility_total_cap.values())
        #         res.append(sum(facility_cap_used.values()))
        #         res.append(sum(facility_total_cap.values()))
        #         res.append(used_ratio)
        #     except:
        #         res.append(None)

        res = [instance_name, fc, obj_lt, obj_no_lt, dif]
        result.append(res)

    return result


if __name__ == "__main__":
    # solve_lateral_trans_instances()
    print(get_exp_lateral_trans_data())

    # for lt in [True, False]:
    #     solution = GurobiSolution("03_10_02_0_0", "LT", allow_lt=lt, f_cap_times=1.2)
    #     print(solution.get_scenarios_routes())
