# coding:utf-8
from itertools import product

from copy import deepcopy

from config import EXP_PARAMETERS, PARAMETERS
from io_operator import get_objective_value, get_gap
from run import make_model_and_solve
from solution import GurobiSolution


def solve_aux_instances():
    for instance_name in EXP_PARAMETERS['originally_optimized']:
        for vehicle_capacity_times in EXP_PARAMETERS['exp_vehicle_capacity_times']:
            make_model_and_solve(instance_name, PARAMETERS, 'NO_VRP', solution_method="benders",
                                 vc_times=vehicle_capacity_times, p_type='AUX')


def transportation_cost_estimate(vc_times):
    instance_unit_transportation_cost = 0
    for instance in EXP_PARAMETERS['originally_optimized']:
        if vc_times == 1.0:
            solution = GurobiSolution(instance, "BAC")
        else:
            solution = GurobiSolution(instance, "NO_VRP", vc_times=vc_times, p_type="AUX")
        scenarios_routes = solution.get_scenarios_routes()

        scenario_transportation_cost = 0
        for s in scenarios_routes:

            facility_transportation_cost = 0
            for i in scenarios_routes[s]:
                for h in scenarios_routes[s][i]:
                    for route in scenarios_routes[s][i][h]:
                        route_cost = solution._get_route_distance(route) * solution.vehicle_info[h]['uc']
                        total_customer_distance_to_facility = sum([solution._get_distance(i, j) for j in route[1:-1]])
                        quantity_to_customer = 0
                        for j in route[1:-1]:
                            if j in solution.demand_set:
                                quantity_to_customer += solution.direct_trans.sum(i, j, s).getValue()
                            elif j in solution.facility_set:
                                quantity_to_customer += solution.lateral_trans.sum(i, j, '*', s).getValue()
                            else:
                                raise Exception("Wrong \'j\' ({}) type.".format(j))
                        route_transportation_cost = (
                                route_cost / total_customer_distance_to_facility / quantity_to_customer)
                        facility_transportation_cost += route_transportation_cost
            facility_transportation_cost /= len(scenarios_routes[s].keys())

            scenario_transportation_cost += facility_transportation_cost

        instance_unit_transportation_cost += scenario_transportation_cost / solution.scenario_num

    unit_transportation_cost = instance_unit_transportation_cost / len(EXP_PARAMETERS['originally_optimized'])

    result = round(unit_transportation_cost, 4)
    print('vehicle capacity times (vc_times): {}'.format(vc_times))
    print('Unit transportation cost:', result)
    return result


def solve_experimental_instances(problem_type):
    for instance_name, vc_times in product(EXP_PARAMETERS['originally_optimized'], EXP_PARAMETERS['exp_vehicle_capacity_times']):
        unit_transportation_cost = transportation_cost_estimate(vc_times)
        exp_sets = [{
            'ut_cost': unit_transportation_cost,
            'ut_times': uc_t,
            'fc_times': fc_t,
            'vc_times': vc_times,
            'p_type': problem_type
        }
            for uc_t, fc_t in product(
                EXP_PARAMETERS['exp_routing_cost_times'],
                EXP_PARAMETERS['exp_facility_cost_times'],
            )
        ]
        for exp_set in exp_sets:
            make_model_and_solve(instance_name, PARAMETERS, "NO_VRP", **exp_set)


def solve_preposition_transportation_instances():
    solve_experimental_instances('TRANS')


def solve_preposition_routing_instances():
    solve_experimental_instances('ROUTE')


def get_vehicle_routing_data(vc_times, unit_transportation_cost):
    result = []
    for instance_name in EXP_PARAMETERS['originally_optimized']:
        exp_sets = [{
            'ut_cost': unit_transportation_cost,
            'ut_times': uc_t,
            'fc_times': fc_t,
            'p_type': "ROUTE"
        }
            for uc_t, fc_t in product(
                EXP_PARAMETERS['exp_routing_cost_times'],
                EXP_PARAMETERS['exp_facility_cost_times'],
            )]
        for exp_set in exp_sets:
            if vc_times == 1.0:
                baseline_obj = get_objective_value(instance_name, "BAC")
            else:
                baseline_obj = get_objective_value(instance_name, "NO_VRP", vc_times=vc_times, p_type="AUX")
            exp_set_ = deepcopy(exp_set)
            exp_set_.update({'p_type': "TRANS"})
            obj_ = get_objective_value(instance_name, "NO_VRP", vc_times=vc_times, **exp_set_)
            obj = get_objective_value(instance_name, "NO_VRP", vc_times=vc_times, **exp_set)
            line = (
                instance_name,
                exp_set['ut_times'],
                vc_times,
                (obj_ - baseline_obj) / baseline_obj if obj_ != None else None,
                (obj - baseline_obj) / baseline_obj if obj != None else None
            )
            result.append(line)
    return result


if __name__ == "__main__":
    # solve auxiliary instances
    solve_aux_instances()

    # solve instances
    solve_preposition_transportation_instances()
    solve_preposition_routing_instances()

    # for vc_times in EXP_PARAMETERS['exp_vehicle_capacity_times']:
    #     unit_transportation_cost = transportation_cost_estimate(vc_times, optimized_instances)
    #     data = get_vehicle_routing_data(vc_times, unit_transportation_cost, optimized_instances)
    #     vehicle_routing_table(data)
