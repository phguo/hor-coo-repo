# coding:utf-8
import multiprocessing
import time
from itertools import product
import math

from gurobipy.gurobipy import tupledict, LinExpr, GRB, quicksum, Model, Env

from config import PARAMETERS, EXPERIMENTAL_INSTANCES_NAME, GUROBI_PARAMETERS, LBBD_TOLERANCE, TOL
from instance import Instance
from io_operator import write_lbbd_sol, write_lbbd_json
from model import BinPacking

from warnings import warn


# from rich import print as rc_print

def print_model_nonzeros(model, keyword=None):
    if model.getAttr("Status") not in [3, 4, 5]:
        for var in model.getVars():
            if "aux" not in var.getAttr("VarName") and var.getAttr('x') != 0:
                if keyword == None:
                    print(var.getAttr("VarName"), var.getAttr('x'))
                else:
                    if keyword in var.getAttr("VarName"):
                        print(var.getAttr("VarName"), var.getAttr('x'))
    else:
        print("Either infeasible or unbounded model.\n")


class LogicBasedBendersDecomposition(Instance):

    def __init__(self, instance_name, parameters, instance_type, **exp_set):

        assert instance_type in ["LBBD", "BAC", "PA_DE", 'NO_VRP', "STC", "another_LT"], \
            Exception("Solve \"{}\" instance with solver.py".format(instance_type))

        self.use_valid_inequality = parameters.get('lbbd_use_valid_inequality', True)
        self.service_level_constraints = parameters['service_level_constraint']
        self.service_level = parameters['service_level']

        Instance.__init__(self, instance_name, instance_type, **exp_set)

        self.instance_name = instance_name
        self.parameters = parameters
        self.instance_type = instance_type
        self.exp_set = exp_set

        self.integer_transportation_decision = parameters.get('integer_transportation_decision', True)

        self.big_m = self.parameters['big_m']
        self.big_m_location_trans = {s: self.big_m for s in self.scenarios}
        self.big_m_aux_assign = self.big_m_location_trans
        self.big_m_assign_flow = {h: self.big_m for h in self.vehicle_types}
        if parameters.get('use_local_big_m', False):
            # total demand of all the demand points in scenario s \in S
            self.big_m_location_trans = {
                s: sum(self.scenarios[s][j][0] for j in self.demand_set) for s in self.scenarios}
            # for manual linearization for max() function
            self.big_m_aux_assign = self.big_m_location_trans
            # capacity of vehicle of type h \in H
            self.big_m_assign_flow = {h: self.vehicle_info[h]["cap"] for h in self.vehicle_types}

    def __str__(self):
        return "LBBD: " + self.instance_name

    def set_gurobi_parameters(self, model):
        for key, value in GUROBI_PARAMETERS.items():
            model.setParam(key, value)
        return model

    def make_empty_model(self):
        return self.set_gurobi_parameters(Model())

    def feasible_solution_heuristic(self):
        # the first-stage decisions
        def assignment():
            # assign demand point to the closest facility candidate
            demand_to_facility = {i: None for i in self.demand_set}
            for j in self.demand_set:
                distance_to_facility = float("inf")
                assign_to_facility = None
                for i in self.facility_set:
                    distance = self._get_distance(i, j)
                    if distance < distance_to_facility:
                        distance_to_facility = distance
                        assign_to_facility = i
                demand_to_facility[j] = assign_to_facility
            facility_for_demand = {i: [] for i in self.facility_set}
            for j, i in demand_to_facility.items():
                facility_for_demand[i].append(j)
            return facility_for_demand

        def worst_scenario_facility_demand(facility_for_demand):
            # scenario with maximum demand quantity
            worst_scenario_facility_demand = {
                i: max(
                    sum(self.scenarios[s][j][0] for j in facility_for_demand[i])
                    for s in self.scenarios)
                for i in self.facility_set}
            return worst_scenario_facility_demand

        def location(worst_scenario_facility_demand):
            # open facility with the lowest feasible capacity
            opened_facility_type = dict()
            for i in self.facility_set:
                facility_type = None
                facility_cap = float("inf")
                if worst_scenario_facility_demand[i] != 0:
                    for k, v in self.facility_types_info.items():
                        k_capacity = v['cap']
                        if k_capacity >= worst_scenario_facility_demand[i] and k_capacity < facility_cap:
                            facility_type = k
                            facility_cap = k_capacity
                else:
                    facility_type = None
                if facility_type != None:
                    opened_facility_type[i] = facility_type
            return opened_facility_type

        def stock(worst_scenario_facility_demand):
            # quantity of relief items prepared at facilities
            result = {key: value if value != 0 else None
                      for key, value in worst_scenario_facility_demand.items()}
            return result

        def vehicle(facility_for_demand):
            def select_vehicle():
                # use vehicles of higher capacity
                selected_vehicle = []
                for vehicle_info in [self.special_vehicle_info, self.general_vehicle_info]:
                    vehicle_type = None
                    vehicle_capacity = 0
                    for h in vehicle_info:
                        temp_vehicle_type = h
                        temp_vehicle_capacity = vehicle_info[h]['cap']
                        if temp_vehicle_capacity > vehicle_capacity:
                            vehicle_type = temp_vehicle_type
                            vehicle_capacity = temp_vehicle_capacity
                    selected_vehicle.append(vehicle_type)
                return selected_vehicle

            def FFD(items, bin_capacity, bin_number_ub):
                # First-fit decreasing
                bin_number = 0
                items = sorted(items, reverse=True)
                opened_bin_remain_cap = []
                if items:
                    bin_number += 1
                    opened_bin_remain_cap.append(bin_capacity)
                    for demand_quantity in items:
                        assigned = False
                        for i, remain_cap in enumerate(opened_bin_remain_cap):
                            if demand_quantity <= remain_cap:
                                opened_bin_remain_cap[i] -= demand_quantity
                                assigned = True
                                break
                        if not assigned:
                            bin_number += 1
                            opened_bin_remain_cap.append(bin_capacity)
                            opened_bin_remain_cap[-1] -= demand_quantity
                assert bin_number == len(opened_bin_remain_cap)
                assert bin_number <= bin_number_ub
                return bin_number

            # Solving bin packing problem with FFD (first-fit decreasing)
            special_vehicle_type, general_vehicle_type = select_vehicle()
            result = {i: 0 for i in facility_for_demand}
            for i in self.facility_set:
                isolated_bin_number = 0
                non_isolated_bin_number = 0
                for s in self.scenarios:
                    isolated_items = []
                    non_isolated_items = []
                    for j in facility_for_demand[i]:
                        demand_quantity = self.scenarios[s][j][0]
                        demand_is_isolated = self.is_isolated(s, j)
                        if demand_is_isolated:
                            isolated_items.append(demand_quantity)
                        else:
                            non_isolated_items.append(demand_quantity)

                    # for isolated points & non-isolated points
                    bin_capacity = self.vehicle_info[special_vehicle_type]['cap']
                    bin_number_ub = self.vehicle_info[special_vehicle_type]['num']
                    s_isolated_bin_number = FFD(isolated_items, bin_capacity, bin_number_ub)
                    bin_capacity = self.vehicle_info[general_vehicle_type]['cap']
                    bin_number_ub = self.vehicle_info[general_vehicle_type]['num']
                    s_non_isolated_bin_number = FFD(non_isolated_items, bin_capacity, bin_number_ub)
                    if s_isolated_bin_number > isolated_bin_number:
                        isolated_bin_number = s_isolated_bin_number
                    if s_non_isolated_bin_number > non_isolated_bin_number:
                        non_isolated_bin_number = s_non_isolated_bin_number

                result[i] = {special_vehicle_type: isolated_bin_number,
                             general_vehicle_type: non_isolated_bin_number}
            return result

        assign = assignment()
        worst_scenario_facility_demand = worst_scenario_facility_demand(assign)
        location = location(worst_scenario_facility_demand)
        stock = stock(worst_scenario_facility_demand)
        vehicle = vehicle(assign)

        solution = {'assign': tupledict(), 'location': tupledict(), 'stock': tupledict(), 'vehicle': tupledict()}
        for i, j_list in assign.items():
            for j in j_list:
                solution['assign'][(i, j)] = 1
        for i, k in location.items():
            solution['location'][(i, k)] = 1
        for i, l in stock.items():
            solution['stock'][(i,)] = l
        for i, dic in vehicle.items():
            for h, w in dic.items():
                solution['vehicle'][(i, h)] = w
        return solution

    def use_decision_vars_value(self, model, solution, fixed=False):
        model.update()
        for var_type, var_content in solution.items():
            for key, var_value in var_content.items():
                if var_value != None:
                    var_id = ','.join((str(a) for a in key))
                    var_name = "{}[{}]".format(var_type, var_id)
                    if fixed:
                        model.getVarByName(var_name).setAttr("ub", var_value)
                        model.getVarByName(var_name).setAttr("lb", var_value)
                    else:
                        model.getVarByName(var_name).setAttr("start", var_value)
        model.update()
        return model

    def _get_var_value(self, use_callback, model, vars, *indexes):
        if indexes:
            if use_callback:
                res = model.cbGetSolution(vars[indexes])
            else:
                res = vars[indexes].getAttr("x")
            res = round(res) if vars[indexes].getAttr("VType") in ['I', 'B'] else res
        else:
            if use_callback:
                res = model.cbGetSolution(vars)
            else:
                res = vars.getAttr("x")
            res = round(res) if vars.getAttr("VType") in ['I', 'B'] else res
        return res

    # """ # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # """
    # """ # # # # # # # # # # # # # #  fetch master # # # # # # # # # # # # # # """
    # """ # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # """

    def _fetch_master_recourse_var(self, master_model):
        recourse_var = master_model.getVarByName("PMP_expected_recourse")
        return recourse_var

    def _fetch_master_recourse_solution(self, master_model, pmp_callback):
        recourse_var = self._fetch_master_recourse_var(master_model)
        recourse_var_value = self._get_var_value(pmp_callback, master_model, recourse_var)
        return recourse_var_value

    def _fetch_master_recourse_vars(self, master_model):
        recourse_vars = tupledict(
            [(i, s), master_model.getVarByName("PMP_recourse[{0},{1}]".format(i, s))]
            for (i, s) in product(self.facility_set, self.scenarios))
        return recourse_vars

    def _fetch_master_recourse_solutions(self, master_model, pmp_callback):
        recourse_vars = self._fetch_master_recourse_vars(master_model)
        recourse_vars_value = tupledict(
            [(i, s), self._get_var_value(pmp_callback, master_model, recourse_vars, i, s)]
            for (i, s) in product(self.facility_set, self.scenarios))
        return recourse_vars_value

    def _fetch_preposition_vars(self, master_model):
        assign_vars = tupledict(
            [(i, j), master_model.getVarByName("PMP_assign[{0},{1}]".format(i, j))]
            for (i, j) in product(self.facility_set, self.demand_set))
        location_vars = tupledict(
            [(i, k), master_model.getVarByName("PMP_location[{0},{1}]".format(i, k))]
            for (i, k) in product(self.facility_set, self.facility_types))
        stock_vars = tupledict(
            [(i,), master_model.getVarByName("PMP_stock[{0}]".format(i))]
            for i in self.facility_set)
        vehicle_vars = tupledict(
            [(i, h), master_model.getVarByName("PMP_vehicle[{0},{1}]".format(i, h))]
            for (i, h) in product(self.facility_set, self.vehicle_types))

        return assign_vars, location_vars, stock_vars, vehicle_vars

    def _fetch_preposition_solution(self, master_model, pmp_callback):
        assign_vars, location_vars, stock_vars, vehicle_vars = self._fetch_preposition_vars(master_model)

        assign_vars_value = tupledict(
            [(i, j), self._get_var_value(pmp_callback, master_model, assign_vars, i, j)]
            for (i, j) in product(self.facility_set, self.demand_set))
        location_vars_value = tupledict(
            [(i, k), self._get_var_value(pmp_callback, master_model, location_vars, i, k)]
            for (i, k) in product(self.facility_set, self.facility_types))
        stock_vars_value = tupledict(
            [(i), self._get_var_value(pmp_callback, master_model, stock_vars, i)]
            for i in self.facility_set)
        vehicle_vars_value = tupledict(
            [(i, h), self._get_var_value(pmp_callback, master_model, vehicle_vars, i, h)]
            for (i, h) in product(self.facility_set, self.vehicle_types))

        return assign_vars_value, location_vars_value, stock_vars_value, vehicle_vars_value

    def _get_assign(self, assign_vars_value):
        assign = {i: set() for i in self.facility_set}
        isolated_assign = {s: {i: set() for i in self.facility_set} for s in self.scenarios}
        result = {s: {i: dict() for i in self.facility_set} for s in self.scenarios}
        isolated_result = {s: {i: dict() for i in self.facility_set} for s in self.scenarios}
        for i, j in assign_vars_value:
            j_is_assigned_to_i = round(assign_vars_value[i, j])
            if j_is_assigned_to_i:
                assign[i].add(j)
                for s in self.scenarios:
                    result[s][i][j] = self.scenarios[s][j][0]
                    if self.scenarios[s][j][1]:
                        isolated_assign[s][i].add(j)
                        isolated_result[s][i][j] = self.scenarios[s][j][0]
        return assign, isolated_assign, result, isolated_result

    # """ # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # """
    # """ # # # # # # # # # # # fetch resource allocation # # # # # # # # # # # """
    # """ # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # """

    def _fetch_RASP_vars(self, resource_allocation_model):
        direct_trans_vars = tupledict(
            [(i, j, s),
             resource_allocation_model.getVarByName("RASP_direct_trans[{0},{1},{2}]".format(i, j, s))]
            for (i, j, s) in product(self.facility_set, self.demand_set, self.scenarios))
        lateral_trans_vars = tupledict(
            [(i, k, j, s),
             resource_allocation_model.getVarByName("RASP_lateral_trans[{0},{1},{2},{3}]".format(i, k, j, s))]
            for (i, k, j, s) in product(self.facility_set, self.facility_set, self.demand_set, self.scenarios))
        vehicle_vars = tupledict(
            [(i, s, h),
             resource_allocation_model.getVarByName("RASP_vehicle[{0},{1},{2}]".format(i, s, h))]
            for (i, s, h) in product(self.facility_set, self.scenarios, self.vehicle_types))

        return direct_trans_vars, lateral_trans_vars, vehicle_vars

    def _fetch_RASP_solution(self, resource_allocation_model, rasp_callback):
        direct_trans_vars, lateral_trans_vars, vehicle_vars \
            = self._fetch_RASP_vars(resource_allocation_model)

        direct_trans_vars_value = tupledict(
            [(i, j, s),
             self._get_var_value(rasp_callback, resource_allocation_model, direct_trans_vars, i, j, s)]
            for (i, j, s) in product(self.facility_set, self.demand_set, self.scenarios))
        lateral_trans_vars_value = tupledict(
            [(i, k, j, s),
             self._get_var_value(rasp_callback, resource_allocation_model, lateral_trans_vars, i, k, j, s)]
            for (i, k, j, s) in product(self.facility_set, self.facility_set, self.demand_set, self.scenarios))
        vehicle_vars_value = tupledict(
            [(i, s, h),
             self._get_var_value(rasp_callback, resource_allocation_model, vehicle_vars, i, s, h)]
            for (i, s, h) in product(self.facility_set, self.scenarios, self.vehicle_types))
        return direct_trans_vars_value, lateral_trans_vars_value, vehicle_vars_value

    def _fetch_RASP_recourse_var(self, resource_allocation_model):
        recourse_var = resource_allocation_model.getVarByName("RASP_expected_recourse")
        return recourse_var

    def _fetch_RASP_recourse_solution(self, resource_allocation_model, rasp_callback):
        recourse_var = self._fetch_RASP_recourse_var(resource_allocation_model)
        recourse_var_value = self._get_var_value(rasp_callback, resource_allocation_model, recourse_var)
        return recourse_var_value

    def _fetch_RASP_recourse_vars(self, resource_allocation_model):
        recourse_vars = tupledict(
            [(i, s), resource_allocation_model.getVarByName("RASP_recourse[{0},{1}]".format(i, s))]
            for (i, s) in product(self.facility_set, self.scenarios))
        return recourse_vars

    def _fetch_RASP_recourse_solutions(self, resource_allocation_model, rasp_callback):
        recourse_vars = self._fetch_RASP_recourse_vars(resource_allocation_model)
        recourse_vars_value = tupledict(
            [(i, s),
             self._get_var_value(rasp_callback, resource_allocation_model, recourse_vars, i, s)]
            for (i, s) in product(self.facility_set, self.scenarios))
        return recourse_vars_value

    def _fetch_RASP_aux_vars(self, resource_allocation_model):
        aux_direct_reduce_vars = tupledict(
            [(i, j, s),
             resource_allocation_model.getVarByName("RASP_aux_direct_reduce[{0},{1},{2}]".format(i, j, s))]
            for (i, j, s) in product(self.facility_set, self.demand_set, self.scenarios))
        aux_lateral_reduce_vars = tupledict(
            [(i, i_, s),
             resource_allocation_model.getVarByName("RASP_aux_lateral_reduce[{0},{1},{2}]".format(i, i_, s))]
            for (i, i_, s) in product(self.facility_set, self.facility_set, self.scenarios))

        aux_direct_recourse_reduce_vars = tupledict(
            [(i, j, s),
             resource_allocation_model.getVarByName("RASP_aux_direct_recourse_reduce[{0},{1},{2}]".format(i, j, s))]
            for (i, j, s) in product(self.facility_set, self.demand_set, self.scenarios))
        aux_lateral_recourse_reduce_vars = tupledict(
            [(i, i_, s),
             resource_allocation_model.getVarByName("RASP_aux_lateral_recourse_reduce[{0},{1},{2}]".format(i, i_, s))]
            for (i, i_, s) in product(self.facility_set, self.facility_set, self.scenarios))

        aux_direct_recourse_bin_vars = tupledict(
            [(i, j, s),
             resource_allocation_model.getVarByName("RASP_aux_direct_recourse_bin[{0},{1},{2}]".format(i, j, s))]
            for (i, j, s) in product(self.facility_set, self.demand_set, self.scenarios))
        aux_lateral_recourse_bin_vars = tupledict(
            [(i, i_, s),
             resource_allocation_model.getVarByName("RASP_aux_lateral_recourse_bin[{0},{1},{2}]".format(i, i_, s))]
            for (i, i_, s) in product(self.facility_set, self.facility_set, self.scenarios))

        return aux_direct_reduce_vars, aux_lateral_reduce_vars, \
            aux_direct_recourse_reduce_vars, aux_lateral_recourse_reduce_vars, \
            aux_direct_recourse_bin_vars, aux_lateral_recourse_bin_vars

    def _get_depots_customers(self, direct_trans_vars_value, lateral_trans_vars_value):
        result = {s: {i: dict() for i in self.facility_set} for s in self.scenarios}
        for s, i in product(self.scenarios, self.facility_set):
            result[s][i][i] = 0
            for p in self.demand_set:
                quantity = direct_trans_vars_value[i, p, s]
                quantity += sum(lateral_trans_vars_value[k, i, p, s] for k in self.facility_set)
                if quantity != 0:
                    result[s][i][p] = quantity
            for p in self.facility_set:
                quantity = 0
                if p != i:
                    quantity = sum(lateral_trans_vars_value[i, p, k, s] for k in self.demand_set)
                if quantity != 0:
                    result[s][i][p] = quantity
        return result

    def _get_vehicle_preparation(self, vehicle_vars_value):
        result = {i:
                      {s:
                           {h: None for h in self.vehicle_types}
                       for s in self.scenarios}
                  for i in self.facility_set}

        for i, s, h in vehicle_vars_value:
            vehicle_num = vehicle_vars_value[i, s, h]
            result[i][s][h] = vehicle_num
        return result

    # """ # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # """
    # """ # # # # # # # # # # # fetch vehicle dispatching # # # # # # # # # # # """
    # """ # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # """

    def _fetch_VDSP_recourse_var(self, vehicle_dispatching_model):
        recourse_var = vehicle_dispatching_model.getVarByName("VDSP_recourse")
        return recourse_var

    def _fetch_VDSP_recourse_solution(self, vehicle_dispatching_model, vdsp_callback):
        recourse_var = self._fetch_VDSP_recourse_var(vehicle_dispatching_model)
        recourse_var_value = self._get_var_value(vdsp_callback, vehicle_dispatching_model, recourse_var)
        return recourse_var_value

    # """ # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # """
    # """ # # # # # # # # # # # # # # make models # # # # # # # # # # # # # # # """
    # """ # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # """

    def preposition_master_model(self, heuristic_solution=None, fix_first_stage=False):
        model = self.make_empty_model()

        # add decision variables
        assign_vars_tup = tupledict([(i, j), -1] for (i, j) in product(self.facility_set, self.demand_set))
        location_vars_tup = tupledict([(i, k), -1] for i in self.facility_set for k in self.facility_types)
        stock_vars_tup = tupledict([(i), -1] for i in self.facility_set)
        vehicle_vars_tup = tupledict(
            [(i, h), -1] for (i, h) in product(self.facility_set, self.vehicle_types))
        recourse_vars_tup = tupledict([(i, s), -1] for (i, s) in product(self.facility_set, self.scenarios))

        assign_vars = model.addVars(assign_vars_tup, vtype=GRB.BINARY, name="PMP_assign")
        location_vars = model.addVars(location_vars_tup, vtype=GRB.BINARY, name="PMP_location")
        # MODELING - fixing numerical issue that make RASP infeasible in BAC, vtype=GRB.CONTINUOUS -> GRB.INTEGER
        stock_vars = model.addVars(stock_vars_tup, vtype=GRB.INTEGER, lb=0, name="PMP_stock")

        vehicle_vars = model.addVars(vehicle_vars_tup, vtype=GRB.INTEGER, lb=0, name="PMP_vehicle")
        recourse_vars = model.addVars(recourse_vars_tup, vtype=GRB.CONTINUOUS, lb=0, name="PMP_recourse")
        recourse_var = model.addVar(vtype=GRB.CONTINUOUS, lb=0, name="PMP_expected_recourse")

        # set objective
        location_cost = LinExpr([self.facility_open_cost[i][k] for i, k in location_vars],
                                [location_vars[i, k] for i, k in location_vars])
        stock_cost = LinExpr([self.unit_facility_acquisition_cost for _ in stock_vars],
                             [stock_vars[i] for i in stock_vars])
        vehicle_cost = LinExpr([self.vehicle_info[h]['fc'] for (i, h) in vehicle_vars],
                               [vehicle_vars[i, h] for (i, h) in vehicle_vars])
        first_stage_cost = location_cost + stock_cost + vehicle_cost
        model.setObjective(first_stage_cost + recourse_var)

        model.addConstr(recourse_var ==
                        quicksum(recourse_vars.sum('*', s) * self.scenario_prob[s] for s in self.scenarios))

        # add constraints
        model.addConstrs(
            location_vars.sum(i, '*') <= 1 for i in self.facility_set)
        model.addConstrs(
            stock_vars[i] <= quicksum(location_vars[i, k] * self.facility_cap_types[k] for k in self.facility_types)
            for i in self.facility_set)
        model.addConstrs(
            assign_vars.sum('*', j) == 1 for j in self.demand_set)
        model.addConstrs(
            assign_vars[i, j] <= location_vars.sum(i, '*') for i in self.facility_set for j in self.demand_set)
        # MODELING - (4.6) temporarily fixing wrong recourse cost approximation (LB > UB, wrong optimal solution)
        # model.addConstrs(vehicle_vars.sum('*', h) <= self.vehicle_info[h]['num']
        #                  for h in self.vehicle_types)

        # add valid inequalities
        if self.use_valid_inequality:
            model.addConstrs(
                location_vars.sum(i, '*') <= assign_vars.sum(i, '*')
                for i in self.facility_set)
            # MODELING - (4.10) temporarily fixing wrong recourse cost approximation (LB > UB, wrong optimal solution)
            # model.addConstrs(
            #     vehicle_vars[i, h] <= assign_vars.sum(i, '*')
            #     for i, h in product(self.facility_set, self.vehicle_types))
            model.addConstrs(
                vehicle_vars[i, h] == assign_vars.sum(i, '*')
                for i, h in product(self.facility_set, self.vehicle_types))

        # add constraints
        # MODELING - (4.11)
        model.addConstrs(
            quicksum(vehicle_vars[i, h] * self.vehicle_info[h]['cap'] for h in self.vehicle_types)
            >= stock_vars[i] for i in self.facility_set)
        model.addConstrs(
            quicksum(vehicle_vars[i, h] * self.vehicle_info[h]['cap']
                     for h in self.vehicle_types)
            >= quicksum(math.ceil(self.service_level * self.scenarios[s][j][0]) * assign_vars[i, j]
                        for j in self.demand_set)
            for i, s in product(self.facility_set, self.scenarios))
        model.addConstrs(
            quicksum(vehicle_vars[i, h] * self.vehicle_info[h]['cap']
                     for h in self.special_vehicle_types)
            >= quicksum(math.ceil(self.service_level * self.scenarios[s][j][0]) * assign_vars[i, j]
                        for j in self.isolated_demand_in_scenario[s])
            for i, s in product(self.facility_set, self.scenarios))
        model.addConstr(
            stock_vars.sum('*') >=
            max(sum(math.ceil(self.service_level * self.scenarios[s][j][0]) for j in self.scenarios[s])
                for s in self.scenarios))
        model.addConstrs(
            stock_vars[i] >=
            quicksum(math.ceil(self.scenarios[s][j][0] * self.service_level) * assign_vars[i, j]
                     for j in self.demand_set)
            for i, s in product(self.facility_set, self.scenarios))

        # recourse cost LB and UB enhancement
        # min_vehicle_uc = min(a['uc'] for a in self.vehicle_info.values())
        # model.addConstrs(
        #     recourse_vars[i, s] >= 2 / self.vehicle_capacity *
        #     quicksum(assign_vars[i,j] * self.scenarios[s][j][0] for j in self.demand_set) *
        #     min_vehicle_uc * self.get_closest_to(i, self.demand_set)
        #     for i, s in product(self.facility_set, self.scenarios)
        # )
        # max_vehicle_uc = max(a['uc'] for a in self.vehicle_info.values())
        # model.addConstrs(
        #     recourse_vars[i, s] <= quicksum(
        #         assign_vars[i,j] * max_vehicle_uc * self._get_distance(i,j) for j in self.demand_set)
        #     for i, s in product(self.facility_set, self.scenarios)
        # )

        if heuristic_solution:
            # use heuristic master problem solution ("assign", "location", "stock")
            model = self.use_decision_vars_value(model, heuristic_solution, fix_first_stage)

        model.update()
        return model

    def _make_resource_allocation_sub_problem(self, master_model, pmp_callback, separate, s_id):
        if not separate:
            assert s_id == None
            s_set = self.scenarios
        else:
            assert s_id in self.scenarios
            s_set = {s_id: self.scenarios[s_id]}

        model = self.make_empty_model()

        # ---------- KNOWN 1ST STAGE DECISIONS ----------
        # x_{ij}     assign_vars              bin  <-  preposition_master_problem
        # y_{ik}     location_vars            bin  <-  preposition_master_problem
        # l_i        stock_vars               int  <-  preposition_master_problem

        assign_vars_value, location_vars_value, \
        stock_vars_value, vehicle_vars_value = self._fetch_preposition_solution(master_model, pmp_callback)

        # add variables

        # ------- UNKNOWN 2(.1)ND STAGE DECISIONS -------
        # t_{ij}^s   direct_trans_vars        int
        # t_{ii'j}^s lateral_trans_vars       int
        # w_i^{sh}   vehicle_vars             int

        direct_trans_vars_tup = tupledict(
            [(i, j, s), -1] for (i, j, s) in product(self.facility_set, self.demand_set, s_set))
        lateral_trans_vars_tup = tupledict(
            [(i, k, j, s), -1] for (i, k, j, s) in
            product(self.facility_set, self.facility_set, self.demand_set, s_set))
        vehicle_vars_tup = tupledict(
            [(i, s, h), -1]
            for (i, s, h) in product(self.facility_set, s_set, self.vehicle_types))
        recourse_vars_tup = tupledict(
            [(i, s), -1]
            for (i, s) in product(self.facility_set, s_set))
        aux_direct_reduce_tup = tupledict(
            [(i, j, s), -1]
            for (i, j, s) in product(self.facility_set, self.demand_set, s_set))
        aux_lateral_reduce_tup = tupledict(
            [(i, i_, s), -1]
            for (i, i_, s) in product(self.facility_set, self.facility_set, s_set))
        aux_direct_recourse_reduce_tup = tupledict(
            [(i, j, s), -1]
            for (i, j, s) in product(self.facility_set, self.demand_set, s_set))
        aux_lateral_recourse_reduce_tup = tupledict(
            [(i, i_, s), -1]
            for (i, i_, s) in product(self.facility_set, self.facility_set, s_set))

        variable_type = GRB.INTEGER if self.integer_transportation_decision else GRB.CONTINUOUS
        direct_trans_vars = model.addVars(direct_trans_vars_tup, vtype=variable_type, lb=0, name="RASP_direct_trans")
        lateral_trans_vars = model.addVars(lateral_trans_vars_tup, vtype=variable_type, ub=0, name="RASP_lateral_trans")
        vehicle_vars = model.addVars(vehicle_vars_tup, vtype=GRB.INTEGER, lb=0, name="RASP_vehicle")
        recourse_vars = model.addVars(recourse_vars_tup, vtype=GRB.CONTINUOUS, lb=0, name="RASP_recourse")
        recourse_var = model.addVar(vtype=GRB.CONTINUOUS, lb=0, name="RASP_expected_recourse")

        aux_direct_reduce_vars = model.addVars(
            aux_direct_reduce_tup, vtype=GRB.BINARY, name="RASP_aux_direct_reduce")
        aux_lateral_reduce_vars = model.addVars(
            aux_lateral_reduce_tup, vtype=GRB.BINARY, name="RASP_aux_lateral_reduce")
        aux_direct_recourse_reduce_vars = model.addVars(
            aux_direct_recourse_reduce_tup,
            vtype=GRB.CONTINUOUS, lb=0, name="RASP_aux_direct_recourse_reduce")
        aux_lateral_recourse_reduce_vars = model.addVars(
            aux_lateral_recourse_reduce_tup,
            vtype=GRB.CONTINUOUS, lb=0, name="RASP_aux_lateral_recourse_reduce")
        aux_direct_recourse_bin_vars = model.addVars(
            aux_direct_recourse_reduce_tup,
            vtype=GRB.BINARY, name="RASP_aux_direct_recourse_bin")
        aux_lateral_recourse_bin_vars = model.addVars(
            aux_lateral_recourse_reduce_tup,
            vtype=GRB.BINARY, name="RASP_aux_lateral_recourse_bin")

        # set objective
        model.setObjective(recourse_var)
        model.addConstr(
            recourse_var ==
            quicksum(recourse_vars.sum('*', s) * self.scenario_prob[s] for s in s_set))

        # add constraints
        model.addConstrs(
            direct_trans_vars[i, j, s] <= self.big_m_location_trans[s] * assign_vars_value[i, j]
            for (i, j, s) in direct_trans_vars)
        model.addConstrs(
            lateral_trans_vars[i, k, j, s] <= self.big_m_location_trans[s] * assign_vars_value[k, j]
            for (i, k, j, s) in lateral_trans_vars)
        model.addConstrs(
            lateral_trans_vars.sum(i, k, '*', s) <= self.big_m_location_trans[s] * location_vars_value.sum(i, '*')
            for (i, k, s) in product(self.facility_set, self.facility_set, s_set))
        model.addConstrs(
            lateral_trans_vars.sum(i, k, '*', s) <= self.big_m_location_trans[s] * location_vars_value.sum(k, '*')
            for (i, k, s) in product(self.facility_set, self.facility_set, s_set))
        model.addConstrs(
            direct_trans_vars.sum(i, '*', s) + lateral_trans_vars.sum(i, '*', '*', s) <= stock_vars_value[i]
            for (i, s) in product(self.facility_set, s_set))
        # MODELING - potentially make RASP infeasible because of transportation decision conflict
        # model.addConstrs(
        #     direct_trans_vars.sum("*", j, s) + lateral_trans_vars.sum("*", "*", j, s) <= s_set[s][j][0]
        #     for (j, s) in product(self.demand_set, s_set))
        # vehicle number lower bound
        model.addConstrs(
            quicksum(vehicle_vars[i, s, h] * self.vehicle_info[h]["cap"] for h in self.vehicle_types) >=
            direct_trans_vars.sum(i, '*', s) +
            lateral_trans_vars.sum('*', i, '*', s) + lateral_trans_vars.sum(i, '*', '*', s)
            for i, s in product(self.facility_set, s_set))
        # special vehicle number lower bound
        model.addConstrs(
            quicksum(vehicle_vars[i, s, h] * self.vehicle_info[h]["cap"] for h in self.special_vehicle_types) >=
            quicksum(direct_trans_vars[i, j, s] for j in self.isolated_demand_in_scenario[s]) +
            quicksum(lateral_trans_vars[k, i, j, s] for k, j in
                     product(self.facility_set, self.isolated_demand_in_scenario[s]))
            for i, s in product(self.facility_set, s_set))
        # separating constraint
        # MODELING - temporarily fix the invalid master optimality cut issue
        model.addConstrs(
            vehicle_vars[i, s, h] == vehicle_vars_value[i, h]
            for (i, s, h) in product(self.facility_set, s_set, self.vehicle_types))
        # add service level constraints
        if self.service_level_constraints:
            model.addConstrs(
                direct_trans_vars.sum("*", j, s) + lateral_trans_vars.sum("*", "*", j, s) >=
                math.ceil(s_set[s][j][0] * self.service_level) for (j, s) in product(self.demand_set, s_set))

        # add valid inequalities
        if self.use_valid_inequality:
            # forbidden self transportation
            model.addConstrs(lateral_trans_vars.sum(i, i, '*', '*') == 0 for i in self.facility_set)

        model.update()
        return model

    def resource_allocation_model(self, master_model, pmp_callback):
        return self._make_resource_allocation_sub_problem(master_model, pmp_callback, False, None)

    def _make_vehicle_dispatching_sub_problem(self, model, depots_customers, vehicle_preparation, i, s):
        # model = self.make_empty_model()
        node_set = [key for key in depots_customers[s][i]]
        customer_set = [key for key in depots_customers[s][i] if key != i]

        # add decision vars
        # ----- UNKNOWN 2(.2)ND STAGE DECISIONS ------
        # z_{pq}^(h)     routing              bin
        # f_{pq}^(h)     commodity_flow       int
        routing_vars_tup = tupledict(
            [(p, q, h), -1] for (p, q, h) in product(node_set, node_set, self.vehicle_types))
        commodity_flow_vars_tup = tupledict(
            [(p, q, h), -1] for (p, q, h) in product(node_set, node_set, self.vehicle_types))

        routing_vars = model.addVars(routing_vars_tup, vtype=GRB.BINARY, name="VDSP_routing")
        commodity_flow_vars = model.addVars(commodity_flow_vars_tup, vtype=GRB.INTEGER, lb=0, name="VDSP_flow")
        recourse_var = model.addVar(vtype=GRB.CONTINUOUS, lb=0, name="VDSP_recourse")

        # set objective
        vehicle_traveling_cost = LinExpr(
            [self.vehicle_info[h]['uc'] * self._get_distance(p, q) for p, q, h in routing_vars_tup],
            [routing_vars[p, q, h] for p, q, h in routing_vars_tup]
        )
        model.setObjective(vehicle_traveling_cost)
        model.addConstr(recourse_var == vehicle_traveling_cost)

        # add constraints
        model.addConstrs(
            (quicksum(routing_vars[i, q, h] for q in customer_set) <= vehicle_preparation[i][s][h]
             for h in self.vehicle_types), name='vehicle_num_cons')
        model.addConstrs(
            routing_vars.sum('*', q, '*') == 1 for q in customer_set)
        model.addConstrs(
            quicksum(routing_vars.sum('*', q, h) for h in self.special_vehicle_types) == 1
            for q in [key for key in depots_customers[s][i] if self.is_isolated(s, key)])
        model.addConstrs(
            routing_vars.sum('*', q, h) - routing_vars.sum(q, '*', h) == 0
            for q in customer_set for h in self.vehicle_types)
        model.addConstrs(
            quicksum(commodity_flow_vars.sum(q, p, '*') for q in node_set) -
            quicksum(commodity_flow_vars.sum(p, q, '*') for q in node_set)
            == depots_customers[s][i][p]
            for p in customer_set)
        model.addConstrs(
            commodity_flow_vars[p, q, h] >= depots_customers[s][i][q] * routing_vars[p, q, h]
            for p, q, h in product(node_set, node_set, self.vehicle_types))
        model.addConstrs(
            commodity_flow_vars[p, q, h] <=
            (self.vehicle_info[h]['cap'] - depots_customers[s][i][p]) * routing_vars[p, q, h]
            for p, q, h in product(node_set, node_set, self.vehicle_types))

        if self.use_valid_inequality:
            model.addConstrs(
                commodity_flow_vars[p, i, h] == 0
                for p, h in product(customer_set, self.vehicle_types))

            model.addConstrs(routing_vars[i, i, h] == 0 for h in self.vehicle_types)

        model.update()
        return model, True if customer_set else False

    def vehicle_dispatching_model(
            self, model, direct_trans_vars_value, lateral_trans_vars_value, vehicle_vars_value, i, s):
        # NOTICE - for multiprocessing
        direct_trans_vars_value = tupledict(direct_trans_vars_value)
        lateral_trans_vars_value = tupledict(lateral_trans_vars_value)
        vehicle_vars_value = tupledict(vehicle_vars_value)

        # -------- KNOWN 2(.1)ND STAGE DECISIONS --------
        # t_{ij}^s   direct_trans_vars        int  <-  resource_allocation_sub_problem
        # t_{ii'j}^s lateral_trans_vars       int  <-  resource_allocation_sub_problem
        # w_i^h      vehicle_vars             int  <-  preposition_master_problem

        depots_customers = self._get_depots_customers(direct_trans_vars_value, lateral_trans_vars_value)
        vehicle_preparation = self._get_vehicle_preparation(vehicle_vars_value)

        vehicle_dispatching_model, non_empty = \
            self._make_vehicle_dispatching_sub_problem(model, depots_customers, vehicle_preparation, i, s)
        return depots_customers[s][i], vehicle_preparation[i], vehicle_dispatching_model, non_empty

    # """ # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # """
    # """ # # # # # # # # # # #  # add Bender's cut # # # ## # #  # # # # # # # """
    # """ # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # """

    def _add_cut(self, model, cut_plane, add_as_lazy_cut):
        if add_as_lazy_cut:
            model.cbLazy(cut_plane >= 0)
        else:
            model.addConstr(cut_plane >= 0)
        model.update()
        return model

    def _add_feasibility_cut_to_resource_allocation(
            self, i, s, u,
            resource_allocation_model, pre_direct_trans_vars_value,
            pre_lateral_trans_vars_value,
            depot_customers,
            rasp_callback):
        assert not rasp_callback
        # variables: w_i^h, t_{ij}^s, t_{ii'j}^s
        # data: BP(t(u)_{ij}^s, t(u)_{ii'j}^s), t(u)_{ij}^s, t(u)_{ii'j}^s

        # sets
        customer = {
            a: depot_customers[a] for a in depot_customers if a in self.demand_set}
        facility_customer = {
            a: depot_customers[a] for a in depot_customers if a in self.facility_set and a != i}
        isolated_customer = {
            a: depot_customers[a] for a in depot_customers if a in self.isolated_demand_in_scenario[s]}

        # data
        all_bin_number = BinPacking(depot_customers, self.vehicle_capacity).mip_solve()
        special_bin_number = BinPacking(isolated_customer, self.vehicle_capacity).mip_solve()

        all_vehicle_RHS = LinExpr(all_bin_number)
        special_vehicle_RHS = LinExpr(special_bin_number)

        # variables
        direct_trans_vars, lateral_trans_vars, vehicle_vars = self._fetch_RASP_vars(resource_allocation_model)

        # global aux_reduce_vars
        # aux_direct_reduce_vars, aux_lateral_reduce_vars \
        #     = self._fetch_RASP_aux_vars(resource_allocation_model)[:2]

        # local aux_reduce_vars
        aux_direct_reduce_tup = tupledict(
            [(i, j, s, u), -1]
            for (i, j, s) in product(self.facility_set, self.demand_set, self.scenarios))
        aux_lateral_reduce_tup = tupledict(
            [(i, i_, s, u), -1]
            for (i, i_, s) in product(self.facility_set, self.facility_set, self.scenarios))
        aux_direct_reduce_vars = \
            resource_allocation_model.addVars(aux_direct_reduce_tup, vtype=GRB.BINARY, name="RASP_aux_direct_reduce")
        aux_lateral_reduce_vars = \
            resource_allocation_model.addVars(aux_lateral_reduce_tup, vtype=GRB.BINARY, name="RASP_aux_lateral_reduce")

        if customer:
            all_vehicle_RHS -= quicksum(aux_direct_reduce_vars[i, j, s, u] for j in customer)
            for j in customer:
                resource_allocation_model.addConstr(
                    (aux_direct_reduce_vars[i, j, s, u] == 0) >>
                    (direct_trans_vars[i, j, s] >= pre_direct_trans_vars_value[i, j, s])
                )
                resource_allocation_model.addConstr(
                    (aux_direct_reduce_vars[i, j, s, u] == 1) >>
                    (direct_trans_vars[i, j, s] <= pre_direct_trans_vars_value[i, j, s] - 1)
                )

        if facility_customer:
            all_vehicle_RHS -= quicksum(aux_lateral_reduce_vars[i, i_, s, u] for i_ in facility_customer)
            for i_ in facility_customer:
                resource_allocation_model.addConstr(
                    (aux_lateral_reduce_vars[i, i_, s, u] == 0) >>
                    (lateral_trans_vars.sum(i, i_, '*', s) >= pre_lateral_trans_vars_value.sum(i, i_, '*', s))
                )
                resource_allocation_model.addConstr(
                    (aux_lateral_reduce_vars[i, i_, s, u] == 1) >>
                    (lateral_trans_vars.sum(i, i_, '*', s) <= pre_lateral_trans_vars_value.sum(i, i_, '*', s) - 1)
                )

        if customer or facility_customer:
            resource_allocation_model.addConstr(
                quicksum(vehicle_vars[i, s, h] for h in self.vehicle_types) >= all_vehicle_RHS)

        if isolated_customer:
            special_vehicle_RHS -= quicksum(aux_direct_reduce_vars[i, j, s, u] for j in isolated_customer)
            resource_allocation_model.addConstr(
                quicksum(vehicle_vars[i, s, h] for h in self.special_vehicle_types) >= special_vehicle_RHS)

        resource_allocation_model.update()
        return resource_allocation_model

    def _add_optimality_cut_to_resource_allocation(
            self, i, s, u,
            resource_allocation_model, pre_direct_trans_vars_value,
            pre_lateral_trans_vars_value,
            depot_customers, recourse_RHS,
            vdsp_callback):

        assert not vdsp_callback

        # Sets
        customer = {a: depot_customers[a] for a in depot_customers if a in self.demand_set}
        facility_customer = {a: depot_customers[a] for a in depot_customers if a in self.facility_set and a != i}
        points_set = {a for a in customer}

        # Data: VDSP_recourse, pre_direct_trans_vars_value, pre_lateral_trans_vars_value
        furthest_distance = self.get_furthest_distance(points_set)
        max_vehicle_unit_cost = max(self.vehicle_info[h]['uc'] for h in self.vehicle_info)
        max_reduction = 2 * furthest_distance * max_vehicle_unit_cost

        # Variables: R_{is}, t_{ij}^s, t_{ii'j}^s
        direct_trans_vars, lateral_trans_vars, vehicle_vars = self._fetch_RASP_vars(resource_allocation_model)

        # NOTICE - bellowing add auxiliary variables globally
        # aux_direct_recourse_reduce_vars, aux_lateral_recourse_reduce_vars, \
        # aux_direct_recourse_bin_vars, aux_lateral_recourse_bin_vars \
        #     = self._fetch_RASP_aux_vars(resource_allocation_model)[2:]
        # for j in customer:
        #     recourse_RHS -= aux_direct_recourse_reduce_vars[i, j, s]
        #     # Using indicator constraints (binary variable == 0 or 1 => some constraint)
        #     resource_allocation_model.addConstr(
        #         (aux_direct_recourse_bin_vars[i, j, s] == 0) >>
        #         (direct_trans_vars[i, j, s] >= pre_direct_trans_vars_value[i, j, s]))
        #     resource_allocation_model.addConstr(
        #         (aux_direct_recourse_bin_vars[i, j, s] == 0) >>
        #         (aux_direct_recourse_reduce_vars[i, j, s] == 0))
        #     resource_allocation_model.addConstr(
        #         (aux_direct_recourse_bin_vars[i, j, s] == 1) >>
        #         (direct_trans_vars[i, j, s] <= pre_direct_trans_vars_value[i, j, s] - 1))
        #     resource_allocation_model.addConstr(
        #         (aux_direct_recourse_bin_vars[i, j, s] == 1) >>
        #         (aux_direct_recourse_reduce_vars[i, j, s] == max_reduction))
        # for i_ in facility_customer:
        #     # Using indicator constraints (binary variable == 0 or 1 => some constraint)
        #     recourse_RHS -= aux_lateral_recourse_reduce_vars[i, i_, s]
        #     resource_allocation_model.addConstr(
        #         (aux_lateral_recourse_bin_vars[i, i_, s] == 0) >>
        #         (lateral_trans_vars.sum(i, i_, '*', s) >= pre_lateral_trans_vars_value.sum(i, i_, '*', s)))
        #     resource_allocation_model.addConstr(
        #         (aux_lateral_recourse_bin_vars[i, i_, s] == 0) >>
        #         (aux_lateral_recourse_reduce_vars[i, i_, s] == 0))
        #     resource_allocation_model.addConstr(
        #         (aux_lateral_recourse_bin_vars[i, i_, s] == 1) >>
        #         (lateral_trans_vars.sum(i, i_, '*', s) <= pre_lateral_trans_vars_value.sum(i, i_, '*', s) - 1))
        #     resource_allocation_model.addConstr(
        #         (aux_lateral_recourse_bin_vars[i, i_, s] == 1) >>
        #         (aux_lateral_recourse_reduce_vars[i, i_, s] == max_reduction))

        # NOTICE - bellowing add auxiliary variables in each iteration u
        aux_direct_recourse_reduce_tup = tupledict(
            [(i, j, s, u), -1] for (i, j, s) in product(self.facility_set, self.demand_set, self.scenarios))
        aux_lateral_recourse_reduce_tup = tupledict(
            [(i, i_, s, u), -1] for (i, i_, s) in product(self.facility_set, self.facility_set, self.scenarios))
        aux_direct_recourse_reduce_vars = resource_allocation_model.addVars(
            aux_direct_recourse_reduce_tup, vtype=GRB.CONTINUOUS, lb=0, name="RASP_aux_direct_recourse_reduce")
        aux_lateral_recourse_reduce_vars = resource_allocation_model.addVars(
            aux_lateral_recourse_reduce_tup, vtype=GRB.CONTINUOUS, lb=0, name="RASP_aux_lateral_recourse_reduce")
        aux_direct_recourse_bin_vars = resource_allocation_model.addVars(
            aux_direct_recourse_reduce_tup, vtype=GRB.BINARY, name="RASP_aux_direct_recourse_bin")
        aux_lateral_recourse_bin_vars = resource_allocation_model.addVars(
            aux_lateral_recourse_reduce_tup, vtype=GRB.BINARY, name="RASP_aux_lateral_recourse_bin")

        # NOTICE - can only be implemented with classical LBBD method
        #   the lower echelon problem is implemented with classical LBBD method.
        for j in customer:
            recourse_RHS -= aux_direct_recourse_reduce_vars[i, j, s, u]
            resource_allocation_model.addConstr(
                (aux_direct_recourse_bin_vars[i, j, s, u] == 0) >>
                (direct_trans_vars[i, j, s] >= pre_direct_trans_vars_value[i, j, s]))
            resource_allocation_model.addConstr(
                (aux_direct_recourse_bin_vars[i, j, s, u] == 0) >>
                (aux_direct_recourse_reduce_vars[i, j, s, u] == 0))
            resource_allocation_model.addConstr(
                (aux_direct_recourse_bin_vars[i, j, s, u] == 1) >>
                (direct_trans_vars[i, j, s] <= pre_direct_trans_vars_value[i, j, s] - 1))
            resource_allocation_model.addConstr(
                (aux_direct_recourse_bin_vars[i, j, s, u] == 1) >>
                (aux_direct_recourse_reduce_vars[i, j, s, u] == max_reduction))

        for i_ in facility_customer:
            recourse_RHS -= aux_lateral_recourse_reduce_vars[i, i_, s, u]
            resource_allocation_model.addConstr(
                (aux_lateral_recourse_bin_vars[i, i_, s, u] == 0) >>
                (lateral_trans_vars.sum(i, i_, '*', s) >= pre_lateral_trans_vars_value.sum(i, i_, '*', s)))
            resource_allocation_model.addConstr(
                (aux_lateral_recourse_bin_vars[i, i_, s, u] == 0) >>
                (aux_lateral_recourse_reduce_vars[i, i_, s, u] == 0))
            resource_allocation_model.addConstr(
                (aux_lateral_recourse_bin_vars[i, i_, s, u] == 1) >>
                (lateral_trans_vars.sum(i, i_, '*', s) <= pre_lateral_trans_vars_value.sum(i, i_, '*', s) - 1))
            resource_allocation_model.addConstr(
                (aux_lateral_recourse_bin_vars[i, i_, s, u] == 1) >>
                (aux_lateral_recourse_reduce_vars[i, i_, s, u] == max_reduction))

        recourse_vars = self._fetch_RASP_recourse_vars(resource_allocation_model)
        resource_allocation_model.addConstr(recourse_vars[i, s] >= recourse_RHS, name='optimality_cut_{}'.format(u))
        resource_allocation_model.update()
        return resource_allocation_model

    def _add_feasibility_cut_to_master(self, master_model, pmp_callback):
        # sets
        assign_vars_value, _, _, vehicle_vars_value = self._fetch_preposition_solution(master_model, pmp_callback)
        assign, isolated_assign, facility_customers, facility_isolated_customer = self._get_assign(assign_vars_value)

        # data: BP(r m_j^s), x(u)_{ij}
        all_vehicle_RHS = {i: 0 for i in self.facility_set}
        special_vehicle_RHS = {s: {i: 0 for i in self.facility_set} for s in self.scenarios}

        # variables: w_i^h, x_{ij}
        assign_vars, location_vars, stock_vars, vehicle_vars = self._fetch_preposition_vars(master_model)

        # compute and save minimal vehicle number by BinPacking
        for s, i in product(self.scenarios, self.facility_set):
            if facility_customers[s][i]:
                min_vehicle_num = BinPacking(facility_customers[s][i], self.vehicle_capacity).mip_solve()
                if min_vehicle_num > all_vehicle_RHS[i]:
                    all_vehicle_RHS[i] = min_vehicle_num
            else:
                all_vehicle_RHS[i] = None
            if facility_isolated_customer[s][i]:
                min_s_vehicle_num = BinPacking(facility_isolated_customer[s][i], self.vehicle_capacity).mip_solve()
                special_vehicle_RHS[s][i] = min_s_vehicle_num
            else:
                special_vehicle_RHS[s][i] = None

        # add feasibility cut for all types of vehicle
        for i, vehicle_lb in all_vehicle_RHS.items():
            if vehicle_lb:
                vehicle_num = quicksum(vehicle_vars_value[(i, h)] for h in self.vehicle_types).getValue()
                if vehicle_num < vehicle_lb:
                    lhs = LinExpr(quicksum(vehicle_vars[i, h] for h in self.vehicle_types))
                    rhs = LinExpr(vehicle_lb - quicksum((1 - assign_vars[i, j] for j in assign[i])))
                    master_model = self._add_cut(master_model, lhs - rhs, pmp_callback)

        # add feasibility cut for special vehicle
        for s, special_vehicle_lb_dict in special_vehicle_RHS.items():
            for i, s_vehicle_lb in special_vehicle_lb_dict.items():
                if s_vehicle_lb:
                    special_vehicle_num = quicksum(
                        vehicle_vars_value[(i, h)] for h in self.special_vehicle_types).getValue()
                    if special_vehicle_num < s_vehicle_lb:
                        lhs = LinExpr(quicksum(vehicle_vars[i, h] for h in self.special_vehicle_types))
                        rhs = LinExpr(s_vehicle_lb - quicksum(1 - assign_vars[i, j] for j in isolated_assign[s][i]))
                        master_model = self._add_cut(master_model, lhs - rhs, pmp_callback)

        master_model.update()
        return master_model

    def _add_optimality_cut_to_master(
            self, master_model, resource_allocation_model, pmp_callback, rasp_callback):
        # sets
        assign_vars_value, _, _, vehicle_vars_value = self._fetch_preposition_solution(master_model, pmp_callback)
        assign, isolated_assign, facility_customers, facility_isolated_customer = self._get_assign(assign_vars_value)

        # data: E(R(u)_{is}), x(u)_{ij}
        resource_allocation_recourse_solutions = \
            self._fetch_RASP_recourse_solutions(resource_allocation_model, rasp_callback)
        master_recourse_solutions = self._fetch_master_recourse_solutions(master_model, pmp_callback)

        # variables: E(R_{is}), x_{ij}
        assign_vars, location_vars, stock_vars, vehicle_vars = self._fetch_preposition_vars(master_model)

        max_vehicle_unit_cost = max(self.vehicle_info[h]['uc'] for h in self.vehicle_info)
        master_optimality_cut_count = 0

        for i, s in product(assign, self.scenarios):
            cluster = set([i] + [a for a in assign[i]])
            if master_recourse_solutions[i, s] - resource_allocation_recourse_solutions[i, s] < TOL:
                rhs_term_a = LinExpr()
                for j in assign[i]:
                    furthest_distance = self.get_furthest_to(j, cluster)  # optimality cut in CIE(2021)
                    # furthest_distance = self.get_furthest_distance(cluster)  # alternative
                    rhs_term_a += (1 - assign_vars[i, j]) * 2 * max_vehicle_unit_cost * furthest_distance
                master_recourse_var = self._fetch_master_recourse_vars(master_model)[i, s]
                sub_recourse_value = resource_allocation_recourse_solutions[i, s]

                lhs = master_recourse_var
                rhs = sub_recourse_value - rhs_term_a
                master_model = self._add_cut(master_model, lhs - rhs, pmp_callback)
                master_optimality_cut_count += 1

        master_model.update()
        return master_model, master_optimality_cut_count

    # """ # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # """
    # """ # # # # # # # # # # # # # solve problem # # # # # # # # # # # # # # # """
    # """ # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # """

    def _reach_optimum(self, lower_bound, upper_bound, p=PARAMETERS['lbbd_precision']):
        # lower_bound = round(lower_bound, p)
        # upper_bound = round(upper_bound, p)
        # assert lower_bound <= upper_bound, Exception("LB ({}) > UB ({})".format(lower_bound, upper_bound))

        if lower_bound == 0 and upper_bound == 0:
            relative_gap = 0
        elif lower_bound != 0 and upper_bound == 0:
            relative_gap = float("inf")
        else:
            relative_gap = abs(upper_bound - lower_bound) / abs(upper_bound)

        is_optimized = relative_gap <= LBBD_TOLERANCE
        return is_optimized, relative_gap

    def parallel_vehicle_dispatching_solving(
            self, RASP, multi_processing=PARAMETERS.get('multi_precessing_vdsp', False)):
        pre_direct_trans_vars_value, pre_lateral_trans_vars_value, pre_vehicle_vars_value = \
            self._fetch_RASP_solution(RASP, rasp_callback=False)
        pre_direct_trans_vars_value = dict(pre_direct_trans_vars_value)
        pre_lateral_trans_vars_value = dict(pre_lateral_trans_vars_value)
        pre_vehicle_vars_value = dict(pre_vehicle_vars_value)

        if multi_processing:
            cores = self.parameters.get('thread', multiprocessing.cpu_count())
            pool = multiprocessing.Pool(processes=cores)
            tasks = [
                (i, s,
                 pre_direct_trans_vars_value, pre_lateral_trans_vars_value, pre_vehicle_vars_value, multi_processing)
                for i, s in product(self.facility_set, self.scenarios)
            ]
            mp_res_list = pool.starmap(self.make_and_solve_vehicle_dispatching, tasks)
            res = {(content[0], content[1]): content[2:] for content in mp_res_list}

        else:
            res = dict()
            for i, s in product(self.facility_set, self.scenarios):
                res[(i, s)] = self.make_and_solve_vehicle_dispatching(
                    i, s, pre_direct_trans_vars_value, pre_lateral_trans_vars_value, pre_vehicle_vars_value,
                    multi_processing)[2:]
        return res

    def make_and_solve_vehicle_dispatching(
            self, i, s,
            pre_direct_trans_vars_value, pre_lateral_trans_vars_value, pre_vehicle_vars_value,
            multi_processing):

        def make_and_solve(model):
            depot_customers, depot_vehicle, VDSP, non_empty = self.vehicle_dispatching_model(
                model, pre_direct_trans_vars_value, pre_lateral_trans_vars_value, pre_vehicle_vars_value, i, s)

            # print(i, s, len(depot_customers.keys()) - 1, depot_customers)
            if len(depot_customers.keys()) - 1 > self.parameters.get('demand_lower_lim', float('inf')):
                if self.parameters.get("vdsp_gap_lim", None):
                    VDSP.setParam("MIPGap", self.parameters["vdsp_gap_lim"])
                if self.parameters.get("vdsp_time_lim", None):
                    VDSP.setParam("TimeLimit", self.parameters['vdsp_time_lim'])

            VDSP.optimize()
            VDSP_obj = VDSP.getAttr("ObjVal")
            VDSP_gap = VDSP.getAttr("MIPGap")
            VDSP_runtime = VDSP.getAttr("Runtime")
            VDSP_status = VDSP.getAttr("Status")
            VDSP_vars_value = [(var.getAttr("VarName"), var.getAttr("X")) for var in VDSP.getVars()]
            return i, s, depot_customers, depot_vehicle, non_empty, VDSP_obj, VDSP_gap, VDSP_runtime, VDSP_status, VDSP_vars_value

        if multi_processing:
            with Env(empty=True) as env:
                env.setParam('OutputFlag', 0)
                env.start()
                with Model(env=env) as model:
                    return make_and_solve(model)
        else:
            return make_and_solve(self.make_empty_model())

    def _nested_problem_solve(self, master_model, pmp_callback, rasp_callback, vdsp_callback,
                              log_to_console=PARAMETERS['lbbd_log_to_console']):
        RASP = self.resource_allocation_model(master_model, pmp_callback)
        reach_optimum = False
        sub_stop = False
        VDSPs_vars_value = None
        iteration_time = 0
        best_ub = float("inf")

        # statistic
        RASP_runtime_li = []
        RASP_feasibility_cut_count_li = []
        RASP_optimality_cut_count_li = []
        VDSP_call_count = 0
        VDSP_runtime_li_li = []

        while not sub_stop:
            iteration_time += 1

            solving_start_t = time.perf_counter()
            RASP.optimize()
            solving_end_t = time.perf_counter()

            RASP_runtime_li.append(solving_end_t - solving_start_t)
            RASP_feasibility_cut_count_li.append(0)
            RASP_optimality_cut_count_li.append(0)

            RASP_status = RASP.getAttr("Status")

            if RASP_status == GRB.INFEASIBLE:
                obj = None
                sub_stop = True


            elif RASP_status == GRB.OPTIMAL:
                pre_direct_trans_vars_value, pre_lateral_trans_vars_value, pre_vehicle_vars_value = \
                    self._fetch_RASP_solution(RASP, rasp_callback=False)

                VDSP_is_all_feasible = True
                VDSP_run_time_li = []

                VDSP_results = tupledict(self.parallel_vehicle_dispatching_solving(RASP))

                for i, s in product(self.facility_set, self.scenarios):
                    depot_customers, depot_vehicle, non_empty, VDSP_obj, VDSP_gap, VDSP_runtime, VDSP_status, VDSP_vars_value = \
                        VDSP_results[(i, s)]

                    if non_empty:
                        VDSP_call_count += 1
                        VDSP_run_time_li.append(VDSP_runtime)
                        if VDSP_status == GRB.OPTIMAL:
                            pass
                            # warn("A VDSP gap = {}, the objective is potentially be suboptimal.".format(VDSP_gap))
                        elif VDSP_status == GRB.INFEASIBLE:
                            VDSP_is_all_feasible = False
                            RASP_feasibility_cut_count_li[-1] += 1
                            RASP = self._add_feasibility_cut_to_resource_allocation(
                                i, s, iteration_time,
                                RASP, pre_direct_trans_vars_value, pre_lateral_trans_vars_value,
                                depot_customers, rasp_callback)
                        elif VDSP_status == GRB.TIME_LIMIT:
                            warn("A VDSP is terminated by TIME_LIMIT, the objective is potentially be suboptimal.")
                        else:
                            raise Exception("Unknown VDSP status: {}".format(VDSP_status))

                VDSP_runtime_li_li.append(VDSP_run_time_li)

                RASP_reach_optimal = False
                if VDSP_is_all_feasible:
                    RASP_scenario_objs = {
                        s: self._fetch_RASP_recourse_solutions(RASP, rasp_callback).sum('*', s).getValue()
                        for s in self.scenarios}
                    VDSP_scenario_objs = {
                        s: sum(a[3] for a in VDSP_results.select('*', s)) for s in self.scenarios}

                    RASP_obj = sum(RASP_scenario_objs[s] * self.scenario_prob[s] for s in self.scenarios)
                    VDSP_obj = sum(VDSP_scenario_objs[s] * self.scenario_prob[s] for s in self.scenarios)
                    RASP_recourse_costs = self._fetch_RASP_recourse_solutions(RASP, rasp_callback)
                    VDSP_recourse_costs = tupledict(
                        [(i, s), VDSP_results[(i, s)][3]] for i, s in product(self.facility_set, self.scenarios))

                    best_ub = VDSP_obj if VDSP_obj < best_ub else best_ub

                    if log_to_console:
                        RASP_obj_ = round(RASP_obj, self.parameters['lbbd_precision'])
                        best_ub_ = round(best_ub, self.parameters['lbbd_precision'])
                        VDSP_obj_ = round(VDSP_obj, self.parameters['lbbd_precision'])
                        print("      {:<3}: {:>15f} {} {} {:>15f} {}, {:>15f} {}".format(
                            iteration_time,
                            RASP_obj_, "(LB, Current RASP)",
                            '<' if RASP_obj_ < best_ub_ else (">" if RASP_obj_ > best_ub_ else "="),
                            best_ub_, "(UB)",
                            VDSP_obj_, "(Current VDSP)"
                        ))

                    RASP_reach_optimal = self._reach_optimum(RASP_obj, best_ub)[0]
                    if not RASP_reach_optimal:
                        for i, s in product(self.facility_set, self.scenarios):
                            depot_customers = VDSP_results[(i, s)][0]
                            recourse_RHS = VDSP_recourse_costs[(i, s)]
                            if RASP_recourse_costs[(i, s)] < VDSP_recourse_costs[(i, s)]:
                                RASP_optimality_cut_count_li[-1] += 1
                                # print("sub optimality cut")
                                RASP = self._add_optimality_cut_to_resource_allocation(
                                    i, s, iteration_time,
                                    RASP, pre_direct_trans_vars_value, pre_lateral_trans_vars_value,
                                    depot_customers, recourse_RHS, vdsp_callback
                                )

                if VDSP_is_all_feasible and RASP_reach_optimal:
                    VDSPs_vars_value = tupledict({(i, s): content[7] for (i, s), content in VDSP_results.items()})
                    sub_stop = True
                    obj = RASP.getObjective().getValue()

            else:
                raise Exception("Unknown RASP status: {}".format(RASP_status))

        if log_to_console:
            print('        [>>', self.instance_name, "RASP reach local optimum with: obj={}]".format(obj))

        stat = {
            "RASP_runtime_li": RASP_runtime_li,
            "RASP_feasibility_cut_count_li": RASP_feasibility_cut_count_li,
            "RASP_optimality_cut_count_li": RASP_optimality_cut_count_li,
            "VDSP_call_count": VDSP_call_count,
            "VDSP_runtime_li_li": VDSP_runtime_li_li
        }

        return RASP, VDSPs_vars_value, obj, stat

    def benders_solve(self, log_to_console=PARAMETERS['lbbd_log_to_console'], save_result=False):

        if log_to_console:
            print("Solving: {} with LBBD".format(self.instance_name))

        use_callback = False
        heuristic_solution = {"PMP_" + key: value for key, value in self.feasible_solution_heuristic().items()}
        PMP = self.preposition_master_model(heuristic_solution, fix_first_stage=False)

        best_known_sol = []
        reach_optimum = False
        stop = False
        best_master_ub = float("inf")
        iteration_time = 0

        # statistic - begin
        upper_bound_dic = dict()
        best_upper_bound_dic = dict()
        lower_bound_dic = dict()
        gap_dic = dict()
        PMP_runtime_dic = dict()
        iteration_finish_time_dic = dict()
        PMP_feasibility_cut_count_dic = dict()
        PMP_optimality_cut_count_dic = dict()
        RASP_stats_dic = dict()

        PMP_call_count = 0
        RASP_call_count = 0
        VDSP_call_count = 0
        overall_start_t = time.perf_counter()

        pmp_obj_dic = dict()
        pmp_recourse_dic = dict()
        rasp_recourse_dic = dict()
        # statistic - end

        while not stop:
            iteration_time += 1
            solving_start_t = time.perf_counter()
            PMP.optimize()
            solving_end_t = time.perf_counter()
            PMP_runtime_dic[iteration_time] = solving_end_t - solving_start_t
            PMP_call_count += 1

            PMP_status = PMP.getAttr("Status")
            if PMP_status == GRB.INFEASIBLE:
                raise Exception("PMP status: INFEASIBLE")

            elif PMP_status == GRB.OPTIMAL:
                solved_RASP, VDSPs_vars_value, obj, RASP_stat = self._nested_problem_solve(
                    PMP,
                    pmp_callback=use_callback, rasp_callback=use_callback, vdsp_callback=use_callback,
                    log_to_console=log_to_console
                )
                # following make json content
                RASP_stats_dic[iteration_time] = RASP_stat
                RASP_call_count += 1
                VDSP_call_count += RASP_stat['VDSP_call_count']

                RASP_status = solved_RASP.getAttr("Status")
                if RASP_status == GRB.INFEASIBLE:
                    if time.perf_counter() - overall_start_t >= self.parameters['lbbd_time_lim']:
                        stop = True
                    else:
                        print("master feasibility cut")
                        PMP = self._add_feasibility_cut_to_master(PMP, use_callback)
                        PMP_feasibility_cut_count_dic[iteration_time] = 1

                elif RASP_status == GRB.OPTIMAL:
                    # following make json content
                    PMP_obj = PMP.getObjective().getValue()
                    PMP_recourse_obj = self._fetch_master_recourse_solution(PMP, use_callback)
                    RASP_obj = solved_RASP.getObjective().getValue()
                    master_ub = PMP_obj - PMP_recourse_obj + RASP_obj
                    best_master_ub = master_ub if master_ub < best_master_ub else best_master_ub
                    upper_bound_dic[iteration_time] = master_ub
                    best_upper_bound_dic[iteration_time] = best_master_ub
                    lower_bound_dic[iteration_time] = PMP_obj
                    master_is_optimized, gap_dic[iteration_time] = self._reach_optimum(PMP_obj, best_master_ub)

                    pmp_obj_dic[iteration_time] = PMP_obj
                    pmp_recourse_dic[iteration_time] = PMP_recourse_obj
                    rasp_recourse_dic[iteration_time] = RASP_obj

                    # following make .sol content
                    if master_ub == best_master_ub or not best_known_sol:
                        best_known_sol = []
                        VDSPs_sol = []
                        for (i, s), vars_value in VDSPs_vars_value.items():
                            for var_name, var_value in vars_value:
                                VDSPs_sol += [(i, s, var_name, var_value)]
                        RASP_sol = [(var.getAttr("VarName"), var.getAttr("X")) for var in solved_RASP.getVars()]
                        PMP_sol = [(var.getAttr("VarName"), var.getAttr("X")) for var in PMP.getVars()]
                        best_known_sol += [master_ub] + PMP_sol + RASP_sol + VDSPs_sol

                    # following make console content
                    if log_to_console:
                        PMP_obj_ = round(PMP_obj, self.parameters['lbbd_precision'])
                        master_ub_ = round(master_ub, self.parameters['lbbd_precision'])
                        best_master_ub_ = round(best_master_ub, self.parameters['lbbd_precision'])
                        print(
                            "\033[7m"
                            "{:<3}: {:>15f} = {} + {} {} {} {:>15f} {}, {:>15f} = {} + {} {}"
                            "\033[0m".format(
                                iteration_time,
                                PMP_obj_, PMP_obj - PMP_recourse_obj, PMP_recourse_obj,
                                "(LB, Current PMP)",
                                '<' if PMP_obj_ < best_master_ub_ else (">" if PMP_obj_ > best_master_ub_ else "="),
                                best_master_ub_, "(UB)",
                                master_ub_, PMP_obj - PMP_recourse_obj, RASP_obj,
                                "(Current RASP)"
                            ))

                    if master_is_optimized:
                        reach_optimum, stop = True, True
                    else:
                        if time.perf_counter() - overall_start_t >= self.parameters['lbbd_time_lim']:
                            reach_optimum, stop = False, True
                        else:
                            PMP, opt_cut_count = self._add_optimality_cut_to_master(
                                PMP, solved_RASP,
                                pmp_callback=use_callback, rasp_callback=use_callback)
                            PMP_optimality_cut_count_dic[iteration_time] = opt_cut_count

                else:
                    raise Exception("Unknown RASP status: {}".format(RASP_status))
            else:
                raise Exception("Unknown PMP status: {}".format(PMP_status))

            iteration_finish_time_dic[iteration_time] = time.perf_counter() - overall_start_t

        overall_runtime = time.perf_counter() - overall_start_t

        if log_to_console:
            if reach_optimum:
                obj = PMP.getObjective().getValue()
                print('[>>', self.instance_name, "PMP reach optimum with: obj={}]\n\n".format(obj))

        stat = {
            "upper_bound_dic": upper_bound_dic,
            "best_upper_bound_dic": best_upper_bound_dic,
            "lower_bound_dic": lower_bound_dic,
            "gap_dic": gap_dic,
            "PMP_runtime_dic": PMP_runtime_dic,
            "iteration_finish_time_dic": iteration_finish_time_dic,
            "PMP_feasibility_cut_count_dic": PMP_feasibility_cut_count_dic,
            "PMP_optimality_cut_count_dic": PMP_optimality_cut_count_dic,
            "RASP_stats_dic": RASP_stats_dic,
            "iteration_time": iteration_time,
            "PMP_call_count": PMP_call_count,
            "RASP_call_count": RASP_call_count,
            "VDSP_call_count": VDSP_call_count,
            "overall_runtime": overall_runtime,
            "reach_optimum": reach_optimum,

            "pmp_obj_dic": pmp_obj_dic,
            "pmp_recourse_dic": pmp_recourse_dic,
            "rasp_recourse_dic": rasp_recourse_dic,

            "SolutionInfo": {
                "Status": GRB.OPTIMAL if reach_optimum else 0,
                "Runtime": overall_runtime,
                "ObjVal": best_known_sol[0],
                "MIPGap": min(gap_dic.values()),
                "SolCount": 1,
            }
        }

        if save_result and self.instance_type == "LBBD":
            write_lbbd_sol(self.instance_name, best_known_sol, "LBBD")
            write_lbbd_json(self.instance_name, stat, "LBBD")

        return stat, best_known_sol

    def branch_and_check_solve(self, log_to_console=PARAMETERS['lbbd_log_to_console'], save_result=False):

        if log_to_console:
            print("Solving: {} with B&C".format(self.instance_name))

        self.best_known_sol = []
        self.best_master_ub = float("inf")
        self.iteration_time = 0

        # statistic
        self.upper_bound_dic = dict()
        self.best_upper_bound_dic = dict()
        self.lower_bound_dic = dict()
        self.gap_dic = dict()
        PMP_runtime_dic = dict()
        overall_start_t = time.perf_counter()
        iteration_finish_time_dic = dict()
        self.PMP_feasibility_cut_count_dic = dict()
        self.PMP_optimality_cut_count_dic = dict()
        self.RASP_stats_dic = dict()

        PMP_call_count = 1
        self.RASP_call_count = 0
        self.VDSP_call_count = 0

        self.pmp_obj_dic = dict()
        self.pmp_recourse_dic = dict()
        self.rasp_recourse_dic = dict()

        def _resource_allocation_callback(master_model, where):
            pmp_callback, rasp_callback, vdsp_callback = True, False, False

            if where == GRB.Callback.MIPSOL:
                self.iteration_time += 1
                solved_RASP, VDSPs_vars_value, obj, RASP_stat = self._nested_problem_solve(
                    master_model,
                    pmp_callback=pmp_callback, rasp_callback=rasp_callback, vdsp_callback=vdsp_callback,
                    log_to_console=False)
                # following make json content
                self.RASP_stats_dic[self.iteration_time] = RASP_stat
                self.RASP_call_count += 1
                self.VDSP_call_count += RASP_stat['VDSP_call_count']

                if solved_RASP.getAttr("Status") == GRB.INFEASIBLE:
                    # solved_RASP.computeIIS()
                    # solved_RASP.write("RASP_IIS_{}_{}.ilp".format(self.instance_name, self.iteration_time))

                    self._add_feasibility_cut_to_master(master_model, pmp_callback=pmp_callback)
                    self.PMP_feasibility_cut_count_dic[self.iteration_time] = 1

                elif solved_RASP.getAttr("Status") == GRB.OPTIMAL:
                    lp_lower_bound = master_model.cbGet(GRB.Callback.MIPSOL_OBJBND)
                    # following make json content
                    PMP_obj = master_model.cbGet(GRB.Callback.MIPSOL_OBJ)
                    PMP_recourse_obj = self._fetch_master_recourse_solution(master_model, pmp_callback)
                    RASP_obj = solved_RASP.getObjective().getValue()
                    master_ub = PMP_obj - PMP_recourse_obj + RASP_obj
                    self.best_master_ub = master_ub if master_ub < self.best_master_ub else self.best_master_ub
                    self.upper_bound_dic[self.iteration_time] = master_ub
                    self.best_upper_bound_dic[self.iteration_time] = self.best_master_ub
                    self.lower_bound_dic[self.iteration_time] = lp_lower_bound
                    _, gap = self._reach_optimum(lp_lower_bound, self.best_master_ub)
                    self.gap_dic[self.iteration_time] = gap

                    self.pmp_obj_dic[self.iteration_time] = PMP_obj
                    self.pmp_recourse_dic[self.iteration_time] = PMP_recourse_obj
                    self.rasp_recourse_dic[self.iteration_time] = RASP_obj

                    # following make .sol file content
                    if master_ub == self.best_master_ub or not self.best_known_sol:
                        self.best_known_sol = []
                        VDSPs_sol = []
                        for (i, s), vars_value in VDSPs_vars_value.items():
                            for var_name, var_value in vars_value:
                                VDSPs_sol += [(i, s, var_name, var_value)]
                        RASP_sol = [(var.getAttr("VarName"), var.getAttr("X")) for var in solved_RASP.getVars()]
                        PMP_sol = [(var.getAttr("VarName"), master_model.cbGetSolution(var))
                                   for var in master_model.getVars()]
                        self.best_known_sol += [master_ub] + PMP_sol + RASP_sol + VDSPs_sol

                    master_model, opt_cut_count = self._add_optimality_cut_to_master(
                        master_model, solved_RASP,
                        pmp_callback=pmp_callback, rasp_callback=rasp_callback)
                    self.PMP_optimality_cut_count_dic[self.iteration_time] = opt_cut_count

                    if gap <= self.parameters.get('lbbd_gap', 0):
                        master_model.terminate()

                else:
                    raise Exception("Unknown RASP status: {}".format(solved_RASP.getAttr("Status")))

                iteration_finish_time_dic[self.iteration_time] = time.perf_counter() - overall_start_t

            if where in [GRB.Callback.MIP, GRB.Callback.MIPSOL]:
                if master_model.cbGet(GRB.Callback.RUNTIME) > self.parameters.get('lbbd_time_lim', float('inf')):
                    master_model.terminate()

        PMP = self.preposition_master_model()
        PMP.setParam("LazyConstraints", 1)
        PMP.setParam("LogToConsole", log_to_console)
        # PMP.setParam("Seed", 2)  # default 0
        PMP.setParam("MIPGap", self.parameters['lbbd_mip_gap'])

        PMP.optimize(callback=_resource_allocation_callback)

        final_feasible_obj = min(self.best_upper_bound_dic.values())
        final_pmp_obj = PMP.getObjective().getValue()
        final_lp_lower_bound = PMP.getAttr("ObjBound")
        final_lbbd_mip_gap = PMP.getAttr("MIPGap")
        status = PMP.getAttr("Status")

        _, final_lbbd_gap = self._reach_optimum(final_lp_lower_bound, final_feasible_obj)
        optimized = True if status in [GRB.OPTIMAL, GRB.INTERRUPTED] else False

        stat = {
            "upper_bound_dic": self.upper_bound_dic,
            "best_upper_bound_dic": self.best_upper_bound_dic,
            "lower_bound_dic": self.lower_bound_dic,
            "gap_dic": self.gap_dic,
            "PMP_runtime_dic": PMP_runtime_dic,
            "iteration_finish_time_dic": iteration_finish_time_dic,
            "PMP_feasibility_cut_count_dic": self.PMP_feasibility_cut_count_dic,
            "PMP_optimality_cut_count_dic": self.PMP_optimality_cut_count_dic,
            "RASP_stats_dic": self.RASP_stats_dic,
            "iteration_time": self.iteration_time,
            "PMP_call_count": PMP_call_count,
            "RASP_call_count": self.RASP_call_count,
            "VDSP_call_count": self.VDSP_call_count,
            "overall_runtime": PMP.getAttr("Runtime"),
            "reach_optimum": optimized,

            "pmp_obj_dic": self.pmp_obj_dic,
            "pmp_recourse_dic": self.pmp_recourse_dic,
            "rasp_recourse_dic": self.rasp_recourse_dic,

            "SolutionInfo": {
                "Status": status,
                "Runtime": PMP.getAttr("Runtime"),
                "ObjVal": final_feasible_obj,
                "MIPGap": final_lbbd_gap,  # consist with DE for io_operator.py
                "SolCount": 1,
            },

            "Final": {
                "final_feasible_obj": final_feasible_obj,
                "final_pmp_obj": final_pmp_obj,
                "final_lp_lower_bound": final_lp_lower_bound,
                "final_lbbd_mip_gap": final_lbbd_mip_gap,
                "final_lbbd_gap": final_lbbd_gap,
                "final_is_optimized": optimized,
            }
        }

        if save_result and self.instance_type == "BAC":
            write_lbbd_sol(self.instance_name, self.best_known_sol, "BAC")
            write_lbbd_json(self.instance_name, stat, "BAC")

        # assert final_feasible_obj == self.best_known_sol[0]
        return stat, self.best_known_sol

    def experiment(self):

        if self.instance_type == "PA_DE":
            stat, best_known_sol = self.branch_and_check_solve()
            write_lbbd_sol(self.instance_name, best_known_sol, self.instance_type, **self.exp_set)
            write_lbbd_json(self.instance_name, stat, self.instance_type, **self.exp_set)

        if self.instance_type == "NO_VRP" and self.problem_type == "AUX":
            stat, best_known_sol = self.branch_and_check_solve()
            write_lbbd_sol(self.instance_name, best_known_sol, self.instance_type, **self.exp_set)
            write_lbbd_json(self.instance_name, stat, self.instance_type, **self.exp_set)

        if self.instance_type == "STC" and self.p_type == "AUX":
            stat, best_known_sol = self.branch_and_check_solve()
            write_lbbd_sol(self.instance_name, best_known_sol, self.instance_type, **self.exp_set)
            write_lbbd_json(self.instance_name, stat, self.instance_type, **self.exp_set)


if __name__ == "__main__":
    t = time.perf_counter()

    instances_to_solve = EXPERIMENTAL_INSTANCES_NAME
    for instance in instances_to_solve:
        LBBD = LogicBasedBendersDecomposition(instance, PARAMETERS, 'LBBD')
        # LBBD.benders_solve(save_result=False)
        LBBD.branch_and_check_solve(save_result=False)

    print('\nTotal running time:', time.perf_counter() - t, 's')
