import json
import math
import time
import warnings
import itertools
import random
from copy import deepcopy
from dataclasses import asdict

from LBBD import LogicBasedBendersDecomposition
from solver import StochasticModels
from config import PARAMETERS, META_DATA_TEMPLATE

# VRPSolverEasy == 0.1.1, https://vrpsolvereasy.readthedocs.io/en/latest/Solver%20API/index.html
import VRPSolverEasy as vrpse
import VRPSolverEasy.src.constants
from gurobipy import GRB, LinExpr

random.seed(PARAMETERS['instance_seed'])

# Accelerating VDSPs
HARD_VRP_DEMAND_NUM = 20
VRP_UB_TIME = 15
HARD_VRP_TIME = 300

# Accelerating by warm-start cuts
WARM_START_TIME_LIM = 300
MINUS_WARM_START_TIME = True


# Accelerate - Stabilization, see dalmeijerBenders102Acceleration2021
# - Trust Region, see zhouStochasticProgrammingModel2022, santosoStochasticProgrammingApproach2005
# - In-out, see fischettiRedesigningBendersDecomposition2017, fischettiBendersDecompositionSeparability2016
# Accelerate - Upper bounding constraints, see maherEnhancingLargeNeighbourhood2021, expected-value warm-start
# Accelerate - Good (optimal) initial solution, see linderothAdvancedFeaturesStochastic2016, santosoStochasticProgrammingApproach2005


class r_LogicBasedBendersDecomposition(LogicBasedBendersDecomposition):
    def __init__(self, instance_name, parameters, instance_type,
                 vrp_solver=True, warm_start=True, minus_warm_start_time=MINUS_WARM_START_TIME, **exp_set):
        super().__init__(instance_name, parameters, instance_type, **exp_set)
        self.original_data = deepcopy(self.data)
        self.points_demand = self.data['points_demand']
        self.points_isolating = self.data['points_isolating']
        self.resampled = False

        self.vrp_solver = vrp_solver
        self.warm_start = warm_start
        self.minus_warm_start_time = minus_warm_start_time
        self.warm_start_time = 0

    # NOTE: Comment following code, only for instances with |D| = 40
    def _make_vehicle_dispatching_sub_problem(self, model, depots_customers, vehicle_preparation, i, s):
        node_set = [key for key in depots_customers[s][i]]
        customer_set = [key for key in depots_customers[s][i] if key != i]
        non_empty = True if customer_set else False

        vrp = model

        for customer_id in customer_set:
            iv = []
            if customer_id in self.isolated_demand_in_scenario[s]:
                iv = self.general_vehicle_types
            vrp.add_customer(id=customer_id, demand=depots_customers[s][i][customer_id], incompatible_vehicles=iv)

        vrp.add_depot(id=i)

        for a, b in itertools.product(node_set, node_set):
            vrp.add_link(start_point_id=a, end_point_id=b, distance=self._get_distance(a, b))

        for vehicle_type, vehicle_num in vehicle_preparation[i][s].items():
            # Vehicle id cannot be less than 1
            vrp.add_vehicle_type(
                id=vehicle_type,
                max_number=vehicle_num,
                start_point_id=i, end_point_id=i,
                capacity=self.vehicle_info[vehicle_type]['cap'], var_cost_dist=self.vehicle_info[vehicle_type]['uc'])

        max_special_vehicle_uc = max(self.vehicle_info[h]['uc'] for h in self.special_vehicle_types)
        max_general_vehicle_uc = max(self.vehicle_info[h]['uc'] for h in self.general_vehicle_types)
        distance_li = [self._get_distance(i, j) for j in customer_set]
        uc_li = [max_special_vehicle_uc if self.is_isolated(s, j) else max_general_vehicle_uc for j in customer_set]
        upper_bound = 2 * sum([a * b for a, b in zip(distance_li, uc_li)])

        # Advanced, but inefficient
        # def terminate_if_feasible(model, where):
        #     if where == GRB.Callback.MIPSOL:
        #         model.terminate()
        #
        # # Is feasible solution by Gurobi efficient? -> No for instances with |D| = 20
        # if non_empty and len(customer_set) > HARD_VRP_DEMAND_NUM:
        #     grb_model = super()._make_vehicle_dispatching_sub_problem(
        #         super().make_empty_model(), depots_customers, vehicle_preparation, i, s)[0]
        #     grb_model.setParam('TimeLimit', VRP_UB_TIME)
        #     grb_model.setParam("MIPGap", 1e-4)
        #     grb_model.setParam('MIPFocus', 1)
        #     grb_model.optimize(callback=terminate_if_feasible)
        #     if grb_model.getAttr("SolCount") > 0:
        #         upper_bound = grb_model.getObjective().getValue()
        #
        # vrp.set_parameters(print_level=-2, upper_bound=upper_bound, time_limit=HARD_VRP_TIME)

        # Simpler, but efficient (Version in 20230511, see TimeMachine backup)
        vrp.set_parameters(print_level=-2, upper_bound=upper_bound * 2)

        return vrp, non_empty

    def make_and_solve_vehicle_dispatching(
            self, i, s,
            pre_direct_trans_vars_value, pre_lateral_trans_vars_value, pre_vehicle_vars_value,
            multi_processing):

        model = vrpse.Model()
        depot_customers, depot_vehicle, VDSP, non_empty = self.vehicle_dispatching_model(
            model, pre_direct_trans_vars_value, pre_lateral_trans_vars_value, pre_vehicle_vars_value, i, s)

        if non_empty:
            start_time = time.perf_counter()
            VDSP.solve()
            VDSP_vars_value = []

            if VDSP.status in [
                vrpse.src.constants.OPTIMAL_SOL_FOUND, vrpse.src.constants.BETTER_SOL_FOUND]:
                # Should usually reach here
                ub = VDSP.solution.json["Solution"]["solutionValue"]
                lb = VDSP.solution.json["Statistics"]["bestLB"]
                # Use feasible solutions of VDSPs
                VDSP_obj = ub
                VDSP_gap = (ub - lb) / ub if ub > 0 else 0
                VDSP_runtime = VDSP.solution.json["Statistics"]["solutionTime"]
                VDSP_status = GRB.OPTIMAL
            elif VDSP.status in [
                vrpse.src.constants.BETTER_SOL_DOES_NOT_EXISTS, vrpse.src.constants.BETTER_SOL_NOT_FOUND]:
                ub = VDSP.parameters.upper_bound
                lb = VDSP.parameters.upper_bound
                VDSP_obj = ub
                VDSP_gap = (ub - lb) / ub if ub > 0 else 0
                VDSP_runtime = time.perf_counter() - start_time
                VDSP_status = GRB.OPTIMAL
            else:
                # Should not reach hear
                warnings.warn(
                    f"VDSP status={VDSP.status}: i={i}, s={s}, vehicle={depot_vehicle}, customer={depot_customers}.")
                VDSP_obj = -1
                VDSP_gap = -1
                VDSP_runtime = time.perf_counter() - start_time
                VDSP_status = GRB.INFEASIBLE

            return i, s, depot_customers, depot_vehicle, non_empty, \
                VDSP_obj, VDSP_gap, VDSP_runtime, VDSP_status, VDSP_vars_value
        else:
            return i, s, depot_customers, depot_vehicle, non_empty, 0, 0, 0, GRB.OPTIMAL, []

    def reset(self, force_original=False):
        # R.2.4 & R.2.6
        if force_original or not self.resampled:
            self.__dict__.update(self.original_data)
            self.resampled = False
        else:
            self.__dict__.update(self.resampled_data)

    def resample(self, scenario_size, correlated_demand):
        # R.2.4 & R.2.5
        def step_func(distance):
            assert distance >= 0
            range_a, range_a_value = self.step_function[0][0], self.step_function[0][1]
            range_b, range_b_value = self.step_function[1][0], self.step_function[1][1]
            other_value = self.step_function[2][1]
            if distance >= range_a[0] and distance < range_a[1]:
                value = range_a_value
            elif distance >= range_b[0] and distance < range_b[1]:
                value = range_b_value
            else:
                value = other_value
            return value

        def distance_to_center(point_loc):
            point_x, point_y = point_loc
            if self.is_line_disaster_center:
                center = self.line_disaster_center
                distance_to_center = abs(point_y - center)
            else:
                center_x, center_y = self.ring_disaster_center
                distance_to_center = math.sqrt((point_x - center_x) ** 2 + (point_y - center_y) ** 2)
            return distance_to_center

        def rand_isolating(severity_score_category, d_to_center):
            isolating_prob = step_func(d_to_center) * META_DATA_TEMPLATE["vulnerability_score"][severity_score_category]
            is_isolated = 0
            if random.random() <= isolating_prob:
                is_isolated = 1
            return is_isolated

        self.reset()
        self.scenario_num = scenario_size
        self.scenario_prob = {s: 1 / self.scenario_num for s in range(self.scenario_num)}

        for s in range(self.scenario_num):
            severity_score_category = s % 4
            self.points_demand[s] = dict()
            self.points_isolating[s] = dict()
            self.scenarios[s] = dict()
            self.isolated_demand_in_scenario[s] = []
            for j in self.demand_set:
                d_to_center = distance_to_center(self.points_loc[j])
                coefficient = 1 if not correlated_demand == False else step_func(d_to_center) * correlated_demand
                severity_score_range = META_DATA_TEMPLATE["triangular_severity_score_range"][severity_score_category]
                u_param_a, u_param_b = severity_score_range[0], severity_score_range[1]
                demand = math.ceil(coefficient * self.points_population[j] * random.uniform(u_param_a, u_param_b))
                is_isolating = rand_isolating(severity_score_category, d_to_center)
                self.points_demand[s][j] = demand
                self.points_isolating[s][j] = is_isolating
                self.scenarios[s][j] = [demand, is_isolating]
                if is_isolating == 1:
                    self.isolated_demand_in_scenario[s].append(j)

        self.big_m_location_trans = {s: self.big_m for s in self.scenarios}
        self.big_m_aux_assign = self.big_m_location_trans
        if PARAMETERS.get('use_local_big_m', False):
            self.big_m_location_trans = {
                s: sum(self.scenarios[s][j][0] for j in self.demand_set) for s in self.scenarios}

        self.resampled = True
        self.resampled_data = deepcopy({k: v for k, v in locals()["self"].__dict__.items() if k in self.data.keys()})

    def wait_and_see_instance(self, ss):
        # R.2.6
        self.reset()
        assert ss in self.scenarios.keys(), f"Scenario {ss} is not in the scenario set = [{self.scenarios.keys()}]."
        self.scenario_num = 1
        self.scenario_prob = {ss: 1}
        self.scenarios = {ss: self.scenarios[ss]}
        self.isolated_demand_in_scenario = {ss: self.isolated_demand_in_scenario[ss]}
        self.points_demand = {ss: self.points_demand[ss]}
        self.points_isolating = {ss: self.points_isolating[ss]}

    def _wait_and_see_warm_start(self, preposition_master_model):
        # R.2.6
        scenario_list = list(self.scenarios.keys())
        assign_vars, location_vars, stock_vars, vehicle_vars = self._fetch_preposition_vars(preposition_master_model)
        recourse_vars = self._fetch_master_recourse_vars(preposition_master_model)

        for s in scenario_list:
            SD = StochasticModels(self.instance_name, PARAMETERS, 'DE')
            self.wait_and_see_instance(s)
            SD.__dict__.update(self.__dict__)
            solved_model = SD.solve(write_to_file=False, time_limit=WARM_START_TIME_LIM)
            obj_lb = solved_model.getAttr("ObjBound")
            self.reset()
            location_cost = LinExpr([self.facility_open_cost[i][k] for i, k in location_vars],
                                    [location_vars[i, k] for i, k in location_vars])
            stock_cost = LinExpr([self.unit_facility_acquisition_cost for _ in stock_vars],
                                 [stock_vars[i] for i in stock_vars])
            vehicle_cost = LinExpr([self.vehicle_info[h]['fc'] for (_, h) in vehicle_vars],
                                   [vehicle_vars[i, h] for (i, h) in vehicle_vars])
            first_stage_cost = location_cost + stock_cost + vehicle_cost
            preposition_master_model.addConstr(first_stage_cost + recourse_vars.sum('*', s) >= obj_lb)

        return preposition_master_model

    def _good_sol_warm_start(self, preposition_master_model):
        # 230621, add upper bound to the objective
        # Solve the model with transportation subproblems
        parameters = deepcopy(PARAMETERS)
        exp_set = {'ut_cost': 6.5874, 'ut_times': 1, 'fc_times': 1, 'vc_times': 1, 'p_type': "TRANS"}
        SM = StochasticModels(instance, parameters, instance_type="NO_VRP", **exp_set)
        model_t = SM.deterministic_equivalent_problem()
        model_t.optimize()
        first_stage_solution = dict()
        for var in model_t.getVars():
            if any(kw in var.varName for kw in ['assign', 'location', 'stock']):
                first_stage_solution[var.varName] = var.x

        # Solve the model with routing subproblems, but fix first stage variables
        parameters = deepcopy(PARAMETERS)
        SM = StochasticModels(instance, parameters, instance_type="DE")
        model_v = SM.deterministic_equivalent_problem()
        for var_name, var_value in first_stage_solution.items():
            model_v.getVarByName(var_name).setAttr("ub", var_value)
            model_v.getVarByName(var_name).setAttr("lb", var_value)
        model_v.optimize()
        ub = model_v.getObjective().getValue()

        # Add upper bound to the objective
        preposition_master_model.addConstr(preposition_master_model.getObjective() <= ub)
        return preposition_master_model

    def preposition_master_model(self, heuristic_solution=None, fix_first_stage=False):
        model = super().preposition_master_model(heuristic_solution, fix_first_stage=fix_first_stage)

        if self.warm_start:
            warm_start_begin_time = time.perf_counter()
            # model = self._good_sol_warm_start(model)
            model = self._wait_and_see_warm_start(model)

            self.warm_start_time = time.perf_counter() - warm_start_begin_time
            if self.minus_warm_start_time:
                self.parameters['lbbd_time_lim'] -= self.warm_start_time

        # "User MIP start did not produce a new incumbent solution"
        # heuristic_solution = {f"PMP_{k}": v for k, v in self.feasible_solution_heuristic().items() if k == 'assign'}
        # model = self.use_decision_vars_value(model, heuristic_solution, fix_first_stage)

        # Use optimal PMP solution
        # from io_operator import read_gurobi_sol
        # solution = read_gurobi_sol(self.instance_name, 'BAC')[1]
        # solution = {f"PMP_{k}": v for k, v in solution.items() if k in ['assign']}
        # model = self.use_decision_vars_value(model, solution, fix_first_stage)

        return model

    def save(self, file_prefix, stat, sol):
        stat['warm_start_time'] = self.warm_start_time
        if self.minus_warm_start_time:
            stat['iteration_finish_time_dic'] = {
                k: v + self.warm_start_time for k, v in stat["iteration_finish_time_dic"].items()}
            stat["SolutionInfo"]["Runtime"] += self.warm_start_time

        with open(f"{file_prefix}.json", 'w') as f:
            f.write(json.dumps(stat, indent=4))

        objective, solution = sol[0], sol[1:]
        # Do not save VDSP solution, which is in the format: [(i, s, var_name, var_value)]
        solution = [a for a in solution if len(a) == 2]
        lines = [f"# Objective value = {objective}\n"]
        lines += [f"{a[0]} {a[1]}\n" for a in solution if 'aux' not in a[0]]
        with open(f"{file_prefix}.sol", 'w') as f:
            f.writelines(lines)


if __name__ == '__main__':
    # instance = '03_10_04_0_1'
    # LBBD = r_LogicBasedBendersDecomposition(instance, PARAMETERS, 'LBBD')
    # LBBD.resample(32, correlated_demand=False)
    # # stat, _ = LBBD.benders_solve(save_result=False)
    # stat, sol = LBBD.branch_and_check_solve(save_result=False)
    # # LBBD.save(f"r_{instance}", stat, sol)
    #
    # print("\n\n\n\n\n")
    #
    # SD = StochasticModels(LBBD.instance_name, PARAMETERS, 'DE')
    # SD.__dict__.update(LBBD.__dict__)
    # solved_model = SD.solve(write_to_file=False, time_limit=3600 * 8)

    instance = "03_40_02_0_1"
    LBBD = r_LogicBasedBendersDecomposition(instance, PARAMETERS, 'LBBD')
    stat, sol = LBBD.branch_and_check_solve(save_result=False)
