# coding:utf-8
import random
import time
from copy import deepcopy
from itertools import product
from math import ceil

from gurobipy.gurobipy import tupledict, LinExpr, GRB, quicksum, Model

from config import PARAMETERS, GUROBI_PARAMETERS, EXPERIMENTAL_INSTANCES_NAME, RANDOM_SEED
from instance import Instance
from io_operator import write_gurobi_results, read_model, solution_file_name_process, read_gurobi_sol, write_model

random.seed(RANDOM_SEED)


class StochasticModels(Instance):

    def __init__(self, instance_name, parameters, instance_type, **exp_set):

        self.instance_name = instance_name
        self.parameters = parameters
        self.instance_type = instance_type
        self.exp_set = exp_set

        self.big_m = self.parameters['big_m']
        self.epsilon = self.parameters['epsilon']
        self.direct_trans_first = self.parameters['direct_trans_first']
        self.max_linearized_by_gurobi = self.parameters['max_linearized_by_gurobi']
        self.service_level_constraints = self.parameters['service_level_constraint']
        self.service_level = self.parameters['service_level']
        self.back_and_forth_approximation = self.parameters['back_and_forth_approximation']
        self.vehicle_routing_sub_problem = self.parameters['vehicle_routing_sub_problem']
        assert self.vehicle_routing_sub_problem != self.back_and_forth_approximation
        self.integer_transportation_decision = self.parameters.get('integer_transportation_decision', True)
        self.valid_inequality = self.parameters.get('valid_inequality', True)

        Instance.__init__(self, instance_name, instance_type, **exp_set)

        self.big_m_location_trans = {s: self.big_m for s in self.scenarios}
        self.big_m_aux_assign = self.big_m_location_trans
        self.big_m_assign_flow = {h: self.big_m for h in self.vehicle_types}
        if self.parameters.get('use_local_big_m', False):
            # total demand of all the demand points in scenario s \in S
            self.big_m_location_trans = {s: sum(self.scenarios[s][j][0] for j in self.demand_set)
                                         for s in self.scenarios}
            # for manual linearization for max() function
            self.big_m_aux_assign = self.big_m_location_trans
            # capacity of vehicle of type h \in H
            self.big_m_assign_flow = {h: self.vehicle_info[h]["cap"] for h in self.vehicle_types}

    def set_gurobi_parameters(self, model):
        for key, value in GUROBI_PARAMETERS.items():
            if key == "LogFile":
                file_name = solution_file_name_process(self.instance_name, self.instance_type, **self.exp_set)[1]
                file_name = file_name.replace(self.instance_name, self.instance_name + '_{}'.format(self.instance_type))
                file_name = "{}_{}".format(time.strftime("%Y-%m-%d-%H-%M-%S", time.localtime()), file_name)
                value = value.format(file_name)
            if key != "LBBDToConsole":
                model.setParam(key, value)
        return model

    def _make_main_model(self, model):

        # define main problem variables
        # x_{ij}     assign_vars_tup        bin
        # y_{ik}     location_vars_tup      bin
        # l_i        stock_vars_tup         int
        # w_i^h      vehicle_vars_tup       int
        # t_{ij}^s   direct_trans_vars_tup  int
        # t_{ii'j}^s lateral_trans_vars_tup int

        assign_vars_tup = tupledict([(i, j), -1] for (i, j) in product(self.facility_set, self.demand_set))
        location_vars_tup = tupledict([(i, k), -1] for i in self.facility_set for k in self.facility_types)
        stock_vars_tup = tupledict([(i), -1] for i in self.facility_set)
        vehicle_vars_tup = tupledict(
            [(i, h), -1] for (i, h) in product(self.facility_set, self.vehicle_types))
        direct_trans_vars_tup = tupledict(
            [(i, j, s), -1] for (i, j, s) in product(self.facility_set, self.demand_set, self.scenarios))
        lateral_trans_vars_tup = tupledict(
            [(i, k, j, s), -1] for (i, k, j, s) in
            product(self.facility_set, self.facility_set, self.demand_set, self.scenarios))

        # add main problem variables
        assign_vars = model.addVars(assign_vars_tup, vtype=GRB.BINARY, name="assign")
        location_vars = model.addVars(location_vars_tup, vtype=GRB.BINARY, name="location")
        stock_vars = model.addVars(stock_vars_tup, vtype=GRB.CONTINUOUS, lb=0, name="stock")
        vehicle_vars = model.addVars(vehicle_vars_tup, vtype=GRB.INTEGER, lb=0, name="vehicle")
        variable_type = GRB.INTEGER if self.integer_transportation_decision else GRB.CONTINUOUS
        direct_trans_vars = model.addVars(direct_trans_vars_tup, vtype=variable_type, lb=0, name="direct_trans")
        lateral_trans_vars = model.addVars(lateral_trans_vars_tup, vtype=variable_type, lb=0, name="lateral_trans")

        # 1st-stage cost
        location_cost = LinExpr([self.facility_open_cost[i][k] for i, k in location_vars_tup],
                                [location_vars[i, k] for i, k in location_vars_tup])
        stock_cost = LinExpr([self.unit_facility_acquisition_cost for _ in stock_vars_tup],
                             [stock_vars[i] for i in stock_vars_tup])
        vehicle_cost = LinExpr([self.vehicle_info[h]['fc'] for (_, h) in vehicle_vars_tup],
                               [vehicle_vars[i, h] for (i, h) in vehicle_vars_tup])
        first_stage_cost = location_cost + stock_cost + vehicle_cost

        # set cost objective function
        # NOTICE - for "NO_VRP"
        if self.instance_type == "NO_VRP" and self.problem_type == "TRANS":
            # 2nd-stage back-and-forth route approximation transportation cost
            expected_back_and_forth_direct_trans_cost = LinExpr(
                [self.scenario_prob[s] * self.unit_direct_trans_cost * self._get_route_distance([i, j])
                 for (i, j, s) in direct_trans_vars_tup],
                [direct_trans_vars[i, j, s] for (i, j, s) in direct_trans_vars])
            expected_back_and_forth_lateral_trans_cost = LinExpr(
                [self.scenario_prob[s] * self.unit_lateral_trans_cost * self._get_route_distance([i, k, j])
                 for (i, k, j, s) in lateral_trans_vars_tup],
                [lateral_trans_vars[i, k, j, s] for (i, k, j, s) in lateral_trans_vars_tup])
            model.setObjective(
                first_stage_cost +
                expected_back_and_forth_direct_trans_cost + expected_back_and_forth_lateral_trans_cost)
        else:
            model.setObjective(first_stage_cost)

        # add constraints
        # first-stage constraints
        model.addConstrs(
            location_vars.sum(i, '*') <= 1 for i in self.facility_set)
        model.addConstrs(
            stock_vars[i] <=
            quicksum(location_vars[i, k] * self.facility_cap_types[k] for k in self.facility_types)
            for i in self.facility_set)
        model.addConstrs(
            assign_vars.sum('*', j) == 1 for j in self.demand_set)
        model.addConstrs(
            assign_vars[i, j] <= location_vars.sum(i, '*') for i in self.facility_set for j in self.demand_set)
        model.addConstrs(
            vehicle_vars.sum('*', h) <= self.vehicle_info[h]['num'] for h in self.vehicle_types)

        # second-stage constraints
        model.addConstrs(
            direct_trans_vars[i, j, s] <= self.big_m_location_trans[s] * assign_vars[i, j]
            for (i, j, s) in direct_trans_vars_tup)
        model.addConstrs(
            lateral_trans_vars[i, k, j, s] <= self.big_m_location_trans[s] * assign_vars[k, j]
            for (i, k, j, s) in lateral_trans_vars_tup)
        model.addConstrs(
            lateral_trans_vars.sum(i, k, '*', s) <= self.big_m_location_trans[s] * location_vars.sum(i, '*')
            for (i, k, s) in product(self.facility_set, self.facility_set, self.scenarios))
        model.addConstrs(
            lateral_trans_vars.sum(i, k, '*', s) <= self.big_m_location_trans[s] * location_vars.sum(k, '*')
            for (i, k, s) in product(self.facility_set, self.facility_set, self.scenarios))
        model.addConstrs(
            direct_trans_vars.sum(i, '*', s) + lateral_trans_vars.sum(i, '*', '*', s) <= stock_vars[i]
            for (i, s) in product(self.facility_set, self.scenarios))
        model.addConstrs(
            direct_trans_vars.sum("*", j, s) + lateral_trans_vars.sum("*", "*", j, s)
            <= self.scenarios[s][j][0] for (j, s) in product(self.demand_set, self.scenarios))

        # add service level constraints
        if self.service_level_constraints:
            model.addConstrs(direct_trans_vars.sum("*", j, s) + lateral_trans_vars.sum("*", "*", j, s)
                             >= ceil(self.scenarios[s][j][0] * self.service_level)
                             for (j, s) in product(self.demand_set, self.scenarios))

        # linearization for max() function
        if self.direct_trans_first and self.max_linearized_by_gurobi:
            # define auxiliary variables for linearization of max()
            # theta_{is}  int
            aux_theta_vars_tup = tupledict([(i, s), -1] for (i, s) in product(self.facility_set, self.scenarios))
            # add auxiliary variables for linearization of max()
            aux_theta_vars = model.addVars(aux_theta_vars_tup, vtype=GRB.INTEGER, name="max_aux_theta")

            tmp_aux_vars_tup = tupledict(
                [(i, s), -1] for (i, s) in product(self.facility_set, self.scenarios))
            tmp_aux_vars = model.addVars(tmp_aux_vars_tup, vtype=GRB.INTEGER, name="tmp_aux_vars")
            for (i, s) in product(self.facility_set, self.scenarios):
                model.addConstr(tmp_aux_vars[i, s]
                                == stock_vars[i]
                                - quicksum(assign_vars[i, j] * self.scenarios[s][j][0] for j in self.demand_set))
                model.addGenConstrMax(aux_theta_vars[i, s], [0, tmp_aux_vars[i, s]])
                model.addConstr(lateral_trans_vars.sum(i, '*', '*', s) <= aux_theta_vars[i, s])

        elif self.direct_trans_first and not self.max_linearized_by_gurobi:
            # define auxiliary variables for linearization of max()
            # lambda_{is} bin
            # mu_{is}     bin
            # theta_{is}  int
            aux_lambda_vars_tup = tupledict([(i, s), -1] for (i, s) in product(self.facility_set, self.scenarios))
            aux_mu_vars_tup = tupledict([(i, s), -1] for (i, s) in product(self.facility_set, self.scenarios))
            aux_theta_vars_tup = tupledict([(i, s), -1] for (i, s) in product(self.facility_set, self.scenarios))
            # add auxiliary variables for linearization of max()
            aux_lambda_vars = model.addVars(aux_lambda_vars_tup, vtype=GRB.BINARY, name="max_aux_lambda")
            aux_mu_vars = model.addVars(aux_mu_vars_tup, vtype=GRB.BINARY, name="max_aux_mu")
            aux_theta_vars = model.addVars(aux_theta_vars_tup, vtype=GRB.INTEGER, name="max_aux_theta")

            for (i, s) in product(self.facility_set, self.scenarios):
                model.addConstr(lateral_trans_vars.sum(i, '*', '*', s) <= aux_theta_vars[i, s])
                model.addConstr(0 <= aux_theta_vars[i, s])
                model.addConstr(stock_vars[i]
                                - quicksum(assign_vars[i, j] * self.scenarios[s][j][0] for j in self.demand_set)
                                <= aux_theta_vars[i, s])
                model.addConstr(0 >= aux_theta_vars[i, s] - self.big_m * (1 - aux_lambda_vars[i, s]))
                model.addConstr(stock_vars[i]
                                - quicksum(assign_vars[i, j] * self.scenarios[s][j][0] for j in self.demand_set)
                                >= aux_theta_vars[i, s] - self.big_m * (1 - aux_mu_vars[i, s]))
                model.addConstr(aux_lambda_vars[i, s] + aux_mu_vars[i, s] >= 1)
                model.addConstr(lateral_trans_vars.sum(i, '*', '*', s) <= aux_theta_vars[i, s])

        if self.valid_inequality:
            # forbidden self transportation
            model.addConstrs(lateral_trans_vars.sum(i, i, '*', '*') == 0 for i in self.facility_set)
            model.addConstr(stock_vars.sum('*') >= self.service_level *
                            max(sum(self.scenarios[s][j][0] for j in self.scenarios[s]) for s in self.scenarios))

        model.update()

        # NOTICE - for "NO_VRP"
        if self.fix_first_stage:
            read_exp_set = deepcopy(self.exp_set)
            read_exp_set.update({'p_type': "TRANS"})
            try:
                first_stage_vars_value = read_gurobi_sol(self.instance_name, self.instance_type, **read_exp_set)[0]
                vars_value = {
                    'assign': first_stage_vars_value['assign'],
                    'location': first_stage_vars_value['location'],
                    'stock': first_stage_vars_value['stock']
                }
            except:
                raise Exception('{} {} need to be solved.'.format(
                    self.instance_name, read_exp_set))

            model = self._use_first_stage_decisions(model, vars_value, True)

        # NOTICE - for "STC"
        if self.p_type == "EEV" and self.expected_value_problem != True:
            read_exp_set = deepcopy(self.exp_set)
            read_exp_set.update({'ev_p': True})
            try:
                first_stage_vars_value = read_gurobi_sol(self.instance_name, self.instance_type, **read_exp_set)[0]
                vars_value = {
                    'assign': first_stage_vars_value['assign'],
                    'location': first_stage_vars_value['location'],
                    'stock': first_stage_vars_value['stock']
                }
            except:
                raise Exception('{} {} need to be solved.'.format(self.instance_name, read_exp_set))

            model = self._use_first_stage_decisions(model, vars_value, True)

        # model.update()
        return model

    def _add_second_stage_decisions(self, model):
        # This function add vehicle routing sub problem variable, constraint, and related objective function
        # NOTICE - for "NO_VRP"
        if self.instance_type == "NO_VRP" and self.problem_type == "TRANS":
            assert self.back_and_forth_approximation
            return model

        # NOTICE - for "NO_VRP"
        if self.vehicle_routing_sub_problem:
            assert not self.back_and_forth_approximation

        def exclude_i(points, i):
            # changed set([i]) to {i}
            # return sorted(list(set(points) - {i}))
            return set(points) - {i}

        # copy the model from _make_main_model which including the first-stage decisions and costs
        #  - assign_vars
        #  - vehicle_vars
        #  - first_stage_cost
        # as well as the second-stage decisions:
        #  - direct_trans_vars
        #  - lateral_trans_vars
        assign_vars = tupledict(
            [(i, j), model.getVarByName("assign[{0},{1}]".format(i, j))]
            for (i, j) in product(self.facility_set, self.demand_set))
        direct_trans_vars = tupledict(
            [(i, j, s), model.getVarByName("direct_trans[{0},{1},{2}]".format(i, j, s))]
            for (i, j, s) in product(self.facility_set, self.demand_set, self.scenarios))
        lateral_trans_vars = tupledict(
            [(i, k, j, s), model.getVarByName("lateral_trans[{0},{1},{2},{3}]".format(i, k, j, s))]
            for (i, k, j, s) in product(self.facility_set, self.facility_set, self.demand_set, self.scenarios))
        vehicle_vars = tupledict(
            [(i, h), model.getVarByName("vehicle[{0},{1}]".format(i, h))]
            for (i, h) in product(self.facility_set, self.vehicle_types))

        # define deterministic equivalent sub problem variables
        # z_{pq}^(s,i,h) routing_tup          bin
        # f_{pq}^(s,i,h) commodity_flow_tup   int
        # x_{pq}^s       auxiliary_assign_tup bin
        # Q_p^(s,i)      auxiliary_demand_tup int
        routing_vars_tup = tupledict(
            [(p, q, s, i, h), -1] for (p, q, s, i, h) in
            product(self.points_set, self.points_set, self.scenarios, self.facility_set, self.vehicle_types))
        commodity_flow_vars_tup = tupledict(
            [(p, q, s, i, h), -1] for (p, q, s, i, h) in
            product(self.points_set, self.points_set, self.scenarios, self.facility_set, self.vehicle_types))
        auxiliary_assign_vars_tup = tupledict(
            [(p, q, s), -1] for (p, q, s) in
            product(self.points_set, self.points_set, self.scenarios))
        auxiliary_demand_vars_tup = tupledict(
            [(p, s, i), -1] for (p, s, i) in
            product(self.points_set, self.scenarios, self.points_set))

        # add deterministic equivalent sub problem variables
        routing_vars = model.addVars(routing_vars_tup, vtype=GRB.BINARY, name="routing")
        commodity_flow_vars = model.addVars(commodity_flow_vars_tup, vtype=GRB.CONTINUOUS, name="flow")
        auxiliary_assign_vars = model.addVars(auxiliary_assign_vars_tup, vtype=GRB.BINARY, name="aux_assign")
        auxiliary_demand_vars = model.addVars(auxiliary_demand_vars_tup, vtype=GRB.CONTINUOUS, name="aux_demand")

        # add vehicle routing sub problem cost
        # the 2nd-stage vehicle preparing cost is DISCARDED
        # expected_vehicle_preparing_cost = 0
        # for i in self.facility_set:
        #     expected_vehicle_preparing_cost += LinExpr(
        #         [self.scenario_prob[s] * self.vehicle_info[h]['sc'] for (q, s, h) in
        #          product(exclude_i(self.points_set, i), self.scenarios, self.vehicle_types)],
        #         [routing_vars[i, q, s, i, h] for (q, s, h) in
        #          product(exclude_i(self.points_set, i), self.scenarios, self.vehicle_types)])

        expected_vehicle_routing_cost = LinExpr(
            [self.scenario_prob[s] * self.vehicle_info[h]['uc'] * self._get_distance(p, q)
             for p, q, s, i, h in routing_vars_tup],
            [routing_vars[p, q, s, i, h]
             for p, q, s, i, h in routing_vars_tup])

        # set cost objective function
        first_stage_cost = model.getObjective()
        model.setObjective(first_stage_cost + expected_vehicle_routing_cost)

        # add vehicle routing sub problem constraints
        model.addConstrs(
            quicksum(routing_vars[i, q, s, i, h] for q in exclude_i(self.points_set, i)) <= vehicle_vars[i, h]
            for (i, h) in product(self.facility_set, self.vehicle_types) for s in self.scenarios)
        model.addConstrs(
            auxiliary_assign_vars[p, q, s] == assign_vars[p, q]
            for (p, q) in product(self.facility_set, self.demand_set) for s in self.scenarios)
        model.addConstrs(
            auxiliary_assign_vars[p, p, s] == 1 for p in self.facility_set for s in self.scenarios)
        model.addConstrs(
            self.big_m_aux_assign[s] * auxiliary_assign_vars[p, q, s] >= lateral_trans_vars.sum(p, q, '*', s)
            for (p, q) in product(self.facility_set, self.facility_set) if p != q for s in self.scenarios)
        model.addConstrs(
            auxiliary_assign_vars[p, q, s] <= lateral_trans_vars.sum(p, q, '*', s)
            for (p, q) in product(self.facility_set, self.facility_set) if p != q for s in self.scenarios)

        model.addConstrs(
            routing_vars.sum('*', q, s, i, '*') == auxiliary_assign_vars[i, q, s]
            for (q, s, i) in product(self.points_set, self.scenarios, self.facility_set)
            if q in exclude_i(self.points_set, i))
        model.addConstrs(
            quicksum(routing_vars[p, q, s, i, h]
                     for (p, h) in product(self.points_set, self.special_vehicle_types))
            == auxiliary_assign_vars[i, q, s]
            for (q, s, i) in product(self.points_set, self.scenarios, self.facility_set)
            if q in self.isolated_demand_in_scenario[s])
        model.addConstrs(
            routing_vars.sum('*', q, s, i, h) - routing_vars.sum(q, '*', s, i, h) == 0
            for (q, s, i, h) in product(self.points_set, self.scenarios, self.facility_set, self.vehicle_types)
            if q in exclude_i(self.points_set, i))
        model.addConstrs(
            commodity_flow_vars[p, q, s, i, h] <= self.big_m_assign_flow[h] * auxiliary_assign_vars[i, p, s]
            for (p, q, h, s, i)
            in product(self.points_set, self.points_set, self.vehicle_types, self.scenarios, self.facility_set))
        model.addConstrs(
            commodity_flow_vars[p, q, s, i, h] <= self.big_m_assign_flow[h] * auxiliary_assign_vars[i, q, s]
            for (p, q, h, s, i)
            in product(self.points_set, self.points_set, self.vehicle_types, self.scenarios, self.facility_set))
        model.addConstrs(
            commodity_flow_vars.sum('*', p, s, i, '*') - commodity_flow_vars.sum(p, '*', s, i, '*')
            == auxiliary_demand_vars[p, s, i]
            for (p, s, i) in product(self.points_set, self.scenarios, self.facility_set)
            if p in exclude_i(self.points_set, i))
        model.addConstrs(
            commodity_flow_vars[p, q, s, i, h] >= auxiliary_demand_vars[q, s, i] * routing_vars[p, q, s, i, h]
            for (p, q, h, s, i)
            in product(self.points_set, self.points_set, self.vehicle_types, self.scenarios, self.facility_set))
        model.addConstrs(
            commodity_flow_vars[p, q, s, i, h] <=
            (self.vehicle_info[h]['cap'] - auxiliary_demand_vars[p, s, i]) * routing_vars[p, q, s, i, h]
            for (p, q, h, s, i)
            in product(self.points_set, self.points_set, self.vehicle_types, self.scenarios, self.facility_set))

        # definition for Q_p^{(s,i)}
        model.addConstrs(
            auxiliary_demand_vars[p, s, i] ==
            direct_trans_vars[i, p, s] + lateral_trans_vars.sum('*', i, p, s)
            for (i, p, s) in product(self.facility_set, self.demand_set, self.scenarios))
        model.addConstrs(
            auxiliary_demand_vars[p, s, i] == lateral_trans_vars.sum(i, p, '*', s)
            for (p, i, s) in product(self.facility_set, self.facility_set, self.scenarios) if p != i)
        model.addConstrs(
            auxiliary_demand_vars[p, s, i] == 0
            for (p, i, s) in product(self.facility_set, self.facility_set, self.scenarios) if p == i)

        if self.valid_inequality:
            # add commodity flow constraint, vehicle return empty to depot
            model.addConstrs(
                commodity_flow_vars[p, i, s, i, h] == 0
                for (p, i, h, s) in product(self.points_set, self.facility_set, self.vehicle_types, self.scenarios))
            # open facility only if at least one point is assigned to it
            location_vars = tupledict(
                [(i, k), model.getVarByName("location[{},{}]".format(i, k))]
                for (i, k) in product(self.facility_set, self.facility_types))
            model.addConstrs(
                location_vars.sum(i, '*') <=
                quicksum(auxiliary_assign_vars[i, q, s]
                         for i, q, s in product(self.facility_set, exclude_i(self.points_set, i), self.scenarios))
                for i in self.facility_set)
            # vehicle number upper bound
            model.addConstrs(
                vehicle_vars[i, h] <=
                quicksum(auxiliary_assign_vars[i, q, s]
                         for i, q, s in product(self.facility_set, exclude_i(self.points_set, i), self.scenarios))
                for i, h in product(self.facility_set, self.vehicle_types))
            # vehicle number lower bound
            model.addConstrs(
                quicksum(vehicle_vars[i, h] * self.vehicle_info[h]["cap"] for h in self.vehicle_types) >=
                direct_trans_vars.sum(i, '*', s) +
                lateral_trans_vars.sum('*', i, '*', s) + lateral_trans_vars.sum(i, '*', '*', s)
                for i, s in product(self.facility_set, self.scenarios))
            # special vehicle number lower bound
            model.addConstrs(
                quicksum(vehicle_vars[i, h] * self.vehicle_info[h]["cap"] for h in self.special_vehicle_types) >=
                quicksum(direct_trans_vars[i, j, s] for j in self.isolated_demand_in_scenario[s]) +
                quicksum(lateral_trans_vars[k, i, j, s] for k, j in
                         product(self.facility_set, self.isolated_demand_in_scenario[s]))
                for i, s in product(self.facility_set, self.scenarios))

        model.update()
        return model

    def _use_first_stage_decisions(self, model, first_stage_vars_value, fixed):
        # This function fix first-stage decision variable values

        # --------- 1ST STAGE DECISIONS ---------
        # x_{ij}     assign_vars_tup          bin
        # y_{ik}     location_vars_tup        bin
        # l_i        stock_vars_tup           int
        # w_i^h      vehicle_vars_tup         int

        # ------- 2(.1)ND STAGE DECISIONS -------
        # t_{ij}^s   direct_trans_vars_tup    int
        # t_{ii'j}^s lateral_trans_vars_tup   int

        # ------- 2(.2)ND STAGE DECISIONS -------
        # z_{pq}^(s,i,h) routing_tup          bin
        # f_{pq}^(s,i,h) commodity_flow_tup   int
        # x_{pq}^s       auxiliary_assign_tup bin
        # Q_p^(s,i)      auxiliary_demand_tup int

        for var_type, var_content in first_stage_vars_value.items():
            for key, var_value in var_content.items():
                var_id = ','.join((str(a) for a in key))
                var_name = "{}[{}]".format(var_type, var_id)
                if fixed:
                    model.getVarByName(var_name).setAttr("ub", var_value)
                    model.getVarByName(var_name).setAttr("lb", var_value)
                else:
                    model.getVarByName(var_name).setAttr("start", var_value)

        model.update()
        return model

    def deterministic_equivalent_problem(self, use_cached_model=PARAMETERS.get('use_cached_model', False)):
        def make_deterministic_equivalent_problem():
            input_model = Model()
            input_model.update()
            main_model = self._make_main_model(input_model)
            de_model = self._add_second_stage_decisions(main_model)
            de_model.update()
            # write_model(de_model, self.instance_name, self.instance_type)
            return de_model

        if use_cached_model:
            try:
                de_model = read_model(self.instance_name)
            except:
                de_model = make_deterministic_equivalent_problem()
        else:
            de_model = make_deterministic_equivalent_problem()

        # de_model = make_deterministic_equivalent_problem()
        de_model = self.set_gurobi_parameters(de_model)
        return de_model

    def solve(self, write_to_file=True, time_limit=None):
        title_length = 150
        print(' \"{} | {} | {}\" processing... '.format(
            self.instance_name, self.instance_type, self.exp_set).center(title_length, '>'))

        model = self.deterministic_equivalent_problem()
        # model.write("{}_{}.mps".format(self.instance_name, self.instance_type))
        # write_model(model, self.instance_name, instance_type="LT", **self.exp_set)
        model.setParam("LogToConsole", True)
        if time_limit:
            model.setParam("TimeLimit", time_limit)
        model.optimize()

        if write_to_file:
            write_gurobi_results(model, self.instance_name, self.instance_type, **self.exp_set)

        return model


def main():
    for instance_name in EXPERIMENTAL_INSTANCES_NAME:
        s_model = StochasticModels(instance_name, PARAMETERS, "DE")
        s_model.solve(write_to_file=False)


if __name__ == '__main__':
    # generate_experimental_instance()
    main()
