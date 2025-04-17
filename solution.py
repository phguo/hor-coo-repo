# coding:utf-8

from copy import deepcopy
from itertools import product

from instance import Instance
from io_operator import read_gurobi_sol, solved_instance_classify, get_objective_value


class GurobiSolution(Instance):
    def __init__(self, instance_name, solution_type, **info):
        Instance.__init__(self, instance_name, solution_type, **info)
        self.solution_type = solution_type

        self.solution, self.non_zero_solution = read_gurobi_sol(instance_name, solution_type, **info)
        self.obj = get_objective_value(instance_name, solution_type, **info)

        # x_{ij}         assign_vars_tup        bin
        # y_{ik}         location_vars_tup      bin
        # l_i            stock_vars_tup         int
        # w_i^h          vehicle_vars_tup       int
        # t_{ij}^s       direct_trans_vars_tup  int scenario-specified
        # t_{ii'j}^s     lateral_trans_vars_tup int scenario-specified
        # z_{pq}^(s,i,h) routing_tup            bin scenario-specified
        # f_{pq}^(s,i,h) commodity_flow_tup     int scenario-specified

        # x_{pq}^s       auxiliary_assign_tup   bin scenario-specified
        # Q_p^(s,i)      auxiliary_demand_tup   int scenario-specified

        # all variables
        # each variable is a tupledict
        self.assign = self.solution['assign']
        self.location = self.solution['location']
        self.stock = self.solution['stock']
        self.vehicle = self.solution['vehicle']
        self.direct_trans = self.solution['direct_trans']
        self.lateral_trans = self.solution['lateral_trans']
        self.routing = self.solution['routing']
        self.flow = self.solution['flow']
        # self.aux_assign = self.solution['aux_assign']
        # self.aux_demand = self.solution['aux_demand']

        # non-zero variables
        self.non_zero_assign = self.non_zero_solution['assign']
        self.non_zero_location = self.non_zero_solution['location']
        self.non_zero_stock = self.non_zero_solution['stock']
        self.non_zero_vehicle = self.non_zero_solution['vehicle']
        self.non_zero_direct_trans = self.non_zero_solution['direct_trans']
        self.non_zero_lateral_trans = self.non_zero_solution['lateral_trans']
        self.non_zero_routing = self.non_zero_solution['routing']
        self.non_zero_flow = self.non_zero_solution['flow']
        # self.non_zero_aux_assign = self.non_zero_solution['aux_assign']
        # self.non_zero_aux_demand = self.non_zero_solution['aux_demand']

        # self.service_level = ratio

        self.lateral_trans_applied = False
        if self.non_zero_lateral_trans != {}:
            self.lateral_trans_applied = True

        # human readable solution
        self.opened_facility = sorted([key[0] for key in self.non_zero_location])
        # in case more than one type of facility opened at a candidate location
        assert len(self.opened_facility) == len(set(self.opened_facility))
        self.opened_facility_type = dict()
        for i, k in self.non_zero_location:
            if i not in self.opened_facility_type:
                self.opened_facility_type[i] = list()
            self.opened_facility_type[i].append(k)
        self.facility_stock = {i: self.stock.select(i)[0] for i in self.opened_facility}
        self.used_vehicle_types = {key[-1] for key in self.non_zero_routing}
        self.facility_assigned_demand = {i: {j for (k, j) in self.non_zero_assign if k == i}
                                         for i in self.opened_facility}
        # self.facility_assigned_facility = {s: {i: [] for i in self.opened_facility} for s in self.scenarios}
        # for (k, j, s) in self.non_zero_aux_assign:
        #     if j in self.opened_facility and j != k:
        #         self.facility_assigned_facility[s][k].append(j)
        self.vehicle_preparing = {i: {h: self.non_zero_vehicle[(k, h)] for (k, h) in self.non_zero_vehicle if k == i}
                                  for i in self.opened_facility}

        self.direct_trans_dict = {}
        for (i, j, s) in self.non_zero_direct_trans:
            # initialize data structure
            if s not in self.direct_trans_dict:
                self.direct_trans_dict[s] = {}
            if i not in self.direct_trans_dict[s]:
                self.direct_trans_dict[s][i] = {}
            # add data
            self.direct_trans_dict[s][i][j] = self.non_zero_direct_trans.select(i, j, s)[0]

        self.lateral_trans_dict = {}
        for (i, k, j, s) in self.non_zero_lateral_trans:
            # initialize data structure
            if s not in self.lateral_trans_dict:
                self.lateral_trans_dict[s] = {}
            if i not in self.lateral_trans_dict[s]:
                self.lateral_trans_dict[s][i] = {}
            if k not in self.lateral_trans_dict[s][i]:
                self.lateral_trans_dict[s][i][k] = {}
            # add data
            self.lateral_trans_dict[s][i][k][j] = self.non_zero_lateral_trans.select(i, k, j, s)[0]

        self.scenarios_routes = self.get_scenarios_routes()

        self.routing_cost = {}
        for s in self.scenarios_routes:
            c = 0
            for i in self.scenarios_routes[s]:
                for h in self.scenarios_routes[s][i]:
                    routes = self.scenarios_routes[s][i][h]
                    for route in routes:
                        cost = self._get_route_distance(route) * self.vehicle_info[h]['uc']
                        c += cost
            self.routing_cost[s] = c

    @staticmethod
    def arcs_to_routes(arcs, facility_id):
        a = list(zip(*arcs))[0] + list(zip(*arcs))[1]
        visited_points_count = len(a) / 2
        arcs_between_points_count = len(arcs)
        assert visited_points_count == arcs_between_points_count, 'NOT a closed tour'

        routes = []
        arcs = sorted(arcs)
        tmp_arcs = deepcopy(arcs)
        for pair in arcs:
            if pair[0] == facility_id:
                route = [pair[0], pair[1]]
                tmp_arcs.remove(pair)
                routes.append(deepcopy(route))
        while tmp_arcs:
            for route in routes:
                for arc in arcs:
                    if arc[0] == route[-1] and route[-1] != facility_id:
                        route.append(arc[1])
                        tmp_arcs.remove(arc)
        return routes

    def get_scenarios_routes(self):
        # scenarios_solution = {
        # scenario_id: {
        #     facility_id: {
        #         vehicle_type_id: {
        #             [[route_by_vehicle], ..., []]
        # }}}}

        scenarios_solution_routes = {s: {i: {} for i in self.opened_facility} for s in self.scenarios}
        for (s, i, h) in product(self.scenarios, self.opened_facility, self.used_vehicle_types):
            if self.non_zero_routing.select('*', '*', s, i, h):
                # if vehicle h used in (s,i)
                arcs = [(p, q) for (p, q, ss, ii, hh) in self.non_zero_routing if (ss, ii, hh) == (s, i, h)]
                routes = self.arcs_to_routes(arcs, i)
                rs = [a for a in routes if not (len(a) == 2 and a[0] == a[1])]
                scenarios_solution_routes[s][i][h] = rs

        return scenarios_solution_routes


def main():
    # test_instances = []
    # optimized, feasible = solved_instance_classify('STC')[:2]
    # test_instances += [(instance, 'STC', info) for instance, info in optimized]
    # test_instances += [(instance, 'STC', info) for instance, info in feasible]
    # for a, b, c in test_instances:
    #     GurobiSolution(a, b, **c)

    GurobiSolution("03_10_02_0_1", "LBBD", **{})


if __name__ == "__main__":
    main()
