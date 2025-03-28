# coding:utf-8
from itertools import product, permutations, combinations

from gurobipy import Model, LinExpr, quicksum, GRB, tupledict, tuplelist

from config import GUROBI_PARAMETERS, EXPERIMENTAL_INSTANCES_NAME
from instance import Instance

from solution import GurobiSolution
import random


def print_model_nonzeros(model, keyword=None):
    if model.getAttr("Status") not in [3, 4, 5]:
        for var in model.getVars():
            if var.getAttr('x') > 0.5 and "aux" not in var.getAttr("VarName"):
                if keyword != None and keyword in var.getAttr("VarName"):
                    print(var.getAttr("VarName"), var.getAttr('x'))
        print('\n')
    else:
        print("Either infeasible or unbounded model.\n")


def gurobi_model(model_name):
    model = Model(model_name)
    for key, value in GUROBI_PARAMETERS.items():
        model.setParam(key, value)
    return model


class BinPacking(object):

    def __init__(self, items, bin_capacity):
        # items: dict({item_id, item_size}, ...)
        # bin_capacity: int
        self.items = items
        self.bins = [i for i, _ in enumerate(items)]
        self.cap = bin_capacity

    def __str__(self):
        return "BinPacking:[{}, {}]".format(self.cap, self.items)

    def mip_solve(self):
        bin_number = None
        if self.items:
            # initialize model
            model = gurobi_model("BinPacking")
            # add vars
            assign_vars = model.addVars(
                ((i, j) for i, j in product(self.bins, self.items)), vtype=GRB.BINARY, name="BP_assign")
            bin_vars = model.addVars(
                ((i) for i in self.bins), vtype=GRB.BINARY, name="BP_bin")
            # set objective function
            objective = LinExpr(bin_vars.sum("*"))
            model.setObjective(objective)
            # add constraints
            model.addConstrs(assign_vars.sum('*', j) == 1 for j in self.items)
            model.addConstrs(
                quicksum(assign_vars[i, j] * self.items[j] for j in self.items) <=
                self.cap * bin_vars[i] for i in self.bins)
            model.optimize()
            if model.getAttr("Status") == GRB.OPTIMAL:
                bin_number = model.getObjective().getValue()
            else:
                raise Exception("Infeasible bin packing problem.")
        # print("BinPacking: ", self.items, bin_number)
        return bin_number

    def ffd_solve(self):
        bin_number = 0
        return bin_number


class CapVehicleRouting(object):
    def __init__(
            self, depots_customers, vehicle_preparation,
            vehicle_info, vehicle_types, special_vehicle_types, is_isolated,
            _get_distance, i, s
    ):
        self.depots_customers = depots_customers
        self.vehicle_preparation = vehicle_preparation

        self.vehicle_info = vehicle_info
        self.vehicle_types = vehicle_types
        self.special_vehicle_types = special_vehicle_types

        self.is_isolated = is_isolated
        self._get_distance = _get_distance
        self.i = i
        self.s = s

        # For three_index_model_with_callback
        self.points = self.depots_customers.keys()
        self.customers = [a for a in self.depots_customers if a != self.i]
        self.vehicles = []
        self.vehicle_classify = dict()
        self.vehicle_group = dict()
        for h in self.vehicle_preparation[self.s]:
            type_h_vehicle_num = self.vehicle_preparation[self.s][h]
            if self.vehicles:
                type_h_vehicle_li = [
                    a for a in range(self.vehicles[-1] + 1, self.vehicles[-1] + 1 + type_h_vehicle_num)]
            else:
                type_h_vehicle_li = [a for a in range(type_h_vehicle_num)]
            self.vehicles += type_h_vehicle_li
            self.vehicle_group[h] = type_h_vehicle_li
            for a in type_h_vehicle_li:
                self.vehicle_classify[a] = h

    def single_commodity_flow_model(self):
        model = gurobi_model("CapVehicleRouting-CommodityFlow")
        model.setParam("MIPGap", 0.5)

        routing_vars_tup = tupledict(
            [(p, q, h), -1] for (p, q, h) in product(self.points, self.points, self.vehicle_types))
        commodity_flow_vars_tup = tupledict(
            [(p, q, h), -1] for (p, q, h) in product(self.points, self.points, self.vehicle_types))

        routing_vars = model.addVars(routing_vars_tup, vtype=GRB.BINARY, name="VDSP_routing")
        commodity_flow_vars = model.addVars(commodity_flow_vars_tup, vtype=GRB.INTEGER, lb=0, name="VDSP_flow")

        vehicle_traveling_cost = LinExpr(
            [self.vehicle_info[h]['uc'] * self._get_distance(p, q) for p, q, h in routing_vars_tup],
            [routing_vars[p, q, h] for p, q, h in routing_vars_tup]
        )
        model.setObjective(vehicle_traveling_cost)

        model.addConstrs(
            quicksum(routing_vars[self.i, q, h] for q in self.customers) <= self.vehicle_preparation[self.s][h]
            for h in self.vehicle_types)
        model.addConstrs(
            routing_vars.sum('*', q, '*') == 1 for q in self.customers)
        model.addConstrs(
            quicksum(routing_vars.sum('*', q, h) for h in self.special_vehicle_types) == 1
            for q in [key for key in self.depots_customers if self.is_isolated(self.s, key)])
        model.addConstrs(
            routing_vars.sum('*', q, h) - routing_vars.sum(q, '*', h) == 0
            for q in self.customers for h in self.vehicle_types)
        model.addConstrs(
            quicksum(commodity_flow_vars.sum(q, p, '*') for q in self.points) -
            quicksum(commodity_flow_vars.sum(p, q, '*') for q in self.points)
            == self.depots_customers[p]
            for p in self.customers)
        model.addConstrs(
            commodity_flow_vars[p, q, h] >= self.depots_customers[q] * routing_vars[p, q, h]
            for p, q, h in product(self.points, self.points, self.vehicle_types))
        model.addConstrs(
            commodity_flow_vars[p, q, h] <=
            (self.vehicle_info[h]['cap'] - self.depots_customers[p]) * routing_vars[p, q, h]
            for p, q, h in product(self.points, self.points, self.vehicle_types))

        model.addConstrs(
            commodity_flow_vars[p, self.i, h] == 0
            for p, h in product(self.customers, self.vehicle_types))
        model.addConstrs(routing_vars[self.i, self.i, h] == 0 for h in self.vehicle_types)

        # model.addConstrs(routing_vars[i, i, h] == 0 for i in self.points for h in self.vehicle_types)

        model.optimize()

        return model

    def subtour(self, edges):
        unvisited = list(set(e[0] for e in edges) | set(e[1] for e in edges))
        cycles = []
        while unvisited:
            this_cycle = []
            neighbors = unvisited
            while neighbors:
                current = neighbors[0]
                this_cycle.append(current)
                unvisited.remove(current)
                neighbors = [j for i, j in edges.select(current, '*') if j in unvisited]
            cycles.append(this_cycle[:])
        return cycles

    def three_index_model_with_callback(self):
        # NOTICE - typically slower than single_commodity_flow_model()

        def subtour_elimination(model, where):
            if where == GRB.Callback.MIPSOL:
                var_values = model.cbGetSolution(model._vars)
                selected_tour = []
                subtour_len = float("inf")
                for v in self.vehicles:
                    edges = tuplelist((i, j) for i, j, r in model._vars.keys() if var_values[i, j, r] > 0.5 and r == v)
                    assigned_to_v = set(e[0] for e in edges) | set(e[1] for e in edges)
                    assigned_to_v.add(self.i)
                    for tour in self.subtour(edges):
                        if len(tour) < len(assigned_to_v):
                            if len(tour) < subtour_len:
                                subtour_len = len(tour)
                                selected_tour = tour
                if selected_tour:
                    lhs = LinExpr(
                        quicksum(
                            model._vars[i, j, r] for i, j in product(selected_tour, selected_tour)
                            for r in self.vehicles))
                    rhs = len(selected_tour) - 1
                    model.cbLazy(lhs <= rhs)

        model = gurobi_model("CapVehicleRouting-3Index")

        routing_vars_tup = tupledict(
            [(i, j, r), -1] for (i, j, r) in product(self.points, self.points, self.vehicles))
        routing_vars = model.addVars(routing_vars_tup, vtype=GRB.BINARY, name='routing')

        vehicle_traveling_cost = LinExpr(
            [self.vehicle_info[self.vehicle_classify[r]]['uc'] * self._get_distance(i, j) for i, j, r in
             routing_vars_tup], [routing_vars[i, j, r] for i, j, r in routing_vars_tup])
        model.setObjective(vehicle_traveling_cost)

        # avoid self loop
        model.addConstrs(
            routing_vars[i, i, r] == 0 for r in self.vehicles for i in self.points)
        # symmetry breaking
        model.addConstrs(
            routing_vars.sum('*', '*', r) >= routing_vars.sum('*', '*', r + 1) for r in self.vehicles[:-1])
        # visit all the customers
        # model.addConstrs(
        #     routing_vars.sum('*', j, '*') == 1 for j in self.customers)
        # model.addConstrs(
        #     routing_vars.sum(j, '*', '*') == 1 for j in self.customers)
        # each vehicle can only be used at most once
        model.addConstrs(
            routing_vars.sum(self.i, '*', r) <= 1 for r in self.vehicles)
        # the in degree equals to out degree
        model.addConstrs(
            routing_vars.sum('*', j, r) == routing_vars.sum(j, '*', r) for r in self.vehicles for j in self.points)
        # vehicle capacity
        model.addConstrs(
            quicksum(self.depots_customers[j] * routing_vars[i, j, r] for i in self.points for j in self.customers)
            <= self.vehicle_info[self.vehicle_classify[r]]['cap'] for r in self.vehicles)

        # Optimize model
        model._vars = routing_vars
        model.Params.lazyConstraints = 1
        model.optimize(subtour_elimination)
        print_model_nonzeros(model, 'routing')

        # if model.getAttr("Status") == GRB.OPTIMAL:
        #     print('')
        #     var_values = model.getAttr('x', routing_vars)
        #     for v in self.vehicles:
        #         selected = tuplelist((i, j) for i, j, r in var_values.keys() if var_values[i, j, v] > 0.5 and r == v)
        #         tour = self.subtour(selected)
        #         print('Optimal tour ({}): {}'.format(v, tour))
        #     print('')

        return model


def run_BP():
    items = {0: 10, 1: 10, 2: 10}
    bin_capacity = 20
    BP = BinPacking(items, bin_capacity)
    bin_number = BP.mip_solve()
    print(bin_number)


def run_CVRP(n):
    # instance_name = "03_20_02_0_0"
    # INST = Instance(instance_name)
    # depots_customers = {
    #     1: 0,
    #     3: 10.0, 4: 72.0, 5: 1.0, 6: 24.0, 7: 31.0,
    #     8: 4.0, 9: 61.0, 10: 56.0, 11: 1.0, 12: 21.0
    # }
    # vehicle_preparation = {
    #     0: {1: 0, 2: 1},
    #     1: {1: 0, 2: 1}
    # }
    # i, s = 1, 0

    instance_name = "07_40_02_0_0"
    INST = Instance(instance_name)
    depots_customers = {
        2: 0,
        7: 10.0, 8: 72.0, 9: 1.0, 10: 24.0,
        11: 31.0, 12: 4.0,
        # 13: 61.0, 15: 1.0, 16: 21.0, 17: 51.0, 18: 31.0,
        # 22: 6.0, 24: 107.0, 25: 49.0, 26: 71.0, 27: 10.0,
        28: 71.0, 29: 6.0, 30: 11.0, 31: 63.0, 32: 85.0, 33: 28.0,
        # 34: 11.0, 35: 26.0, 36: 14.0, 37: 11.0, 38: 31.0,
        # 39: 9.0, 40: 23.0, 41: 28.0, 42: 2.0, 43: 28.0
    }
    depots_customers = {key: value + random.randint(1, 3) for key, value in depots_customers.items()}
    depots_customers[2] = 0

    vehicle_preparation = {  # i, s, h
        0: {1: 5, 2: 0},
        1: {1: 5, 2: 0}
    }
    i, s = 2, 0

    CVRP = CapVehicleRouting(
        depots_customers, vehicle_preparation,
        INST.vehicle_info, INST.vehicle_types, INST.special_vehicle_types, INST.is_isolated,
        INST._get_distance, i, s)
    VRP = CVRP.single_commodity_flow_model()
    # VRP = CVRP.three_index_model_with_callback()


if __name__ == "__main__":
    # run_BP()
    run_CVRP(1)
