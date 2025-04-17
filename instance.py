import json
import math
import random
from itertools import combinations, product

from config import META_DATA_TEMPLATE, PARAMETERS, EXPERIMENTAL_INSTANCES_NAME
from io_operator import write_instance, read_instance

random.seed(PARAMETERS['instance_seed'])


class RandomDataGenerator(object):

    def __init__(self, meta_data):

        self.facility_num = meta_data['facility_num']
        self.demand_num = meta_data['demand_num']
        self.scenario_num = meta_data['scenario_num']
        self.is_clustered_instance = meta_data['is_clustered_instance']
        self.is_line_disaster_center = meta_data['is_line_disaster_center']

        instance_name_format = PARAMETERS['instance_name_format']
        self.instance_name = instance_name_format.format(self.facility_num, self.demand_num, self.scenario_num,
                                                         self.is_clustered_instance, self.is_line_disaster_center)

        self.square = meta_data['square']
        self.cluster_num = meta_data['cluster_num']
        self.cluster_radius = meta_data['cluster_radius']
        self.population_range = meta_data['population_range']
        self.severity_score_range = meta_data['severity_score_range']
        self.triangular_severity_score_range = meta_data['triangular_severity_score_range']
        self.vulnerability_score = meta_data['vulnerability_score']
        self.ring_disaster_center = meta_data['ring_disaster_center']
        self.line_disaster_center = meta_data['line_disaster_center']
        self.step_function = meta_data['step_function']

        self.vehicle_capacity = meta_data['vehicle_capacity']
        self.general_vehicle_info = meta_data['general_vehicle_info']
        self.special_vehicle_info = meta_data['special_vehicle_info']
        self.facility_types_info = meta_data['facility_types_info']

        self.unit_direct_trans_cost = meta_data['unit_direct_trans_cost']
        self.unit_lateral_trans_cost = meta_data['unit_lateral_trans_cost']

        self.data = meta_data
        self.data.update(self._generate_data())
        # del self.data['square']

        # write_instance(self.data, self.instance_name)

    def _generate_data(self):
        facility_set = [i for i in range(self.facility_num)]
        demand_set = [i for i in range(facility_set[-1] + 1, self.demand_num + self.facility_num)]
        points_set = facility_set + demand_set

        points_loc, clusters, clusters_center = self.rand_points_loc(points_set)
        points_population = self.rand_population(demand_set)
        points_demand = self.rand_points_demand_baseline(demand_set, points_population)
        points_isolating = self.rand_points_isolating(demand_set, points_loc)
        scenarios = {s: {j: (points_demand[s][j], points_isolating[s][j]) for j in demand_set}
                     for s in range(self.scenario_num)}

        facility_types = sorted([k for k in self.facility_types_info])
        facility_cost_types = {k: self.facility_types_info[k]['fc'] +
                                  self.facility_types_info[k]['cap'] * self.facility_types_info[k]['uc']
                               for k in facility_types}
        facility_cap_types = {k: self.facility_types_info[k]['cap'] for k in facility_types}
        facility_open_cost = {i: facility_cost_types for i in facility_set}

        scenario_prob = {s: 1 / self.scenario_num for s in scenarios}

        isolated_demand_in_scenario = {s: [j for j in scenarios[s] if scenarios[s][j][1] == 1] for s in scenarios}

        general_vehicle_type_num = 0
        special_vehicle_type_num = 0
        if self.general_vehicle_info:
            general_vehicle_type_num = max(set(self.general_vehicle_info))
        if self.special_vehicle_info:
            special_vehicle_type_num = max(set(self.special_vehicle_info)) - general_vehicle_type_num

        general_vehicle_types = list(key for key in self.general_vehicle_info)
        special_vehicle_types = list(key for key in self.special_vehicle_info)
        vehicle_types = general_vehicle_types + special_vehicle_types

        vehicle_info = {}
        vehicle_info.update(self.general_vehicle_info)
        vehicle_info.update(self.special_vehicle_info)

        data = locals()
        data.pop('self')
        # data.pop('points_demand')

        return data

    def rand_uniform_point_loc(self):
        x = random.randint(self.square[0], self.square[1])
        y = random.randint(self.square[0], self.square[1])
        return (x, y)

    def rand_cluster_center_loc(self):
        cluster_center_square = [self.cluster_radius, self.square[1] - self.cluster_radius]
        x = random.randint(cluster_center_square[0], cluster_center_square[1])
        y = random.randint(cluster_center_square[0], cluster_center_square[1])
        return (x, y)

    def rand_cluster_point_loc(self, cluster_center):
        o_x, o_y = cluster_center
        x_range = (o_x - self.cluster_radius, o_x + self.cluster_radius)
        y_range = (o_y - self.cluster_radius, o_y + self.cluster_radius)
        return (random.randint(x_range[0], x_range[1]), random.randint(y_range[0], y_range[1]))

    def rand_points_loc(self, points_set):
        random.seed(PARAMETERS['location_seed'])
        points_loc = dict()
        clusters = None
        clusters_center = None
        if self.is_clustered_instance:
            clusters = {c: dict() for c in range(self.cluster_num)}
            clusters_center = {c: self.rand_cluster_center_loc() for c in range(self.cluster_num)}
            for i in points_set:
                i_assign_to = random.randint(0, self.cluster_num - 1)
                i_loc = self.rand_cluster_point_loc(clusters_center[i_assign_to])
                clusters[i_assign_to][i] = i_loc
                points_loc[i] = i_loc
        else:
            points_loc = {i: self.rand_uniform_point_loc() for i in points_set}
        return points_loc, clusters, clusters_center

    def rand_population(self, demand_set):
        random.seed(PARAMETERS['population_seed'])
        population = {j: random.randint(self.population_range[0], self.population_range[1]) for j in demand_set}
        return population

    def rand_points_demand_baseline(self, demand_set, points_population, dst_type="uniform"):
        random.seed(PARAMETERS['demand_quantity_seed'])
        demand = dict()

        # NOTICE - DO NOT REMOVE
        tmp = [random.triangular(
            self.triangular_severity_score_range[s][0],
            self.triangular_severity_score_range[s][1],
            self.triangular_severity_score_range[s][2]
        ) for s in range(self.scenario_num)]

        if dst_type == "uniform":
            for s in range(self.scenario_num):
                demand[s] = {
                    i: math.ceil(
                        1 * points_population[i] *
                        random.uniform(
                            self.triangular_severity_score_range[s][0],
                            self.triangular_severity_score_range[s][1]
                        )) for i in demand_set}

        elif dst_type == "triangular":
            for s in range(self.scenario_num):
                demand[s] = {
                    i: math.ceil(
                        1 * points_population[i] *
                        random.triangular(
                            self.triangular_severity_score_range[s][0],
                            self.triangular_severity_score_range[s][1],
                            self.triangular_severity_score_range[s][2]
                        )) for i in demand_set}

        else:
            raise Exception("Unknown distribution type: {}".format(dst_type))

        return demand

    def isolating_step_func(self, distance):
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

    def rand_isolating(self, scenario_id, demand_point_loc):
        random.seed(PARAMETERS['isolating_seed'])
        point_x, point_y = demand_point_loc
        if self.is_line_disaster_center:
            center = self.line_disaster_center
            distance_to_center = abs(point_y - center)
        else:
            center_x, center_y = self.ring_disaster_center
            distance_to_center = math.sqrt((point_x - center_x) ** 2 + (point_y - center_y) ** 2)

        isolating_prob = self.isolating_step_func(distance_to_center) * self.vulnerability_score[scenario_id]
        is_isolated = 0
        if random.random() <= isolating_prob:
            is_isolated = 1

        return is_isolated

    def rand_points_isolating(self, demand_set, points_loc):
        points_isolating = dict()
        for s in range(self.scenario_num):
            points_isolating[s] = {i: self.rand_isolating(s, points_loc[i]) for i in demand_set}
        # print(self.instance_name,
        #       {s: ([points_isolating[s][k] for k in points_isolating[s]].count(1)) for s in points_isolating})
        return points_isolating


def load_json(file_name):
    with open(file_name, 'r') as f:
        lines = f.read()
        data = json.loads(
            lines, object_hook=lambda d: {int(k) if k.lstrip('-').isdigit() else k: v for k, v in d.items()})
    return data


class Instance(object):

    def __init__(self, instance_name, instance_type=None, **info):
        self.instance_name = instance_name
        self.instance_type = instance_type
        self.exp_set = info
        if '.json' not in self.instance_name:
            self.data = read_instance(self.instance_name, self.instance_type, **info)
        else:
            self.data = load_json(self.instance_name)

        self.vehicle_capacity = self.data['vehicle_capacity']

        self.general_vehicle_types = self.data['general_vehicle_types']
        self.special_vehicle_types = self.data['special_vehicle_types']
        self.vehicle_types = self.data['vehicle_types']
        self.general_vehicle_info = self.data['general_vehicle_info']
        self.special_vehicle_info = self.data['special_vehicle_info']
        self.vehicle_info = self.data['vehicle_info']

        self.facility_types_info = self.data['facility_types_info']
        self.facility_types = self.data['facility_types']
        self.facility_cap_types = self.data['facility_cap_types']
        self.facility_open_cost = self.data['facility_open_cost']
        self.unit_facility_acquisition_cost = self.data['unit_facility_acquisition_cost']

        # for back-and-forth approximation
        self.unit_direct_trans_cost = self.data['unit_direct_trans_cost']
        self.unit_lateral_trans_cost = self.data['unit_lateral_trans_cost']

        self.scenario_num = self.data['scenario_num']
        self.scenarios = self.data['scenarios']
        self.scenario_prob = self.data['scenario_prob']
        self.isolated_demand_in_scenario = self.data['isolated_demand_in_scenario']

        self.facility_num = self.data['facility_num']
        self.demand_num = self.data['demand_num']
        self.facility_set = self.data['facility_set']
        self.demand_set = self.data['demand_set']
        self.points_set = self.data['points_set']
        self.points_loc = self.data['points_loc']

        self.points_population = self.data['points_population']

        self.is_clustered_instance = self.data['is_clustered_instance']
        self.is_line_disaster_center = self.data['is_line_disaster_center']
        self.clusters = self.data['clusters']
        self.clusters_center = self.data['clusters_center']
        self.square = self.data['square']
        self.step_function = self.data['step_function']
        self.ring_disaster_center = self.data['ring_disaster_center']
        self.line_disaster_center = self.data['line_disaster_center']
        self.cluster_radius = self.data['cluster_radius']

        self.experiment_setup()

    def experiment_setup(self):
        self.allow_lateral_transshipment = True
        self.lateral_trans_var_ub = float('inf')
        self.fix_first_stage = False
        self.solve_routing = True
        self.expected_value_problem = None
        self.p_type = None

        if self.instance_type == "DE_NO_VI":
            self.valid_inequality = False

        elif self.instance_type == "DE":
            self.valid_inequality = True

        elif self.instance_type in ["LBBD", "BAC", None]:
            pass

        elif self.instance_type == "PA_DE":
            ratio = self.exp_set.get('ratio', None)
            assert ratio != None
            self.service_level = ratio

        elif self.instance_type == "LT":
            self.valid_inequality = False
            allow_lt = self.exp_set.get('allow_lt', None)
            assert allow_lt != None
            self.allow_lateral_transshipment = allow_lt
            self.lateral_trans_var_ub = 0 if not self.allow_lateral_transshipment else self.lateral_trans_var_ub

            facility_cap_times = self.exp_set.get('f_cap_times', None)
            assert facility_cap_times != None
            self.scenarios_total_demand = {
                s: sum(v[0] for k, v in value.items()) for s, value in self.scenarios.items()}
            maximum_demand = max(self.scenarios_total_demand.values())
            large_cap_base = math.ceil(maximum_demand / self.facility_num)
            small_cap_base = math.ceil(large_cap_base * 0.25)
            large_facility_cap = math.ceil(large_cap_base * facility_cap_times)
            small_facility_cap = math.ceil(small_cap_base * facility_cap_times)
            for key, value in self.facility_types_info.items():
                if key == 1:
                    self.facility_types_info[key] = {k: large_facility_cap if k == 'cap' else v for k, v in
                                                     value.items()}
                    self.facility_cap_types[key] = large_facility_cap
                elif key == 2:
                    self.facility_types_info[key] = {k: small_facility_cap if k == 'cap' else v for k, v in
                                                     value.items()}
                    self.facility_cap_types[key] = small_facility_cap
                else:
                    raise Exception("Check the facility type ({})".format(key))

        elif self.instance_type == "NO_VRP":
            self.problem_type = self.exp_set.get('p_type', None)
            unit_trans_cost = self.exp_set.get('ut_cost', None)
            uc_times = self.exp_set.get('ut_times', None)
            fc_times = self.exp_set.get('fc_times', None)
            vc_times = self.exp_set.get('vc_times', None)

            assert self.problem_type != None
            assert vc_times != None
            if self.problem_type != "AUX":
                assert unit_trans_cost != None
                assert uc_times != None
                assert fc_times != None
                for key, value in self.facility_types_info.items():
                    self.facility_types_info[key]['fc'] *= fc_times

            self.vehicle_capacity *= vc_times
            for h in self.vehicle_types:
                self.vehicle_info[h]['cap'] *= vc_times
                if h in self.special_vehicle_types:
                    self.special_vehicle_info[h]['cap'] *= vc_times
                if h in self.general_vehicle_info:
                    self.general_vehicle_info[h]['cap'] *= vc_times

            if self.problem_type == "AUX":
                self.back_and_forth_approximation = False
                self.vehicle_routing_sub_problem = True
                self.fix_first_stage = False
            elif self.problem_type == "ROUTE":
                self.back_and_forth_approximation = False
                self.vehicle_routing_sub_problem = True
                self.fix_first_stage = True
            elif self.problem_type == "TRANS":
                self.back_and_forth_approximation = True
                self.vehicle_routing_sub_problem = False
                self.fix_first_stage = False
                self.unit_direct_trans_cost = unit_trans_cost * uc_times
                self.unit_lateral_trans_cost = unit_trans_cost * uc_times
            else:
                raise Exception("Unknown problem_type (p_type): {}".format(self.problem_type))

        elif self.instance_type == "STC":
            self.p_type = self.exp_set.get('p_type', None)
            uct_level = self.exp_set.get('uct_level', None)
            assert self.p_type != None
            assert uct_level != None

            if self.p_type == "EEV":
                self.expected_value_problem = self.exp_set.get('ev_p', None)
                assert self.expected_value_problem != None
                if self.expected_value_problem:
                    self.scenarios = dict({0: {j: [
                        max(self.scenarios[s][j][0] for s in self.scenarios),
                        # ceil(sum(self.scenarios[s][j][0] for s in self.scenarios) / self.scenario_num),
                        max(self.scenarios[s][j][1] for s in self.scenarios)
                    ] for j in self.demand_set}})
                    self.scenario_num = 1
                    self.scenario_prob = dict({0: 1})

            elif self.p_type == "WS":
                scenario_id = self.exp_set.get('s_id', None)
                assert scenario_id != None
                self.scenario_num = 1
                self.scenario_prob = dict({scenario_id: 1})
                self.scenarios = dict({scenario_id: self.scenarios[scenario_id]})
            elif self.p_type != "AUX":
                raise Exception("Unknown p_type ({}) for STC instance".format(self.p_type))

        else:
            raise Exception("Unknown instance type: \'{}\'".format(self.instance_type))

    def _get_distance(self, point_a, point_b):
        coordinate_a = self.points_loc[point_a]
        coordinate_b = self.points_loc[point_b]
        distance = math.sqrt(
            (coordinate_a[0] - coordinate_b[0]) ** 2 +
            (coordinate_a[1] - coordinate_b[1]) ** 2
        )
        return distance

    def get_furthest_distance(self, points_list):
        distance = -1
        for point_a, point_b in combinations(points_list, 2):
            tmp_distance = self._get_distance(point_a, point_b)
            if tmp_distance > distance:
                distance = tmp_distance
        return distance

    def get_furthest_to(self, point, other_points_list):
        distance = -1
        for other_point in other_points_list:
            tmp_distance = self._get_distance(point, other_point)
            if tmp_distance > distance:
                distance = tmp_distance
        return distance

    def get_closest_to(self, point, other_points_list):
        distance = float('inf')
        for other_point in other_points_list:
            tmp_distance = self._get_distance(point, other_point)
            if tmp_distance < distance:
                distance = tmp_distance
        return distance

    def _get_route_distance(self, point_seq):
        distance = 0
        for i, point in enumerate(point_seq):
            current_point = point
            next_point = point_seq[i + 1]
            current_point_coordinate = self.points_loc[current_point]
            next_point_coordinate = self.points_loc[next_point]
            distance += math.sqrt(
                (current_point_coordinate[0] - next_point_coordinate[0]) ** 2 +
                (current_point_coordinate[1] - next_point_coordinate[1]) ** 2
            )
            if i == len(point_seq) - 2:
                break
        return distance

    def is_isolated(self, scenario_id, demand_id):
        assert demand_id in self.points_set, '{} is NOT a demand point or facility'.format(demand_id)
        isolated = False
        if demand_id in self.isolated_demand_in_scenario[scenario_id]:
            isolated = True
        return isolated

    def save_instance(self, instance_type, **info):
        write_instance(self.data, self.instance_name, instance_type, **info)


def generate_experimental_instance():
    for instance_name in EXPERIMENTAL_INSTANCES_NAME:
        point_nums = [int(a) for a in instance_name.split('_')]
        meta_data = {}
        meta_data.update(META_DATA_TEMPLATE)
        meta_data.update({'facility_num': point_nums[0], 'demand_num': point_nums[1], 'scenario_num': point_nums[2],
                          'is_clustered_instance': point_nums[3], 'is_line_disaster_center': point_nums[4]})

        D = RandomDataGenerator(meta_data)
        I = Instance(D.instance_name)


if __name__ == '__main__':
    # generate_experimental_instance()
    # instance = Instance(EXPERIMENTAL_INSTANCE_NAME)

    from run import generate_instances

    generate_instances(True)
