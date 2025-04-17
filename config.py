# coding:utf-8

import os

from gurobipy.gurobipy import GRB

RANDOM_SEED = 9

SERVER_NAME = 'localhost'
SEND_NOTIFICATION = True
COPY_TO_CLIPBOARD = False

ALL_INSTANCES_NAME = sorted([a.replace('.json', '') for a in os.listdir("instances") if '.json' in a])
ALL_INSTANCES_NAME = sorted(ALL_INSTANCES_NAME, key=lambda a: a[3:5])

EXPERIMENTAL_INSTANCES_NAME = [
    '03_10_02_0_0',
    '03_10_02_0_1',
    '03_10_04_0_0',
    '03_10_04_0_1',
    '05_10_02_0_0',
    '05_10_02_0_1',
    '05_10_04_0_0',
    '05_10_04_0_1',
    '07_10_02_0_0',
    '07_10_02_0_1',
    '07_10_04_0_0',
    '07_10_04_0_1',

    '03_20_02_0_0',
    '03_20_02_0_1',
    '03_20_04_0_0',
    '03_20_04_0_1',
    '05_20_02_0_0',
    '05_20_02_0_1',
    '05_20_04_0_0',
    '05_20_04_0_1',
    '07_20_02_0_0',
    '07_20_02_0_1',
    '07_20_04_0_0',
    '07_20_04_0_1',

    '03_40_02_0_0',
    '03_40_02_0_1',
    '03_40_02_1_0',
    '03_40_02_1_1',
    '03_40_04_0_0',
    '03_40_04_0_1',
    '03_40_04_1_0',
    '03_40_04_1_1',

    '05_40_02_0_0',
    '05_40_02_0_1',
    '05_40_02_1_0',
    '05_40_02_1_1',
    '05_40_04_0_0',
    '05_40_04_0_1',
    '05_40_04_1_0',
    '05_40_04_1_1',

    '07_40_02_0_0',
    '07_40_02_0_1',
    '07_40_02_1_0',
    '07_40_02_1_1',
    '07_40_04_0_0',
    '07_40_04_0_1',
    '07_40_04_1_0',
    '07_40_04_1_1'
]

DRAW_INSTANCE_NAMES = [
    "03_40_04_1_0", "03_40_04_0_1",
]
DRAW_SCENARIO_ID = 1

TOL = 1e-4  # FeasibilityTol
LBBD_TOLERANCE = TOL

VEHICLE_CAPACITY = 300
INSTANCE_GENERATING_PARAMETERS = {
    'facility_num_li': [3, 5, 7],
    'demand_num_li': [10, 20, 40],
    'scenario_num_li': [2, 4],
    'is_clustered_li': [0, 1],
    'is_line_center_li': [0, 1],
    'mind_demand_point_of_clustered': 40
}
META_DATA_TEMPLATE = {
    'facility_num': None,
    'demand_num': None,
    'scenario_num': None,
    'is_clustered_instance': None,
    'is_line_disaster_center': None,

    'square': (0, 1000),
    'cluster_num': 4,
    'cluster_radius': 150,

    'line_disaster_center': 500,
    'ring_disaster_center': (500, 500),

    'population_range': (50, 300),
    'severity_score_range': ((0.15, 0.2), (0.3, 1), (0.2, 0.25), (0.25, 0.3)),
    'triangular_severity_score_range': ((0.0, 0.4, 0.2), (0.6, 1.0, 0.8), (0.2, 0.6, 0.4), (0.4, 0.8, 0.6)),
    'vulnerability_score': (0.1, 1, 0.25, 0.5),

    'step_function': (((0, 50), 0.8),
                      ((50, 200), 0.5),
                      ((200,), 0.20)),

    # cap - capacity, fc - fixed cost, uc - unit cost
    'facility_types_info': {
        1: {'cap': 4000, 'fc': 7000, 'uc': 0},
        2: {'cap': 1000, 'fc': 2500, 'uc': 0}
    },
    # type 1 : (num, capacity, first-stage preparing cost, second-stage preparing cost, unit traveling cost)
    'vehicle_capacity': VEHICLE_CAPACITY,
    'general_vehicle_info': {
        1: {'num': 40, 'cap': VEHICLE_CAPACITY, 'fc': 0, 'sc': 0, 'uc': 40}
    },
    'special_vehicle_info': {
        2: {'num': 40, 'cap': VEHICLE_CAPACITY, 'fc': 0, 'sc': 0, 'uc': 400}
    },
    # cost for acquire relief items
    'unit_facility_acquisition_cost': 100,

    # for back-and-forth approximation
    'unit_direct_trans_cost': None,
    'unit_lateral_trans_cost': None,
}

PARAMETERS = {
    'instance_seed': RANDOM_SEED,
    'location_seed': RANDOM_SEED,
    'population_seed': RANDOM_SEED,
    'demand_quantity_seed': RANDOM_SEED,
    'isolating_seed': RANDOM_SEED,

    'big_m': 9999 * 100,
    'use_local_big_m': True,
    'epsilon': 0.0001,
    'direct_trans_first': False,
    'max_linearized_by_gurobi': True,
    'service_level_constraint': True,
    'service_level': 1,
    'back_and_forth_approximation': False,
    'vehicle_routing_sub_problem': True,
    'gurobi_infeasible_code': (GRB.INFEASIBLE, GRB.INF_OR_UNBD, GRB.UNBOUNDED, GRB.CUTOFF),
    'gurobi_optimal': {GRB.OPTIMAL},
    'instance_name_format': '{:0>2d}_{:0>2d}_{:0>2d}_{:0>1d}_{:0>1d}',
    # 'use_cached_model': True,
    # 'integer_transportation_decision': False,
    # 'valid_inequality': False,  # valid inequality provide stronger LP relaxation

    'lbbd_use_valid_inequality': True,
    'lbbd_precision': 4,
    'lbbd_log_to_console': True,
    'lbbd_time_lim': 3600,
    'lbbd_gap': LBBD_TOLERANCE,  # (feasible_UB - LP_LB) / feasible_UB
    'lbbd_mip_gap': 0,  # (relaxed_PMP_UB - LP_LB) / relaxed_PMP_UB

    # 'multi_precessing_vdsp': True,
    # 'thread': 2,

    # 'vdsp_gap_lim': 0.1,
    # 'vdsp_time_lim': 300,
    # 'demand_lower_lim': 20,

    'vdsp_gap_lim': 0.1,
    'vdsp_time_lim': 150,
    'demand_lower_lim': 20,
}

GUROBI_PARAMETERS = {
    # 'TimeLimit': 3600,

    # 'Seed': RANDOM_SEED,
    # 'MIPGap': 1 / 100,
    # 'NodefileStart': 2.5,  # in GB
    # 'Threads': 1,
    # 'InputFile': './tune/tune0.prm',
    'LogToConsole': False,
    # 'PreQLinearize': -1,
    # 0 leaves Q matrices unmodified, 1 for a strong LP relaxation, 2 for a compact relaxation.
    # 'MIQCPMethod': -1,
    # 1 uses a linearized, outer-approximation approach, 0 solves continuous QCP relaxations at each node.
    # 'MIPFocus': 3
    # 0 balance, 1 find feasible, 3 prove optimality
    # 'LazyConstraints': 1
}

LANG_ZH = True

FILE_DIR = {
    'instances': './instances/',
    'de_solutions': './de_solutions/',
    'de_solutions_no_vi': './de_solutions_no_vi/',

    'LBBD_solutions': './LBBD_solutions/',
    'BAC_solutions': './BAC_solutions/',

    'figures': './figures/' if not LANG_ZH else './figures_zh/',

    'exp_pareto': './experimental_results/pareto/',
    'exp_stochasticity': './experimental_results/stochasticity/',
    'exp_stc_aux_instances': './experimental_results/stochasticity/aux_instances/',
    'exp_stc_aux_solutions': './experimental_results/stochasticity/aux_solutions/',
    'exp_vehicle_routing': './experimental_results/vehicle_routing/',
    'exp_lateral_trans': './experimental_results/lateral_trans/'
}

EXP_PARAMETERS = {
    'originally_optimized': [a for a in ALL_INSTANCES_NAME if a[3:5] in ['10']],

    # exp.pareto
    'exp_pareto_list': [i / 10 for i in range(1, 11)],

    # exp.lateral_trans
    # 'exp_lateral_trans_list': [1.0, 1.5, 2.0, 4.0],  # for changing acquiring cost
    'exp_lateral_trans_facility_cap_times': [1, 1.2, 2],

    # exp.vehicle_routing
    'exp_unit_transportation_cost': 6.5874,
    'exp_routing_cost_times': [0.0, 0.25, 0.5, 1.0, 2.0],
    'exp_facility_cost_times': [1],
    'exp_vehicle_capacity_times': [1.0],
    'baseline_vehicle_capacity_times': 1.0,

    # exp.stochastic
    'uncertainty_level': [1.0, 2.0, 3.0, 4.0, 5.0],
    'baseline_uncertainty_level': 1.0,
    'uct_types': [
        ('triangular', [(0.0, 0.4, 0.2), (0.6, 1.0, 0.8), (0.2, 0.6, 0.4), (0.4, 0.8, 0.6)]),
        ('uniform', [(0.00, 0.25), (0.75, 1.00), (0.25, 0.50), (0.50, 0.75)]),
        ('uniform', [(0.15, 0.2), (0.3, 1.0), (0.2, 0.25), (0.25, 0.3)]),
        ('uniform', [(0.0, 1.0), (0.0, 1.0), (0.0, 1.0), (0.0, 1.0)])
    ]
}

if __name__ == '__main__':
    print(ALL_INSTANCES_NAME)
