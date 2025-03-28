# coding:utf-8

import json
import os
import time
from copy import deepcopy
from itertools import product
import requests

from keys import pushover_user, pushover_token, bark_key
from config import META_DATA_TEMPLATE, PARAMETERS, FILE_DIR, SEND_NOTIFICATION, SERVER_NAME, \
    INSTANCE_GENERATING_PARAMETERS, \
    ALL_INSTANCES_NAME, EXPERIMENTAL_INSTANCES_NAME
from instance import RandomDataGenerator, Instance
from io_operator import solved_instance_classify
from r_LBBD import r_LogicBasedBendersDecomposition

from solver import StochasticModels
from LBBD import LogicBasedBendersDecomposition


def shutdown():
    from keys import sudo_password
    cmd = "shutdown -h now"
    os.system("echo {}|sudo -S {}".format(sudo_password, cmd))


def bark(title, content, bark_key=bark_key):
    url = "https://api.day.app/{}/{}/{}".format(bark_key, title, content)
    if SEND_NOTIFICATION:
        try:
            return requests.get(url).content
        except:
            pass
    else:
        print(title, content)


def pushover(title, content, user=pushover_user, token=pushover_token):
    url = "https://api.pushover.net/1/messages.json"
    headers = {"Content-Type": "application/json; charset=utf-8"}
    json_data = json.dumps({
        "message": content,
        "device": "guoph-iPhone",
        "title": title,
        "token": token,
        "user": user
    })
    if SEND_NOTIFICATION:
        try:
            return requests.post(url=url, headers=headers, data=json_data).content
        except:
            pass
    else:
        print(title, content)


def send(make_and_solve):
    def wrapper(instance_name, parameters, instance_type, solution_method='mip', **exp_set):
        try:
            t = time.perf_counter()
            title = "✅ {} | {} | {} Solved in (s) ".format(instance_name, instance_type, exp_set)
            make_and_solve(instance_name, parameters, instance_type, solution_method, **exp_set)
            content = str(time.perf_counter() - t)
            pushover(title, content)
        except Exception as content:
            title = "❌ {} | {} | {} Error".format(instance_name, instance_type, exp_set)
            pushover(title, content)

    return wrapper


def clean():
    for file_type, directory in FILE_DIR.items():
        if file_type != 'figures':
            for file in os.listdir(directory):
                if file != ".gitkeep" and file != "_gsdata_":
                    os.remove(directory + file)


def generate_instances(write_to_file):
    # ----- BELLOWING GENERATE RANDOM DATA -----
    # generate random data according to 'meta_data' and save to './instances/
    # meta_data, random_seed --> (RandomDataGenerator) --> './instances/{instance_name}.json'
    instance_name_list = []

    facility_num_li = INSTANCE_GENERATING_PARAMETERS['facility_num_li']
    demand_num_li = INSTANCE_GENERATING_PARAMETERS['demand_num_li']
    scenario_num_li = INSTANCE_GENERATING_PARAMETERS['scenario_num_li']
    is_line_center_li = INSTANCE_GENERATING_PARAMETERS['is_line_center_li']

    for facility_num, demand_num, scenario_num, line_center in product(facility_num_li, demand_num_li,
                                                                       scenario_num_li, is_line_center_li):
        if demand_num >= INSTANCE_GENERATING_PARAMETERS['mind_demand_point_of_clustered']:
            is_clustered_li = INSTANCE_GENERATING_PARAMETERS['is_clustered_li']
        else:
            is_clustered_li = [INSTANCE_GENERATING_PARAMETERS['is_clustered_li'][0]]
        for is_clustered_instance in is_clustered_li:
            meta_data = deepcopy(META_DATA_TEMPLATE)
            meta_data['facility_num'] = facility_num
            meta_data['demand_num'] = demand_num
            meta_data['scenario_num'] = scenario_num
            meta_data['is_line_disaster_center'] = line_center
            meta_data['is_clustered_instance'] = is_clustered_instance
            if write_to_file:
                try:
                    RandomDataGenerator(meta_data)
                except Exception as content:
                    title = "Instance Generation Error"
                    pushover(title, content)
            else:
                pass
            instance_name = PARAMETERS['instance_name_format'].format(facility_num, demand_num, scenario_num,
                                                                      is_clustered_instance, line_center)
            instance_name_list.append(instance_name)
    return instance_name_list


def check_instance_location():
    facility_num_li = INSTANCE_GENERATING_PARAMETERS['facility_num_li']
    demand_num_li = INSTANCE_GENERATING_PARAMETERS['demand_num_li']
    scenario_num_li = INSTANCE_GENERATING_PARAMETERS['scenario_num_li']
    is_line_center_li = INSTANCE_GENERATING_PARAMETERS['is_line_center_li']

    for facility_num, demand_num in product(facility_num_li, demand_num_li):
        if demand_num >= INSTANCE_GENERATING_PARAMETERS['mind_demand_point_of_clustered']:
            is_clustered_li = INSTANCE_GENERATING_PARAMETERS['is_clustered_li']
        else:
            is_clustered_li = [INSTANCE_GENERATING_PARAMETERS['is_clustered_li'][0]]
        for is_clustered_instance in is_clustered_li:
            # instances with same |T|, |D|, and c. (|T|/|D|/*/c/*)
            pre_points_loc = None
            pre_points_population = None
            for scenario_num, line_center in product(scenario_num_li, is_line_center_li):
                instance_name_format = PARAMETERS['instance_name_format']
                instance_name = instance_name_format.format(facility_num, demand_num, scenario_num,
                                                            is_clustered_instance, line_center)
                instance = Instance(instance_name)
                points_loc = instance.points_loc
                points_population = instance.points_population
                if pre_points_loc:
                    assert points_loc == pre_points_loc
                if pre_points_population:
                    assert points_population == pre_points_population
                pre_points_loc = deepcopy(points_loc)
                pre_points_population = deepcopy(points_population)


@send
def make_model_and_solve(instance_name, parameters, instance_type, solution_method="mip", **exp_set):
    if solution_method == "mip":
        s_model = StochasticModels(instance_name, parameters, instance_type, **exp_set)
        s_model = s_model.solve()
        # s_model.computeIIS()
        # s_model.write("model.ilp")
    elif solution_method == "benders":
        lbbd = LogicBasedBendersDecomposition(instance_name, parameters, instance_type, **exp_set)
        lbbd.experiment()


def solve_deterministic_equivalents():
    # --------  BELLOWING OPTIMIZE MODEL -------
    # read data from './instances/' and generate Instance according to 'instance_name'
    # instance_name --> (Instance)

    # generate StochasticModels according to 'instance_name' and 'parameters'
    # instance_name, parameters --> (StochasticModels)

    # use StochasticModels.solve() solve problem and save solution to './solutions/'
    # (StochasticModels.solve()) --> './solutions/{instance_name}.json', './solutions/{instance_name}.sol'

    # initializing solutions
    instance_type = "DE"
    instances_to_solve = [a[0] for a in solved_instance_classify(instance_type)[2]]
    for instance_name in instances_to_solve:
        make_model_and_solve(instance_name, PARAMETERS, instance_type)


def solve_deterministic_equivalents_without_valid_inequality():
    instance_type = "DE_NO_VI"
    instances_to_solve = [a[0] for a in solved_instance_classify(instance_type)[2]]
    for instance_name in instances_to_solve:
        make_model_and_solve(instance_name, PARAMETERS, instance_type)


def logic_based_Benders(instance_type):
    for instance_name in EXPERIMENTAL_INSTANCES_NAME:
        try:
            LBBD = r_LogicBasedBendersDecomposition(instance_name, PARAMETERS, instance_type)
            if instance_type == "BAC":
                stat = LBBD.branch_and_check_solve()[0]
            elif instance_type == "LBBD":
                stat = LBBD.benders_solve()[0]
            else:
                raise Exception("Unknown instance type: {}".format(instance_type))
            try:
                title = "{}:{} on {}".format(
                    instance_name, instance_type, SERVER_NAME)
                content = "{}, {}, {}".format(
                    stat["SolutionInfo"]["ObjVal"],
                    stat["SolutionInfo"]["MIPGap"],
                    stat["SolutionInfo"]["Runtime"]
                )
                # pushover(title, content)
                bark(title, content)

            except:
                # pushover(instance_name, "failed to retrieve SolutionInfo.")
                bark(instance_name, "failed to retrieve SolutionInfo.")

        except:
            # pushover(instance_name, "is not solved.")
            bark(instance_name, "is not solved.")


def solve_with_logic_based_benders_decomposition():
    logic_based_Benders("LBBD")


def solve_with_branch_and_check():
    logic_based_Benders("BAC")


if __name__ == '__main__':
    start_t = time.perf_counter()

    try:
        # clean()
        # generate_instances(True)
        # solve_deterministic_equivalents()
        # check_instance_location()
        # solve_deterministic_equivalents_without_valid_inequality()

        solve_with_branch_and_check()

        # solve_with_logic_based_benders_decomposition()



    except Exception as content:
        title = "❌ Main Loop Error"
        pushover(title, content)

    finally:
        content = str(time.perf_counter() - start_t)
        pushover("{} Terminated".format(SERVER_NAME), content)

        # shutdown()
