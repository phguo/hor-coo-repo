# coding:utf-8

import copy
import math
import time
import uuid
from datetime import datetime
from itertools import product

import seaborn as sns
import matplotlib.pyplot as plt
import pandas as pd
from matplotlib.collections import PolyCollection
from matplotlib.ticker import ScalarFormatter

from config import PARAMETERS, SERVER_NAME
from r_2_4 import save_json, load_json
from r_2_6 import get_stat
from r_LBBD import r_LogicBasedBendersDecomposition
from run import bark as notify

BASE_INSTANCE = "03_10_04_0_1"

SCENARIO_NUM_LI = [4, 8, 16, 32]
EACH_SCENARIO_NUM_RUN = 10
COEFFICIENT_LI = [0.25, 0.5, 1.0]


def generate_correlated_demand_instances(instance, scenario_num_li, each_scenario_num_run, coefficient_li):
    def step_func(lbbd, distance):
        assert distance >= 0
        range_a, range_a_value = lbbd.step_function[0][0], lbbd.step_function[0][1]
        range_b, range_b_value = lbbd.step_function[1][0], lbbd.step_function[1][1]
        other_value = lbbd.step_function[2][1]
        if distance >= range_a[0] and distance < range_a[1]:
            value = range_a_value
        elif distance >= range_b[0] and distance < range_b[1]:
            value = range_b_value
        else:
            value = other_value
        return value

    def distance_to_center(lbbd, point_loc):
        point_x, point_y = point_loc
        if lbbd.is_line_disaster_center:
            center = lbbd.line_disaster_center
            distance_to_center = abs(point_y - center)
        else:
            center_x, center_y = lbbd.ring_disaster_center
            distance_to_center = math.sqrt((point_x - center_x) ** 2 + (point_y - center_y) ** 2)
        return distance_to_center

    for scenario_num, i, coefficient in product(scenario_num_li, range(each_scenario_num_run), coefficient_li):
        original_file_name = f"./r_revision/r.2.4/instances/{instance}_{scenario_num}_{i}.json"
        parameters = PARAMETERS.copy()
        LBBD = r_LogicBasedBendersDecomposition(instance, parameters, 'LBBD')
        original_data = load_json(original_file_name)
        data = copy.deepcopy(original_data)
        for s, point_demand in data['points_demand'].items():
            for j, demand in point_demand.items():
                j_to_center = distance_to_center(LBBD, LBBD.points_loc[j])
                step_function_value = step_func(LBBD, j_to_center)
                new_demand = demand * coefficient * step_function_value
                point_demand[j] = new_demand
                data["scenarios"][s][j][0] = new_demand
        for s in data["scenarios"].keys():
            original_s_total_demand = sum(
                [original_data["scenarios"][s][j][0] for j in original_data["scenarios"][s].keys()])
            new_s_total_demand = sum(
                [data["scenarios"][s][j][0] for j in data["scenarios"][s].keys()])
            for j in data["demand_set"]:
                new_new_demand = data["scenarios"][s][j][0] * original_s_total_demand / new_s_total_demand
                new_new_demand = math.ceil(new_new_demand)
                min_vehicle_cap = min(v['cap'] for v in data["vehicle_info"].values())
                new_new_demand = min(new_new_demand, min_vehicle_cap)
                data["scenarios"][s][j][0] = new_new_demand
                data["points_demand"][s][j] = new_new_demand
        new_file_name = f"./r_revision/r.2.5/instances/{instance}_{scenario_num}_{coefficient}_{i}.json"
        save_json(new_file_name, data)


def run_exp_correlated_demand(instance, scenario_num_li, each_scenario_num_run, coefficient_li):
    # Send start notification
    start_time = time.perf_counter()
    start_content = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    u = uuid.uuid1()
    start_content += f"\n\n{instance}, {scenario_num_li}, {each_scenario_num_run}, {coefficient_li}"
    start_content += f"\n\n{u}"
    start_title = f"R.2.5 [START] on [{SERVER_NAME}]"
    notify(start_title, start_content)

    def exp(instance, scenario_num, coefficient, i):
        title = f"{instance}, {coefficient}, {scenario_num}, {i}"
        parameters = PARAMETERS.copy()
        file_name = f"./r_revision/r.2.5/instances/{instance}_{scenario_num}_{coefficient}_{i}.json"
        LBBD = r_LogicBasedBendersDecomposition(file_name, parameters, 'BAC', vrp_solver=True, warm_start=True)
        stat, sol = LBBD.branch_and_check_solve()
        sol_file = f"./r_revision/r.2.5/solutions/{instance}_{scenario_num}_{coefficient}_{i}"
        LBBD.save(sol_file, stat, sol)
        obj, gap, _, time, _ = get_stat(sol_file + '.json')
        content = f"obj={obj}, gap={gap:.2f}%, time={time:.2f}s"
        content += f"\n\n{u}"
        notify(title, content)

    for coefficient, scenario_num, i in product(coefficient_li, scenario_num_li, range(each_scenario_num_run)):
        try:
            exp(instance, scenario_num, coefficient, i)
        except:
            notify(f"{instance}, {coefficient}, {scenario_num}, {i}", f"Error \n\n{u}")

    # Send end notification
    end_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    end_content = f"Total time: {round(time.perf_counter() - start_time, 2)}s \n" + end_time
    end_content += f"\n\n{u}"
    notify(f"R.2.5 [END] on [{SERVER_NAME}]", end_content)


def get_dataframe(instance, scenario_num_li, each_scenario_num_run, coefficient_li):
    data = []
    for coefficient, scenario_num, i in product(coefficient_li, scenario_num_li, range(each_scenario_num_run)):
        file_name = f"./r_revision/r.2.5/solutions/{instance}_{scenario_num}_{coefficient}_{i}.json"
        obj, gap, _, time, _ = get_stat(file_name)
        data.append([coefficient, str(scenario_num), i, time, obj / 100000])
    df = pd.DataFrame(data, columns=['coefficient', '$|S|$', 'i', 'Time (s)', 'Total cost ($\\times 10^5$)'])
    return df


def draw_confidence_interval(original_df, fig_dir):
    fig, axs = plt.subplots(2, 3, figsize=(10, 6))

    lineplot_style = dict(
        label='Mean',
        errorbar=('ci', 95),
        estimator="mean",
        zorder=1, c='0.25', lw=1, ls='--'
    )
    stripplot_style = dict(
        zorder=20,
        native_scale=True,
        alpha=0.8,
        jitter=True,
        s=5, marker='o', facecolor='w', edgecolor='k', linewidth=0.4,
    )

    for row, y_name in enumerate(['time', 'obj']):
        for col, coefficient in enumerate(COEFFICIENT_LI):
            df = original_df.loc[original_df['coefficient'] == coefficient]
            ax = axs[row][col]

            y = 'Time (s)' if y_name == 'time' else 'Total cost ($\\times 10^5$)'
            sns.lineplot(ax=ax, data=df, x='$|S|$', y=y, **lineplot_style)
            sns.stripplot(ax=ax, data=df, x='$|S|$', y=y, **stripplot_style)

            st = {k: v for k, v in stripplot_style.items() if k not in ['native_scale', 'jitter']}
            ax.scatter([], [], label="Individual run", **st)
            if y_name == 'time':
                ax.set_yscale("log", base=2)
                formatter = ScalarFormatter()
                formatter.set_scientific(False)
                ax.yaxis.set_major_formatter(formatter)
            if y_name == 'obj':
                ax.set_ylim(2.8, 8.6)

            for child in ax.findobj(PolyCollection):
                child.set_color('#a5acaf')
                child.set_edgecolor('none')
                child.set_alpha(0.25)
                child.set_label('95\% CI')

            ax.set_title(f'$\\alpha={coefficient:.2f}$', fontsize=10)
            ax.legend(edgecolor='none', framealpha=0.8, ncol=1, fontsize=8, loc="lower right")
            ax.grid(alpha=0.3, ls='--', axis='x', zorder=0)
            # ax.grid(alpha=0.3, ls='--', axis='y', zorder=0)
            ax.set_axisbelow(True)

            if col != 0:
                ax.set_ylabel('')
            if row == 0:
                ax.set_xlabel('')
            if row == 1:
                ax.set_title('')
            if not (row == 1 and col == 2):
                ax.get_legend().remove()

    plt.savefig(fig_dir, bbox_inches='tight', transparent=True, pad_inches=0.05)


def draw_correlated(instance, scenario_num_li, each_scenario_num_run, coefficient_li):
    df = get_dataframe(instance, scenario_num_li, each_scenario_num_run, coefficient_li)
    draw_confidence_interval(df, fig_dir='./figures/r_2_5.pdf')


if __name__ == '__main__':
    # generate_correlated_demand_instances(BASE_INSTANCE, SCENARIO_NUM_LI, EACH_SCENARIO_NUM_RUN, COEFFICIENT_LI)
    # run_exp_correlated_demand(BASE_INSTANCE, SCENARIO_NUM_LI, EACH_SCENARIO_NUM_RUN, COEFFICIENT_LI)

    draw_correlated(BASE_INSTANCE, SCENARIO_NUM_LI, EACH_SCENARIO_NUM_RUN, COEFFICIENT_LI)
