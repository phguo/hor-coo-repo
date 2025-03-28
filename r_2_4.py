# coding:utf-8

import json
import os
import random
import time
import uuid
from datetime import datetime
from itertools import product

import seaborn as sns
import matplotlib.pyplot as plt
import pandas as pd
from matplotlib.collections import PolyCollection
from matplotlib.ticker import ScalarFormatter

custom_params = {
    "axes.spines.right": False,
    "axes.spines.top": False,
    # "text.usetex": True,
    # "font.family": "serif",
    "font.family": ['Arial', 'SimHei'],
}
plt.rcParams.update(custom_params)

# sns.set_style(None, {'font.family':['Arial', 'SimHei']})

from config import PARAMETERS, SERVER_NAME, FILE_DIR, LANG_ZH
from r_2_6 import get_stat
from r_LBBD import r_LogicBasedBendersDecomposition
from run import bark as notify

BASE_INSTANCE = "03_10_04_0_1"
SCENARIO_NUM_LI = [4, 8, 16, 32]
EACH_SCENARIO_NUM_RUN = 10
RANDOM_SEED = 1

METHOD_LI = [
    "BAC",  # BAC++
    "BACnvw",  # BAC
    "LBBD",  # LBBD++
    "LBBDnvw",  # LBBD
    "BACnw",  # BAC+
    "LBBDnw"  # LBBD+
]

# Ensure generated instances are the same
random.seed(RANDOM_SEED)


def save_json(file_name, json_data):
    with open(file_name, 'w') as f:
        f.write(json.dumps(json_data, sort_keys=True, indent=4))


def load_json(file_name):
    with open(file_name, 'r') as f:
        lines = f.read()
        data = json.loads(
            lines, object_hook=lambda d: {int(k) if k.lstrip('-').isdigit() else k: v for k, v in d.items()})
    return data


def generate_increased_s_instances(instance, scenario_num_li, each_scenario_num_run):
    for scenario_num in scenario_num_li:
        for i in range(each_scenario_num_run):
            parameters = PARAMETERS.copy()
            LBBD = r_LogicBasedBendersDecomposition(instance, parameters, 'LBBD')
            LBBD.resample(scenario_num, correlated_demand=False)
            json_data = LBBD.resampled_data
            file_name = f"./r_revision/r.2.4/instances/{instance}_{scenario_num}_{i}.json"
            save_json(file_name, json_data)


def run_exp_increased_s(instance, scenario_num_li, each_scenario_num_run):
    # Send start notification
    start_time = time.perf_counter()
    start_content = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    u = uuid.uuid1()
    start_content += f"\n\n{BASE_INSTANCE}, {SCENARIO_NUM_LI}, {EACH_SCENARIO_NUM_RUN}, {METHOD_LI}, {RANDOM_SEED}"
    start_content += f"\n\n{u}"
    start_title = f"R.2.4 [START] on [{SERVER_NAME}]"
    notify(start_title, start_content)

    def exp(instance, scenario_num, i, method):
        title = f"{instance}, {scenario_num}, {i}, {method}"
        parameters = PARAMETERS.copy()
        file_name = f"./r_revision/r.2.4/instances/{instance}_{scenario_num}_{i}.json"

        vrp_solver = False if 'nvw' in method else True
        warm_start = False if 'nw' in method or 'nvw' in method else True
        s_method = "LBBD" if 'LBBD' in method else "BAC"

        # Solve instance
        LBBD = r_LogicBasedBendersDecomposition(
            file_name, parameters, s_method, vrp_solver=vrp_solver, warm_start=warm_start)
        stat, sol = LBBD.benders_solve() if s_method == "LBBD" else LBBD.branch_and_check_solve()

        # Save solution
        app = '' if method == 'BAC' else f"_{method}"
        sol_file = f"./r_revision/r.2.4/solutions/{instance}_{scenario_num}_{i}{app}"
        LBBD.save(sol_file, stat, sol)

        # Retrieve solution
        obj, gap, _, time, _ = get_stat(sol_file + '.json')
        content = f"obj={obj}, gap={gap:.2f}%, time={time:.2f}s"
        content += f"\n\n{u}"
        notify(title, content)

    for method in METHOD_LI:
        for scenario_num, i in product(scenario_num_li, range(each_scenario_num_run)):
            try:
                exp(instance, scenario_num, i, method)
            except:
                notify(f"{instance}, S_num = {scenario_num} ({i}/{EACH_SCENARIO_NUM_RUN}), {method}", f"Error \n\n{u}")

    # Send end notification
    end_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    end_content = f"Total time: {round(time.perf_counter() - start_time, 2)}s \n" + end_time
    end_content += f"\n\n{u}"
    notify(f"R.2.4 [END] on [{SERVER_NAME}]", end_content)


def get_dataframe(instance, scenario_num_li, each_scenario_num_run):
    data = []
    for i, scenario_num, method in product(range(each_scenario_num_run), scenario_num_li, METHOD_LI):
        app = '' if method == 'BAC' else f"_{method}"
        file_name = f"./r_revision/r.2.4/solutions/{instance}_{scenario_num}_{i}{app}.json"
        if os.path.exists(file_name):
            obj, gap, _, time, _ = get_stat(file_name)
            data.append([i, str(scenario_num), method, time, obj / 100000])
    t = "Time (s)" if not LANG_ZH else "时间 (秒)"
    obj = "Total cost ($\\times 10^5$)" if not LANG_ZH else "目标函数值（10^5）"
    df = pd.DataFrame(data, columns=['i', '$|S|$', 'method', t, obj])
    return df


def draw_time_confidence_interval(original_df):
    fig, ax = plt.subplots(figsize=(4, 3))

    lineplot_styles = [
        dict(
            label='Mean (NLBBD-C)' if not LANG_ZH else '均值 (NLBBD-C)',
            errorbar=('ci', 95),
            estimator="mean",
            zorder=1, c='0.25', lw=1, ls=':'
        ),
        dict(
            label='Mean (NLBBD-B&C++)' if not LANG_ZH else '均值 (NLBBD-B&C++)',
            errorbar=('ci', 95),
            estimator="mean",
            zorder=1, c='0.25', lw=1, ls='--'
        )
    ]
    stripplot_styles = [
        dict(
            label='Instance (NLBBD-C)' if not LANG_ZH else '算例 (NLBBD-C)',
            zorder=20,
            native_scale=True,
            alpha=0.8,
            jitter=True,
            s=5, marker='X', facecolor='w', edgecolor='k', linewidth=0.4,
        ),
        dict(
            label='Instance (NLBBD-B&C++)' if not LANG_ZH else '算例 (NLBBD-B&C++)',
            zorder=20,
            native_scale=True,
            alpha=0.8,
            jitter=True,
            s=5, marker='o', facecolor='w', edgecolor='k', linewidth=0.4,
        )
    ]

    for j, method in enumerate(['LBBDnvw', 'BAC']):
        df = original_df.loc[original_df['method'] == method]
        y = "Time (s)" if not LANG_ZH else "时间 (秒)"
        sns.lineplot(ax=ax, data=df, x='$|S|$', y=y, **lineplot_styles[j])
        sns.stripplot(ax=ax, data=df, x='$|S|$', y=y, **stripplot_styles[j])
        # sns.boxplot(
        #     ax=ax, data=df, x='$|S|$', y=y, width=0.45,
        #     boxprops={'facecolor': 'w', 'edgecolor': 'k', 'alpha': 1},
        #     linewidth=1, flierprops={'marker': 'x'})

    ax.set_yscale("log", base=2)
    formatter = ScalarFormatter()
    formatter.set_scientific(False)
    ax.yaxis.set_major_formatter(formatter)

    for child in ax.findobj(PolyCollection):
        child.set_color('#a5acaf')
        child.set_edgecolor('none')
        child.set_alpha(0.25)
        child.set_label('95% CI')

    handles, labels = ax.get_legend_handles_labels()
    han = handles[:3]
    han.insert(1, handles[6])
    han.append(handles[8])
    lab = labels[:3]
    lab.insert(1, labels[6])
    lab.append(labels[8])
    ax.legend(han, lab, edgecolor='none', framealpha=0.5, ncol=1, fontsize=7, loc="upper left")

    ax.grid(alpha=0.3, ls='--', axis='x', zorder=0)
    ax.set_axisbelow(True)
    ax.set_xlim(-0.3, 3.3)
    ax.set_ylim(2 ** 4, 2 ** 12)
    ax.set_yticks([2 ** i for i in range(4, 13, 2)])
    # ax.set_yticklabels([f"$2^{{{i}}}$" for i in range(4, 13, 2)])
    # ax.ticklabel_format(style='sci', scilimits=(-1, 2), axis='y')

    file_dir = FILE_DIR['figures']
    plt.savefig(file_dir + "r_increased_s_time.pdf", bbox_inches='tight', transparent=True, pad_inches=0.05)


def draw_obj_confidence_interval(original_df):
    fig, ax = plt.subplots(figsize=(4, 3))

    lineplot_style = dict(
        label='Mean' if not LANG_ZH else '均值',
        errorbar=('ci', 95),
        estimator="mean",
        zorder=1, c='0.25', lw=1, ls='--'
    )
    stripplot_style = dict(
        label='Instance' if not LANG_ZH else '算例',
        zorder=20,
        native_scale=True,
        alpha=0.8,
        jitter=True,
        s=5, marker='o', facecolor='w', edgecolor='k', linewidth=0.4,
    )

    df = original_df.loc[original_df['method'] == "BAC"]
    y = "Total cost ($\\times 10^5$)" if not LANG_ZH else "目标函数值（10^5）"
    sns.lineplot(ax=ax, data=df, x='$|S|$', y=y, **lineplot_style)
    sns.stripplot(ax=ax, data=df, x='$|S|$', y=y, **stripplot_style)

    for child in ax.findobj(PolyCollection):
        child.set_color('#a5acaf')
        child.set_edgecolor('none')
        child.set_alpha(0.25)
        child.set_label('95% CI')

    handles, labels = ax.get_legend_handles_labels()
    ax.legend(handles[:3], labels[:3], edgecolor='none', framealpha=0.5, ncol=1, fontsize=7, loc="upper right")
    ax.grid(alpha=0.3, ls='--', axis='x', zorder=0)
    ax.set_xlim(-0.3, 3.3)
    # ax.set_ylim(4, 8)
    ax.set_yticks([4, 5, 6, 7, 8])

    file_dir = FILE_DIR['figures']
    plt.savefig(file_dir + "r_increased_s_obj.pdf", bbox_inches='tight', transparent=True, pad_inches=0.05)


def draw_increased_s(instance, scenario_num_li, each_scenario_num_run):
    original_df = get_dataframe(instance, scenario_num_li, each_scenario_num_run)
    draw_time_confidence_interval(original_df)
    draw_obj_confidence_interval(original_df)


if __name__ == '__main__':
    # generate_increased_s_instances(BASE_INSTANCE, SCENARIO_NUM_LI, EACH_SCENARIO_NUM_RUN)
    # run_exp_increased_s(BASE_INSTANCE, SCENARIO_NUM_LI, EACH_SCENARIO_NUM_RUN)

    draw_increased_s(BASE_INSTANCE, SCENARIO_NUM_LI, EACH_SCENARIO_NUM_RUN)
