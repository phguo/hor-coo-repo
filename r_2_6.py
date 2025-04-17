# coding:utf-8

import json

import pandas as pd

from config import EXPERIMENTAL_INSTANCES_NAME, PARAMETERS, SERVER_NAME
from r_LBBD import r_LogicBasedBendersDecomposition, WARM_START_TIME_LIM, MINUS_WARM_START_TIME
from run import bark as notify
from datetime import datetime
import time
import uuid

ENHANCE_CONTRIBUTE_INSTANCES = [
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
]


def get_stat(stat_file):
    with open(stat_file, 'r') as f:
        stat = json.load(f)
        obj = round(float(stat["SolutionInfo"]["ObjVal"]))
        gap = round(float(stat["SolutionInfo"]["MIPGap"]) * 100, 2)
        time = round(float(stat['SolutionInfo']['Runtime']), 2)

        # time = f"{time:.2f}" if time < 3600 else "Lmt."
        ws_time = stat.get("warm_start_time", None)
        # ws_time = f"{ws_time:.2f}" if ws_time is not None else None

        bks_time = None
        try:
            ub = float("inf")
            for i, v in stat["best_upper_bound_dic"].items():
                if v < ub:
                    ub = v
                    bks_time = stat["iteration_finish_time_dic"][i]
            # bks_time = round(bks_time, 2)
            # bks_time = f"{bks_time:.2f}" if bks_time < 3600 else "Lmt."
        except:
            pass

        return obj, gap, bks_time, time, ws_time


def enhance_impact():
    # Send start notification
    start_time = time.perf_counter()
    start_content = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    start_content += f"\nWARM_START_TIME_LMT: {WARM_START_TIME_LIM}; MINUS_WARM_START_TIME: {MINUS_WARM_START_TIME}"
    u = uuid.uuid1()
    start_content += f"\n\n{u}"
    start_title = f"R.2.6 Enhancement impact [START] on [{SERVER_NAME}]"
    notify(start_title, start_content)

    def exp(file_suffix, vrp_solver, warm_start):
        title = f"{instance}{file_suffix}"
        parameters = PARAMETERS.copy()
        LBBD = r_LogicBasedBendersDecomposition(instance, parameters, "BAC", vrp_solver, warm_start)
        stat, sol = LBBD.branch_and_check_solve()
        LBBD.save(f"./r_revision/r.2.6/enhance/{instance}{file_suffix}", stat, sol)
        obj, gap, bks_time, time, ws_time = get_stat(f"./r_revision/r.2.6/enhance/{instance}{file_suffix}.json")
        content = f"{obj}, {gap}%, {bks_time}s*, {time}s, {ws_time}s (ws)"

        # Compare with previous results
        bac_vw_stat_file = f"./r_revision/r.2.6/enhance/backup230519/{instance}-vw.json"
        obj_, gap_, bks_time_, time_, ws_time_ = get_stat(bac_vw_stat_file)
        content += f"\n{obj_}, {gap_}%, {time_}s"
        gap_diff = gap - gap_
        content += f"\nGap_diff.: {gap_diff:.2f}%"

        content += f"\n\n{u}"
        notify(title, content)

    for instance in ENHANCE_CONTRIBUTE_INSTANCES:
        # try:
        #     exp('-v', vrp_solver=True, warm_start=False)
        # except:
        #     notify(f"{instance}-v", "Error in vrp_solver=True, warm_start=False")
        try:
            exp('-vw', vrp_solver=True, warm_start=True)
        except:
            notify(f"{instance}-vw", "Error in vrp_solver=True, warm_start=True")

    # Send end notification
    end_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    end_content = f"Total time: {round(time.perf_counter() - start_time, 2)}s \n" + end_time
    end_content += f"\n\n{u}"
    notify(f"R.2.6 Enhancement impact [END] on [{SERVER_NAME}]", end_content)


def make_enhance_contribute_table():
    # Table R.2.6 in the response letter
    BAC_SOL_DIR = "./BAC_solutions/backup210815_8c16g"
    BAC_V_SOL_DIR = "./r_revision/r.2.6/enhance/backup230518"
    BAC_VW_SOL_DIR = "./r_revision/r.2.6/enhance/backup230520"

    for instance in ENHANCE_CONTRIBUTE_INSTANCES:
        bac_stat_file = f"{BAC_SOL_DIR}/{instance}.json"
        bac_v_stat_file = f"{BAC_V_SOL_DIR}/{instance}-v.json"
        bac_vw_stat_file = f"{BAC_VW_SOL_DIR}/{instance}-vw.json"

        line = instance.replace('_', '-')
        for stat_file in [bac_stat_file, bac_v_stat_file, bac_vw_stat_file]:
            obj, gap, bks_time, time, ws_time = get_stat(stat_file)
            if bks_time is None:
                line += f"\t{obj}\t{gap:.2f}\t{time}"
            else:
                line += f"\t{obj}\t{gap:.2f}\t{bks_time}\t{time}"
            if ws_time is not None:
                line += f"\t{ws_time}"

        print(line)


def run_all():
    # Send start notification
    start_time = time.perf_counter()
    start_content = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    start_content += f"\nWARM_START_TIME_LMT: {WARM_START_TIME_LIM}; MINUS_WARM_START_TIME: {MINUS_WARM_START_TIME}"
    u = uuid.uuid1()
    start_content += f"\n\n{u}"
    start_title = f"R.2.6 All after enhancing [START] on [{SERVER_NAME}]"
    notify(start_title, start_content)

    def exp(instance, method):
        title = f"{instance}, {method}"
        parameters = PARAMETERS.copy()
        LBBD = r_LogicBasedBendersDecomposition(instance, parameters, method, vrp_solver=True, warm_start=True)
        stat, sol = LBBD.benders_solve() if method == "LBBD" else LBBD.branch_and_check_solve()
        LBBD.save(f"./r_revision/r.2.6/{method}_solutions/{instance}", stat, sol)
        obj, gap, bks_time, time, ws_time = get_stat(f"./r_revision/r.2.6/{method}_solutions/{instance}.json")
        # content = f"{obj}, {gap}%, {bks_time}s*, {time}s, {ws_time}s (ws)"
        content = f"{obj}, {gap}%, {time}s"
        content += f"\n\n{u}"
        notify(title, content)

    for method in ["BAC", "LBBD"]:
        for instance in EXPERIMENTAL_INSTANCES_NAME:
            try:
                exp(instance, method)
            except Exception as e:
                notify(f"{instance}, {method}", "Error in vrp_solver=True, warm_start=True \n" + str(e))

    # Send end notification
    end_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    end_content = f"Total time: {round(time.perf_counter() - start_time, 2)}s \n" + end_time
    end_content += f"\n\n{u}"
    notify(f"R.2.6 Enhancement impact [END] on [{SERVER_NAME}]", end_content)


def make_efficiency_table(approaches):
    SOL_DIR_DIC = {
        "de": "./de_solutions_no_vi",
        "de_vi": "./de_solutions",
        "lbbd": "./LBBD_solutions/backup210815_8c16g",
        "bac": "./BAC_solutions/backup210815_8c16g",
        "lbbd_pp": "./r_revision/r.2.6/LBBD_solutions",
        "bac_pp": "./r_revision/r.2.6/BAC_solutions"
    }

    lines = []
    for instance in EXPERIMENTAL_INSTANCES_NAME:
        line = [instance.replace('_', '-')]
        for a in approaches:
            obj, gap, _, time, _ = get_stat(f"{SOL_DIR_DIC[a]}/{instance}.json")
            line += [obj, gap, time]
        lines.append(line)

    columns = ["instance"]
    for a in approaches:
        columns += [f"{a}_obj", f"{a}_gap", f"{a}_time"]
    df = pd.DataFrame(lines, columns=columns)
    df.set_index("instance", inplace=True)

    for a in approaches:
        df.loc[df[f"{a}_obj"] >= 10e10, f"{a}_obj"] = None
        df.loc[df[f"{a}_gap"] >= 102, f"{a}_gap"] = None
        df.loc[df[f"{a}_time"] >= 3600, f"{a}_time"] = 'Lmt.'
    s = df.style.highlight_min(subset=[f"{a}_obj" for a in approaches], axis=1, props="bf:--wrap")
    s.format(subset=[f"{a}_obj" for a in approaches] + [f"{a}_time" for a in approaches], precision=0, na_rep="-")
    s.format(subset=[f"{a}_gap" for a in approaches], precision=2, na_rep="-")


    def highlight_optimal(x):
        dff = pd.DataFrame('', index=x.index, columns=x.columns)
        for a in approaches:
            mask = x[f"{a}_gap"] == 0
            dff.loc[mask, f"{a}_obj"] = 'relax*:--rwrap'
        return dff

    s.apply(highlight_optimal, axis=None)
    print(s.to_latex())


if __name__ == '__main__':
    # enhance_impact()
    # make_enhance_contribute_table()

    # run_all()

    # Tables in the main text
    make_efficiency_table(["de", "de_vi", "lbbd", "bac", "bac_pp"])
    # Tables in the supplementary material
    # make_efficiency_table(["lbbd", "lbbd_pp", "bac", "bac_pp"])

