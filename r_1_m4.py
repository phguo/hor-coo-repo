# coding:utf-8
import json

# SOLUTION_DIR = "./BAC_solutions/backup210815_8c16g/"
SOLUTION_DIR = "./r_revision/r.2.6/BAC_solutions/"
INSTANCES = [

    '03_10_02_0_0', '03_10_02_0_1', '03_10_04_0_0', '03_10_04_0_1',
    '05_10_02_0_0', '05_10_02_0_1', '05_10_04_0_0', '05_10_04_0_1',
    '07_10_02_0_0', '07_10_02_0_1', '07_10_04_0_0', '07_10_04_0_1',

    '03_20_02_0_0', '03_20_02_0_1', '03_20_04_0_0', '03_20_04_0_1',
    '05_20_02_0_0', '05_20_02_0_1', '05_20_04_0_0', '05_20_04_0_1',
    '07_20_02_0_0', '07_20_02_0_1', '07_20_04_0_0', '07_20_04_0_1',

    '03_40_02_0_0', '03_40_02_0_1', '03_40_02_1_0', '03_40_02_1_1',
    '03_40_04_0_0', '03_40_04_0_1', '03_40_04_1_0', '03_40_04_1_1',
    '05_40_02_0_0', '05_40_02_0_1', '05_40_02_1_0', '05_40_02_1_1',
    '05_40_04_0_0', '05_40_04_0_1', '05_40_04_1_0', '05_40_04_1_1',
    '07_40_02_0_0', '07_40_02_0_1', '07_40_02_1_0', '07_40_02_1_1',
    '07_40_04_0_0', '07_40_04_0_1', '07_40_04_1_0', '07_40_04_1_1'
]


def load_json(dir):
    with open(dir, 'r') as f:
        lines = f.read()
        data = json.loads(
            lines, object_hook=lambda d: {int(k) if k.lstrip('-').isdigit() else k: v for k, v in d.items()})
    return data


if __name__ == '__main__':
    total_vdsp_num = 0
    total_runtime = 0
    run_time_li = []

    for instance in INSTANCES:
        json_dir = f"{SOLUTION_DIR}{instance}.json"
        data = load_json(json_dir)
        for v in data["RASP_stats_dic"].values():
            for li in v["VDSP_runtime_li_li"]:
                for run_time in li:
                    total_runtime += run_time
                    run_time_li.append(run_time)
                    total_vdsp_num += 1

    print(total_vdsp_num, total_runtime / total_vdsp_num)

    run_time_li.sort()
    from matplotlib import pyplot as plt

    custom_params = {
        "axes.spines.right": False,
        "axes.spines.top": False,
        "text.usetex": True,
        "font.family": "serif",
    }
    plt.rcParams.update(custom_params)

    fig, ax = plt.subplots(figsize=(5, 4))
    ax.hist(run_time_li, bins=100, color="#fd860f", edgecolor="#fd860f")
    # ax.set_xscale('log', base=10)
    ax.set_yscale('log', base=10)
    ax.set_xlim(0, 500)
    ax.set_xlabel("VDSP runtime (s)")
    ax.set_ylabel("Frequency")
    plt.savefig(f"r_vdsp_time.pdf", bbox_inches='tight', transparent=True, pad_inches=0.05)
