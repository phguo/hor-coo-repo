# coding:utf-8
# Codes are copied from visualizer.py
import json

from matplotlib import pyplot as plt

plt.rcParams.update({
    "text.usetex": True,
    "font.family": "serif",
})

from config import ALL_INSTANCES_NAME, EXPERIMENTAL_INSTANCES_NAME
from r_2_6 import get_stat

# from instance import load_json

SOL_DIR_DIC = {
    "de": "./de_solutions_no_vi",
    "de_vi": "./de_solutions",
    "lbbd": "./LBBD_solutions/backup210815_8c16g",
    "lbbd_pp": "./r_revision/r.2.6/LBBD_solutions",
    "bac": "./BAC_solutions/backup210815_8c16g",
    "bac_pp": "./r_revision/r.2.6/BAC_solutions"
}

METHOD_TITLE_DIC = {
    # "de": '',
    # "de_vi": '',
    "lbbd": "(a) NLBBD-C",
    # "lbbd_pp": "(b) NLBBD-C\\textsuperscript{++}",
    "bac": "(b) NLBBD-B\\&C",
    "bac_pp": "(c) NLBBD-B\\&C\\textsuperscript{++}"
}


def load_json(file_name):
    with open(file_name, 'r') as f:
        solution_stats = json.load(f)
    return solution_stats


def get_instance_stat(instance, method):
    stat = load_json(f"{SOL_DIR_DIC[method]}/{instance}.json")
    for key, value in stat['gap_dic'].items():
        stat['gap_dic'][key] = 1.0 if value > 1 else value
    iteration_time = max([int(a) for a in stat['lower_bound_dic'].keys()])
    if stat["SolutionInfo"]["Status"] == 2 and stat["gap_dic"][
        str(iteration_time)] != 0 and 'bac' in method:
        stat["gap_dic"][str(iteration_time + 1)] = 0
    return stat


def get_runtime_dict(instance_name, method):
    stat = load_json(f"{SOL_DIR_DIC[method]}/{instance_name}.json")
    runtime = dict()
    runtime['pmp'] = dict(stat.get("PMP_runtime_dic", dict()))  # empty for BAC solution
    runtime['rasp'] = dict(
        {key: sum(stat["RASP_stats_dic"][key]["RASP_runtime_li"]) for key in stat["RASP_stats_dic"]})
    runtime['vdsp'] = dict(
        {key: sum(sum(a) for a in stat["RASP_stats_dic"][key]["VDSP_runtime_li_li"]) for key in stat["RASP_stats_dic"]})
    return runtime


def draw_gap_curve():
    fig, axs = plt.subplots(3, 3, figsize=(9, 5))

    for ax_id, demand_num in enumerate(['10', '20', '40']):
        for j, method in enumerate(["lbbd", "bac", "bac_pp"]):
            ax = axs[ax_id][j]
            ax.spines.top.set_visible(False)
            ax.spines.right.set_visible(False)
            ax.grid(alpha=0.3, ls='--')

            # x_axes
            ax.set_xlabel(f"Iteration\\\\{METHOD_TITLE_DIC[method]}")
            ax.set_xscale('log', base=10)
            ax.set_xlim(1, 10000)
            ax.set_xticks([10, 100, 1000, 10000])
            if ax_id in [0, 1]:
                ax.get_xaxis().get_label().set_visible(False)
                ax.set_xticklabels([])

            # y_axes
            ax.set_ylabel("Gap")
            ax.set_ylim(0, 1)
            ax.set_yticks([0.1, 0.3, 0.5, 0.7, 0.9], minor=False)
            if ax_id in [0, 2]:
                ax.get_yaxis().get_label().set_visible(False)
                ax.get_yaxis().get_label().set_visible(False)
            ax.set_prop_cycle(lw=[0.5])
            if method in ["lbbd_pp", "bac", "bac_pp"]:
                ax.get_yaxis().get_label().set_visible(False)
                ax.set_yticklabels([])
            if method == "bac_pp":
                ax.get_yaxis().get_label().set_visible(True)
                ax.set_ylabel(f"$|D| = {demand_num}$")
                ax.yaxis.set_label_position("right")

            lines = []
            for i, instance_name in enumerate(a for a in ALL_INSTANCES_NAME if a[3:5] == demand_num):
                try:
                    stat = get_instance_stat(instance_name, method)
                except:
                    stat = {'gap_dic': {}}
                xs = [int(a) for a in stat['gap_dic'].keys()]
                ys = list(stat['gap_dic'].values())
                ax.plot(xs, ys, color='silver', ls='-', alpha=0.9, label="Instance")
                lines.append(ys)

            max_iteration = max(len(line) for line in lines)
            min_ys = []
            best_gap = float('inf')
            for i in range(max_iteration):
                res = min([line[i] if i < len(line) else 1.5 for line in lines])
                if res < best_gap:
                    best_gap = res
                if best_gap <= 0.00001:
                    min_ys.append(0)
                else:
                    min_ys.append(res)

            upper_limits = [max(line[i] if i < len(line) else 0 for line in lines) for i in range(max_iteration)]
            lower_limits = [min(line[i] if i < len(line) else min_ys[i] for line in lines) for i in
                            range(max_iteration)]
            mean_line = [[] for _ in range(max_iteration)]
            for line in lines:
                for i, value in enumerate(line):
                    mean_line[i].append(value)
            mean_line = [sum(line) / len(line) for line in mean_line]
            xs = [i for i in range(1, max_iteration + 1)]
            colors = ['#a5acaf', '#60636a', '#414451']
            c = colors[0]
            ax.fill_between(xs, upper_limits, lower_limits, zorder=0, alpha=.25, color=c, edgecolor='none')
            ax.plot(xs, mean_line, color="k", lw=1, label="Mean")

    def legend_without_duplicate_labels(ax):
        handles, labels = ax.get_legend_handles_labels()
        unique = [(h, l) for i, (h, l) in enumerate(zip(handles, labels)) if l not in labels[:i]]
        leg = ax.legend(*zip(*unique), fontsize=9, loc="upper right", framealpha=0.5)
        leg.get_frame().set_linewidth(0.0)

    legend_without_duplicate_labels(axs[0][2])

    fig.subplots_adjust(hspace=0)
    fig.subplots_adjust(wspace=0)
    fig.savefig(f"r_gap_curve.pdf", bbox_inches='tight', transparent=True, pad_inches=0.05)


def draw_convergence_curve(instance):
    fig, axs = plt.subplots(2, 3, figsize=(9, 3.8), gridspec_kw={'height_ratios': [3, 1]}, sharey='row')
    fig.subplots_adjust(hspace=0)
    fig.subplots_adjust(wspace=0)

    for i, method in enumerate(["lbbd", "bac", "bac_pp"]):
        highlight_color = 'k'

        stat = get_instance_stat(instance, method)
        iteration_time = max([int(a) for a in stat['lower_bound_dic'].keys()])

        if "bac" in method:
            for key, value in stat['lower_bound_dic'].items():
                stat['lower_bound_dic'][key] = 0 if value < 0 else value
            if stat["SolutionInfo"]["Status"] == 2 and stat["gap_dic"][str(iteration_time)] != 0:
                stat["gap_dic"][str(iteration_time + 1)] = 0
                stat['best_upper_bound_dic'][str(iteration_time + 1)] = stat["SolutionInfo"]["ObjVal"]
                stat['lower_bound_dic'][str(iteration_time + 1)] = stat["SolutionInfo"]["ObjVal"]

        ax = axs[0][i]

        ax.ticklabel_format(style='sci', scilimits=(-1, 2), axis='y')
        ax.spines.top.set_visible(False)
        ax.spines.right.set_visible(False)

        ax.grid(alpha=0.3, ls='--', zorder=0)
        # ax.set_title("{} ({})".format(
        #     instance_name.replace('_', '-'), "NLBBD-B\&C" if instance_type == "BAC" else "NLBBD"))
        ax.set_ylabel('Obj.', color='k')
        ax.set_yticks([i / 10 * 10 ** 6 for i in range(1, 10, 2)], minor=True)

        xs = [int(key) for key in stat['upper_bound_dic'].keys()]
        ys = [float(value) for value in stat['upper_bound_dic'].values()]
        ax.scatter(xs, ys, marker='+', color='k', linewidths=.3, s=7, alpha=.8, label="Upper bound", zorder=2)
        # ax.plot(xs, ys, c='k', lw='0.5', ls='-', label="Upper bound")

        xs = [int(key) for key, value in stat['upper_bound_dic'].items()
              if stat['upper_bound_dic'][key] == stat['best_upper_bound_dic'][key]
              and int(key) != iteration_time]
        ys = [float(value) for key, value in stat['upper_bound_dic'].items()
              if stat['upper_bound_dic'][key] == stat['best_upper_bound_dic'][key]
              and int(key) != iteration_time]
        ax.scatter(xs, ys, marker='o', color='w', edgecolor='k', linewidths=.7, s=15, zorder=3,
                   label='New best solution')

        xs = [int(key) for key in stat['best_upper_bound_dic'].keys()]
        ys = [float(value) for value in stat['best_upper_bound_dic'].values()]
        ax.plot(xs, ys, c='k', lw='1', ls='-', label="Best upper bound")

        xs = [int(key) for key in stat["lower_bound_dic"].keys()]
        ys = [float(value) for value in stat["lower_bound_dic"].values()]
        ax.plot(xs, ys, c='k', lw='0.5', ls='-',
                label="Lower bound (PMP)" if 'lbbd' in method else "Lower bound (LP)")

        if 'bac' in method:
            xs = [int(key) for key in stat['pmp_obj_dic'].keys()]
            ys = [float(value) for value in stat['pmp_obj_dic'].values()]
            ax.plot(xs, ys, c='k', lw='0.5', ls=':', label="PMP objective")

        # draw gap
        ax2 = ax.twinx()
        ax2.yaxis.label.set_color(highlight_color)
        ax2.spines["right"].set_edgecolor(highlight_color)
        ax2.tick_params(axis='y', colors=highlight_color)
        ax2.spines.top.set_visible(False)
        ax2.set_ylim(0, 1)
        ax2.set_ylabel('Gap', color=highlight_color)
        ax2.tick_params(axis='y', labelcolor=highlight_color)
        ax2.plot([int(a) for a in stat['gap_dic'].keys()], stat['gap_dic'].values(), c=highlight_color, lw=.5, ls='--',
                 label='Gap', alpha=1, zorder=4)
        ax.plot([], [], c=highlight_color, lw=.5, ls='--', label='Gap')  # Make an agent in ax

        if i in [0, 1]:
            ax2.set_yticklabels([])
            ax2.set_yticks([])
            ax2.set_ylabel('')

        # draw runtime
        ax_ = axs[1][i]
        runtime_dic = get_runtime_dict(instance, method)
        ordered_keys = sorted([int(i) for i in runtime_dic['vdsp'].keys()])
        pmp_time = [runtime_dic['pmp'].get(str(i), 0) for i in ordered_keys]
        rasp_time = [runtime_dic['rasp'][str(i)] for i in ordered_keys]
        vdsp_time = [runtime_dic['vdsp'][str(i)] for i in ordered_keys]
        sum_time = [sum([pmp_time[i], rasp_time[i], vdsp_time[i]]) for i, value in enumerate(rasp_time)]
        ax_.plot(ordered_keys, sum_time, label='Total', ls='-', color='silver', zorder=0, alpha=1, lw=2)
        if 'lbbd' in method:
            ax_.plot(ordered_keys, pmp_time, label='PMP', c='k', ls='-', lw=0.5)
        ax_.plot([], [], label='PMP', c='k', ls='-', lw=0.5)
        ax_.scatter(ordered_keys, vdsp_time, label='VDSP', c='k', s=.3, marker='+', alpha=.7)
        ax_.plot(ordered_keys, rasp_time, label='RASP', c='k', ls=':', lw=0.5)
        ax_.set_yscale('log', base=10)
        if method == 'bac_pp':
            ax_.legend(ncol=1, fontsize=7, framealpha=0.5, frameon=False, loc='upper right')
        ax_.set_xticks([])

        ax_.set_xlabel(f"Iteration\\\\{METHOD_TITLE_DIC[method]}")
        ax_.set_ylim((10 ** -3, 10 ** 1))
        ax_.set_yticks((10 ** -3, 10 ** -2, 10 ** -1, 10 ** 0, 10 ** 1))
        ax_.set_yticklabels(('', '$10^{-2}$', '$10^{-1}$', '', ''))
        ax_.set_ylabel('Time (s)')

        ax_.grid(alpha=0.3, ls='--', zorder=0, axis='x')
        # x_ticks_num = 10
        # ax.set_xticks(range(0, ceil(xs[-1] / 50) * 50, int(ceil(xs[-1] / 50) * 50 / x_ticks_num)))

        x_ticks = range(0, 410, 50)
        x_lim = (-20, 420)
        ax.set_xticks(x_ticks)
        ax2.set_xticks(x_ticks)
        ax_.set_xticks(x_ticks)
        ax.set_ylim(0, .8 * 10 ** 6)
        ax.set_xlim(x_lim[0], x_lim[1])
        ax2.set_xlim(x_lim[0], x_lim[1])
        ax_.set_xlim(x_lim[0], x_lim[1])
        ax.xaxis.set_ticklabels([])

        if method == 'bac_pp':
            leg = ax.legend(fontsize=9, framealpha=0.5, loc="upper right", ncol=1)
            leg.get_frame().set_linewidth(0.0)

    axs[0][1].set_ylabel('')
    axs[0][2].set_ylabel('')
    axs[1][1].set_ylabel('')
    axs[1][2].set_ylabel('')

    fig.savefig(f"r_cvg_curve.pdf", bbox_inches='tight', transparent=True, pad_inches=0.05)
    plt.close()


if __name__ == '__main__':
    draw_gap_curve()
    draw_convergence_curve('03_10_02_0_0')
