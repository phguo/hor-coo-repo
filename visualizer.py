# coding:utf-8
import itertools
from math import ceil, log
from pprint import pprint

import matplotlib.patches as mpatches
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.collections import PatchCollection
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from exp_lateral_trans import get_exp_lateral_trans_data
from exp_pareto import get_pareto_frontiers_data
from exp_stochasticity import get_expected_value_data, get_wait_and_see_data
from exp_vehicle_routing import get_vehicle_routing_data, transportation_cost_estimate
from solution import GurobiSolution
import matplotlib.patheffects as path_effects

from matplotlib.path import Path
import matplotlib.patches as patches

from adjustText import adjust_text

plt.rcParams.update({
    # "text.usetex": True,
    # "font.family": "serif",
    "font.family": ['Arial', 'SimHei'],

    # 'axes.labelsize': 7,
    # 'font.size': 7,
    # 'legend.fontsize': 7,
    # 'xtick.labelsize': 7,
    # 'ytick.labelsize': 7,
})

from config import DRAW_SCENARIO_ID, FILE_DIR, DRAW_INSTANCE_NAMES, EXP_PARAMETERS, COPY_TO_CLIPBOARD, \
    ALL_INSTANCES_NAME, EXPERIMENTAL_INSTANCES_NAME, LANG_ZH
from instance import Instance
from io_operator import solved_instance_classify, get_runtime, get_objective_value, get_gap, get_solution_stats


def copy_to_clipboard(content):
    if COPY_TO_CLIPBOARD:
        import clipboard
        clipboard.copy(content)


def draw_route():
    for instance_name in EXP_PARAMETERS['originally_optimized']:
        G = GurobiSolution(instance_name, "DE_NO_VI")
        loc = G.points_loc
        solution = G.scenarios_routes
        print('\n ' + instance_name, G.lateral_trans_applied)
        for s in solution:
            for i in solution[s]:
                for h in solution[s][i]:
                    for route in solution[s][i][h]:
                        route = [str(a) for a in route if a in G.facility_set and a != i]
                        print(s, i, h, route)


def draw_instance(instance):
    plt.figure(figsize=(6, 6))

    def draw_area(instance):

        risk_areas = []
        cluster_areas = []

        if instance.is_clustered_instance:
            for c, c_center in instance.clusters_center.items():
                cluster_area = mpatches.Rectangle(
                    (c_center[0] - instance.cluster_radius, c_center[1] - instance.cluster_radius),
                    instance.cluster_radius * 2, instance.cluster_radius * 2)
                cluster_areas.append(cluster_area)

        if instance.is_line_disaster_center:
            rectangle = mpatches.Rectangle(
                (instance.square[0], instance.square[1] / 2 - instance.step_function[0][0][1]),
                instance.square[1], instance.step_function[0][0][1] * 2)
            risk_areas.append(rectangle)
            rectangle = mpatches.Rectangle(
                (instance.square[0], instance.square[1] / 2 - instance.step_function[1][0][1]),
                instance.square[1], instance.step_function[1][0][1] * 2)
            risk_areas.append(rectangle)

        if not instance.is_line_disaster_center:
            center = ((instance.square[1] - instance.square[0]) / 2, (instance.square[1] - instance.square[0]) / 2)
            circle = mpatches.Circle(center, instance.step_function[1][0][1])
            risk_areas.append(circle)
            circle = mpatches.Circle(center, instance.step_function[0][0][1])
            risk_areas.append(circle)

        cluster_collection = PatchCollection(cluster_areas, ls='dotted', alpha=1, zorder=-1, ec='k', fc='None')
        risk_collection = PatchCollection(risk_areas, alpha=.25, color='grey', zorder=0, ec='None')
        ax.add_collection(cluster_collection)
        ax.add_collection(risk_collection)

    def marker_size(instance, demand_id):
        demand_quantity = instance.scenarios[DRAW_SCENARIO_ID][demand_id][0]
        # demand_quantity = instance.points_population[demand_id]
        return (demand_quantity / 25 * 3) ** 2

    fig, ax = plt.subplots()
    ax.set_xticks([i * 100 for i in range(11)], minor=True)
    ax.set_yticks([i * 100 for i in range(11)], minor=True)
    ax.set_xticks([instance.square[0], (instance.square[1] - instance.square[0]) / 2, instance.square[1]])
    ax.set_yticks([instance.square[0], (instance.square[1] - instance.square[0]) / 2, instance.square[1]])
    ax.grid(which='major', alpha=1, ls='-', zorder=-2, c='dimgrey')
    ax.grid(which='minor', alpha=0.2, ls='--', zorder=-2, c='dimgrey')
    ax.set_aspect(1)
    demand_points_style = {
        'marker': 'o', 's': 40, 'linewidth': 1,
        'c': 'w', 'edgecolors': 'k',
        'zorder': 2, 'alpha': 0.8
    }
    isolated_demand_style = {
        'marker': 'X', 's': 40, 'linewidth': 1,
        'c': 'whitesmoke', 'edgecolors': 'k',
        'zorder': 2, 'alpha': 0.8
    }
    facility_points_style = {
        'marker': 'P', 's': 600, 'linewidth': 1,
        'c': 'r', 'edgecolors': 'k',
        'alpha': 0.9, 'zorder': 3
    }

    if instance.is_clustered_instance:
        facility_x, facility_y = [], []
        for c in instance.clusters:
            # cluster_center_loc = instance.clusters_center[c]
            # print(cluster_center_loc)
            demand_x, demand_y = [], []
            isolated_demand_x, isolated_demand_y = [], []
            demand_area = []
            isolated_demand_area = []
            for key in instance.clusters[c]:
                if key in instance.demand_set:
                    if key in instance.isolated_demand_in_scenario[DRAW_SCENARIO_ID]:
                        isolated_demand_x.append(instance.clusters[c][key][0])
                        isolated_demand_y.append(instance.clusters[c][key][1])
                        isolated_demand_area.append(marker_size(instance, key))
                    else:
                        demand_x.append(instance.clusters[c][key][0])
                        demand_y.append(instance.clusters[c][key][1])
                        demand_area.append(marker_size(instance, key))
                elif key in instance.facility_set:
                    facility_x.append(instance.clusters[c][key][0])
                    facility_y.append(instance.clusters[c][key][1])
                else:
                    pass
            demand_points_style['s'] = demand_area
            isolated_demand_style['s'] = isolated_demand_area
            ax.scatter(demand_x, demand_y, **demand_points_style)
            ax.scatter(isolated_demand_x, isolated_demand_y, **isolated_demand_style)
        ax.scatter(facility_x, facility_y, **facility_points_style)

        # demand_points_style['s'] = 50
        # isolated_demand_style['s'] = 50
        # facility_points_style['s'] = 50
        # ax.scatter([-1000], [-1000], **facility_points_style, label='Relief facility')
        # ax.scatter([-1000], [-1000], **isolated_demand_style, label='Isolated community')
        # ax.scatter([-1000], [-1000], **demand_points_style, label='Non isolated community')
        # leg = ax.legend(ncol=1, fontsize=9, loc='lower left', framealpha=0.8, edgecolor='none')
        # leg.get_frame().set_linewidth(0.0)

    else:
        demand_x, demand_y = [], []
        isolated_demand_x, isolated_demand_y = [], []
        demand_area = []
        isolated_demand_area = []
        for j in instance.demand_set:
            if j in instance.isolated_demand_in_scenario[DRAW_SCENARIO_ID]:
                isolated_demand_x.append(instance.points_loc[j][0])
                isolated_demand_y.append(instance.points_loc[j][1])
                isolated_demand_area.append(marker_size(instance, j))
            else:
                demand_x.append(instance.points_loc[j][0])
                demand_y.append(instance.points_loc[j][1])
                demand_area.append(marker_size(instance, j))
        facility_x, facility_y = [], []
        for i in instance.facility_set:
            facility_x.append(instance.points_loc[i][0])
            facility_y.append(instance.points_loc[i][1])
        demand_points_style['s'] = demand_area
        isolated_demand_style['s'] = isolated_demand_area
        ax.scatter(demand_x, demand_y, **demand_points_style)
        ax.scatter(isolated_demand_x, isolated_demand_y, **isolated_demand_style)
        ax.scatter(facility_x, facility_y, **facility_points_style)

    demand_points_style['s'] = 50
    isolated_demand_style['s'] = 50
    facility_points_style['s'] = 50
    label = "援助设施" if LANG_ZH else 'Relief facility'
    ax.scatter([-1000], [-1000], **facility_points_style, label=label)
    label = "孤立点" if LANG_ZH else 'Isolated community'
    ax.scatter([-1000], [-1000], **isolated_demand_style, label=label)
    label = "非孤立点" if LANG_ZH else 'Non isolated community'
    ax.scatter([-1000], [-1000], **demand_points_style, label=label)
    leg = ax.legend(ncol=1, fontsize=9, loc='lower left', framealpha=0.8, edgecolor='none')
    leg.get_frame().set_linewidth(0.0)

    ax.set_xlim(instance.square[0] - 100, instance.square[1] + 100)
    ax.set_ylim(instance.square[0] - 100, instance.square[1] + 100)
    figure_title = instance.instance_name.replace('_', '-') + ' ($s=I$)'
    plt.title(figure_title)
    # plt.legend()
    # plt.rc('font', family='Times New Roman')
    # plt.rc('axes', axisbelow=True)
    # plt.xlabel('x-axis')
    # plt.ylabel('y-axis')
    draw_area(instance)
    plt.savefig(FILE_DIR['figures'] + 'vis_' + instance.instance_name + '.pdf',
                bbox_inches='tight', transparent=True, pad_inches=0.05)
    # plt.show()
    return plt


def visualize_instances():
    for instance_name in DRAW_INSTANCE_NAMES:
        instance = Instance(instance_name)
        draw_instance(instance)


def algorithm_comparison_table():
    def tex_format(content):
        pre_facility_num = None
        pre_demand_num = None
        lines = ''
        for row in content:
            instance_name = row[0]
            instance_name = instance_name.replace('_', '-')
            facility_num = int(instance_name[:2])
            demand_num = int(instance_name[3:5])

            for i, cell in enumerate(row):
                if cell != None:
                    if i in [1, 4, 7, 10]:  # objective value
                        if cell == float(1e+100) or cell == float('inf'):
                            row[i] = '-'
                        else:
                            # row[i] = float(round(row[i], 2))
                            # row[i] = "%.2f" % row[i]
                            row[i] = int(float(round(row[i], 0)))
                            row[i] = str(row[i])

                    elif i in [2, 5, 8, 11]:  # gap
                        if cell == float(1e+100) or cell == float('inf'):
                            row[i] = '-'
                        else:
                            row[i] = float(round(row[i] * 100, 2))
                            row[i] = "%.2f" % row[i]
                            if row[i] == '0.00':
                                # row[i - 1] += '*'
                                row[i - 1] = '*' + row[i - 1]

                    elif i in [3, 6, 9, 12]:  # run time
                        row[i] = float(round(row[i], 2))
                        # row[i] = "%.2f" % row[i]
                        if row[i] >= 3600:
                            row[i] = "Lmt."
                        else:
                            row[i] = "%.2f" % row[i]
                else:
                    row[i] = ''

            obj_list = [row[1], row[4], row[7], row[10]]
            for i, content in enumerate(obj_list):
                if content in ['', '-']:
                    obj_list[i] = 'inf'
                else:
                    obj_list[i] = content.replace('*', '')
                obj_list[i] = float(obj_list[i])
            min_obj_value = min(obj_list)
            min_obj_cols = [i * 3 + 1 for i, v in enumerate(obj_list) if v == min_obj_value]
            for col in min_obj_cols:
                row[col] = "{{\\bf {}}}".format(row[col]) if row[col] != '-' else row[col]
            if facility_num != pre_facility_num:
                lines += "\midrule\n"
            # if facility_num == pre_facility_num and demand_num != pre_demand_num:
            if demand_num == 40 and pre_demand_num == 20:
                lines += "\cline{2-13}\n"
            line = "        {} & {} & {} & {} & {} & {} & {} & {} & {} & {} & {} & {} & {} \\\\".format(instance_name,
                                                                                                        *row[1:])
            pre_facility_num = facility_num
            pre_demand_num = demand_num
            lines += line + '\n'

        print(lines)
        copy_to_clipboard(lines)
        return lines

    def summary_tex_format(content):
        idx = [0, 1, 2, 3, 4]
        nums = [[3, 5, 7], [10, 20, 40], [2, 4], [0, 1], [0, 1]]
        selected = [2, 3, 5, 6, 8, 9, 11, 12]
        for i, num_ in zip(idx, nums):
            for num in num_:
                line = [[a[j] for a in content if int(a[0].split('_')[i]) == num and a[j] < 1e99] for j in selected]
                line = [i, num] + [round(sum(a) / len(a), 2) for a in line]
                print(" & ".join([str(a) for a in line]), '\\\\')

    de_type = "DE"
    de_no_vi_type = "DE_NO_VI"
    lbbd_type = "LBBD"
    bac_type = "BAC"
    all_instance_names = [a[0] for a in solved_instance_classify(de_type, True)[2]]
    content = []
    for instance in all_instance_names:
        de_no_vi_gap = get_gap(instance, de_no_vi_type)
        de_no_vi_run_time = get_runtime(instance, de_no_vi_type)
        de_no_vi_objective = get_objective_value(instance, de_no_vi_type)

        de_gap = get_gap(instance, de_type)
        de_run_time = get_runtime(instance, de_type)
        de_objective = get_objective_value(instance, de_type)

        lbbd_gap = get_gap(instance, lbbd_type)
        lbbd_run_time = get_runtime(instance, lbbd_type)
        lbbd_objective = get_objective_value(instance, lbbd_type)

        bac_gap = get_gap(instance, bac_type)
        bac_run_time = get_runtime(instance, bac_type)
        bac_objective = get_objective_value(instance, bac_type)

        row = [instance,
               de_no_vi_objective, de_no_vi_gap, de_no_vi_run_time,
               de_objective, de_gap, de_run_time,
               lbbd_objective, lbbd_gap, lbbd_run_time,
               bac_objective, bac_gap, bac_run_time
               ]
        content.append(row)

    tex_format(content)
    # summary_tex_format(content)
    return content


def _draw_pareto(draw_type, pareto_frontiers_data):
    # plt.figure(figsize=(5, 5))
    # if draw_type == 1:
    #     plt.figure(figsize=(3, 6))
    # elif draw_type == 2:
    #     plt.figure(figsize=(3, 6))
    # elif draw_type == 3:
    #     plt.figure(figsize=(2.5, 2.5))
    # elif draw_type == 4:
    #     plt.figure(figsize=(2.5, 2.5))

    fig, ax = plt.subplots(figsize=(3, 6))
    if draw_type in [3, 4]:
        fig, ax = plt.subplots(figsize=(2.5, 2.5))

    marker = itertools.cycle(('d', 'v', 'o', 'X', 's'))
    line = itertools.cycle(('--', '-', ':'))
    # color = itertools.cycle(
    #     [
    #         # '#a6cee3',
    #         '#1f78b4',
    #         # '#b2df8a', '#33a02c', '#fb9a99',
    #         # '#e31a1c', '#fdbf6f', '#ff7f00', '#cab2d6'
    #     ])
    color = itertools.cycle(['w'])
    style = {}
    for instance_name in EXP_PARAMETERS['originally_optimized']:
        instance_name = instance_name.replace('_', '-')
        style[instance_name] = {'ls': next(line), 'mfc': next(color), 'marker': next(marker)}

    for instance_name in EXP_PARAMETERS['originally_optimized']:
        instance_name = instance_name.replace('_', '-')
        facility_num = int(instance_name[0:2])
        scenario_num = int(instance_name[6:8])
        type_id = int(instance_name[-1])
        ls = '-' if type_id == 0 else '--'
        c = ['#a5acaf', '#60636a', '#414451']
        mfc = c[2] if facility_num == 3 else ('silver' if facility_num == 5 else 'w')
        marker = 'o' if scenario_num == 2 else 'd'
        style[instance_name] = {'ls': ls, 'mfc': mfc, 'marker': marker}

    if draw_type == 1:
        for instance_name, x_li, y_li in pareto_frontiers_data:
            instance_name = instance_name.replace('_', '-')
            new_y_li = list(map(lambda a: a / 1000 if a else None, y_li))
            ax.plot(x_li, new_y_li,
                    color='k', lw='1', mec='k', mew=.7, markersize=5,
                    # ls=next(line), mfc=next(color), marker=next(marker),
                    **style[instance_name],
                    label=instance_name,
                    )
        plt.ylabel(r'Total cost (\(\times 10^3\))')

    elif draw_type in [2, 3]:
        data = []
        for instance_name, x_li, y_li in pareto_frontiers_data:
            instance_name = instance_name.replace('_', '-')
            try:
                if draw_type == 2:
                    new_y_li = [(y_li[i] - y_li[0]) / y_li[0] * 100 for i in range(0, len(y_li))]
                else:
                    new_y_li = [(y_li[i] - y_li[i - 1]) / y_li[i - 1] * 100 for i in range(1, len(y_li))]
                    x_li = x_li[1:]
            except:
                if draw_type == 2:
                    new_y_li = [None for _ in range(len(y_li))]
                else:
                    new_y_li = [None for _ in range(1, len(y_li))]
                    x_li = x_li[1:]
            data.append((instance_name, x_li, new_y_li))

        data.sort(key=lambda a: a[-1][-1] if a[-1][-1] else float('inf'), reverse=True)
        for instance_name, x_li, y_li in data:
            ax.plot(x_li, y_li, color='k', lw='1', mec='k', mew=.7, markersize=5,
                    # ls=next(line), mfc=next(color), marker=next(marker),
                    **style[instance_name],
                    label=instance_name,
                    )
        ax.set_ylabel(r'Cost increase (\(\%\))')

    elif draw_type == 4:
        data = []
        for instance_name, x_li, y_li in pareto_frontiers_data:
            instance_name = instance_name.replace('_', '-')
            try:
                new_y_li = [(y_li[i] - y_li[i - 1]) / y_li[i - 1] * 100 for i in range(1, len(y_li))]
                x_li = x_li[1:]
            except:
                new_y_li = [None for _ in range(1, len(y_li))]
                x_li = x_li[1:]
            data.append((instance_name, x_li, new_y_li))
        data.sort(key=lambda a: a[-1][-1] if a[-1][-1] else float('inf'), reverse=True)
        result = [[] for a in EXP_PARAMETERS['exp_pareto_list'][1:]]
        for i, content in enumerate(data):
            for j in range(len(result)):
                result[j].append(content[2][j])
        ax.boxplot(
            [[]] + result,
            labels=EXP_PARAMETERS['exp_pareto_list'],
            medianprops=dict(color='k'),
            flierprops=dict(marker='x', markersize=3),
            showfliers=True,
            # showbox=False,
            # showcaps=False,
        )
        # print(EXP_PARAMETERS['exp_pareto_list'])
        ax.set_xticks([0, 1, 3, 5, 7, 9], minor=True)
        ax.set_xticks([2, 4, 6, 8, 10])
        ax.set_xticklabels([0.2, 0.4, 0.6, 0.8, 1.0])
        # for ins, x, y in data:
        #     ax.scatter(
        #         [a * 10 for a in x], y,
        #         **{
        #             'marker': '.', 's': 10, 'c': 'none', 'zorder': 10, 'alpha': 1,
        #             #  'linewidth': 0.5,
        #             # 'edgecolors': 'none',
        #         }
        #     )

        plt.ylabel(r'Cost increase (\(\%\))')

    else:
        assert Exception("Unknown draw type: '{}'".format(draw_type))

    # ax.set_xticks(EXP_PARAMETERS['exp_pareto_list'])

    ax.grid(alpha=0.3, ls='--')

    if draw_type in [1, 2, 3]:
        ax.set_xticks([0.1, 0.3, 0.5, 0.7, 0.9], minor=True)
        ax.set_xticks([0.2, 0.4, 0.6, 0.8, 1.0])
        ax.set_xlabel(r'Service level ($r$)')

        if draw_type == 2:
            ax.legend(loc='upper left', fontsize=8, frameon=False)
        ax.set_xlim(0, 1.05)
        ax.set_ylim(-5, 60)
        if draw_type == 1:
            ax.set_ylim(0, 600)
        elif draw_type == 2:
            ax.set_ylim(-10, 120)
            ax.set_yticks([-10, 15, 40, 65, 90, 115, 140])
        elif draw_type == 3:
            ax.set_ylim(0, 20)

    elif draw_type in [4]:
        ax.grid(alpha=0.3, ls='--')
        ax.set_xlabel(r'Service level ($r$)')
        ax.set_ylim(0, 20)

    ax.spines.top.set_visible(False)
    ax.spines.right.set_visible(False)
    plt.savefig(FILE_DIR['figures'] + 'pareto{}.pdf'.format(draw_type),
                bbox_inches='tight', transparent=True, pad_inches=0.01)


def draw_pareto_frontiers(data):
    _draw_pareto(1, data)
    _draw_pareto(2, data)
    _draw_pareto(3, data)
    _draw_pareto(4, data)


def lateral_trans_table(data):
    def tex_format(content):
        lines = ""
        for li in content:
            if li[0] == "Min.":
                lines += "\midrule\n"
            li[0] = li[0].replace('_', '-')
            for i, value in enumerate(li):
                if value == None:
                    li[i] = float('inf')
            # line_content = "        {} & " \
            #                "{:.2f} & {:.2f} & " \
            #                "{:.2f} & {:.2f} & " \
            #                "{:.2f} & {:.2f} & " \
            #                "{:.2f} & {:.2f} ".format(*li)
            line_content = "{} & " \
                           "{:.2f} & {:.2f} & {:.2f} & " \
                           "{:.2f} & {:.2f} & {:.2f} & " \
                           "{:.2f} & {:.2f} & {:.2f} " \
                .format(*li)
            lines += line_content + "\\\\\n"
        lines = lines.replace("inf", '-')
        lines = lines.replace('-0.00', '0.00')
        print(lines)
        copy_to_clipboard(lines)
        return lines

    pre_instance_name = None
    result = []
    row = []
    for line in data:
        instance_name = line[0]
        if instance_name != pre_instance_name:
            row = [instance_name]
            row += line[2:]
            result.append(row)
        else:
            row += line[2:]
        pre_instance_name = instance_name

    result = add_stats_to_table_bottom(result)
    tex_format(result)
    return result


def vehicle_routing_table(data):
    def tex_format(content):
        lines = ""
        for li in content:
            if li[0] == "Min.":
                lines += "        \midrule\n"
            li[0] = li[0].replace('_', '-')
            for i, value in enumerate(li):
                if i != 0:
                    if value == None:
                        li[i] = float('inf')
                    else:
                        li[i] = value * 100
            line_content = "        {} & {:.2f} & {:.2f} & {:.2f} & {:.2f} " \
                           "& {:.2f} & {:.2f} " \
                           "& {:.2f} & {:.2f} & {:.2f} & {:.2f} ".format(*li)
            line_content = line_content.replace('inf', '-')
            lines += line_content + "\\\\\n"
        print(lines)
        copy_to_clipboard(lines)
        return lines

    pre_instance_name = None
    result = []
    row = []
    for line in data:
        instance_name = line[0]
        if instance_name != pre_instance_name:
            row = [instance_name]
            row += line[3:]
            result.append(row)
        else:
            row += line[3:]
        pre_instance_name = instance_name

    result = add_stats_to_table_bottom(result)
    tex_format(result)
    return result


def stochasticity_table(expected_value_data, wait_and_see_data):
    def tex_format(content):
        lines = ""
        for li in content:
            if li[0] == "Min.":
                lines += "        \midrule\n"
            li[0] = li[0].replace('_', '-')
            for i, value in enumerate(li):
                if i != 0:
                    if value == None:
                        li[i] = float('inf')
                    else:
                        li[i] = value * 100
            li[1], li[5] = li[5], li[1]
            li[2], li[6] = li[6], li[2]
            line_content = "        {} & {:.2f} & {:.2f} & {:.2f} & {:.2f} & " \
                           "{:.2f} & {:.2f} & " \
                           "{:.2f} & {:.2f} & {:.2f} & {:.2f} ".format(*li)
            line_content = line_content.replace('inf', '-')
            lines += line_content + "\\\\\n"
        print(lines)
        copy_to_clipboard(lines)
        return lines

    pre_instance_name = None
    result = []
    row = []
    instance_num = len(expected_value_data)
    instance_num_b = len(wait_and_see_data)
    assert instance_num == instance_num_b

    for i in range(instance_num):
        instance_name = expected_value_data[i][0]
        instance_name_b = wait_and_see_data[i][0]
        assert instance_name == instance_name_b
        if instance_name != pre_instance_name:
            row = [instance_name]
            for j in range(1, len(expected_value_data[i])):
                row += [expected_value_data[i][j]]
                row += [wait_and_see_data[i][j]]
            result.append(row)
        else:
            for j in range(1, len(expected_value_data[i])):
                row += [expected_value_data[i][j]]
                row += [wait_and_see_data[i][j]]
        pre_instance_name = instance_name

    result = add_stats_to_table_bottom(result)
    tex_format(result)
    return result


def add_stats_to_table_bottom(result):
    min_list = ['Min.']
    med_list = ['Med.']
    avg_list = ['Avg.']
    max_list = ['Max.']
    for i in range(1, len(result[0])):
        exp_result = [result[ins][i] for ins in range(len(result)) if result[ins][i] != None]
        if exp_result:
            min_list.append(min(exp_result))
            med_list.append((exp_result[len(exp_result) // 2] + exp_result[~len(exp_result) // 2]) / 2)
            avg_list.append(sum(exp_result) / len(exp_result))
            max_list.append(max(exp_result))
        else:
            min_list.append(None)
            med_list.append(None)
            avg_list.append(None)
            max_list.append(None)
    result.append(min_list)
    result.append(med_list)
    result.append(avg_list)
    result.append(max_list)
    return result


def _get_runtime_dict(instance_name, instance_type):
    assert instance_type in ["BAC", "LBBD"]
    stat = get_solution_stats(instance_name, instance_type)
    runtime = dict()
    runtime['pmp'] = dict(stat.get("PMP_runtime_dic", dict()))  # empty for BAC solution
    runtime['rasp'] = dict(
        {key: sum(stat["RASP_stats_dic"][key]["RASP_runtime_li"]) for key in stat["RASP_stats_dic"]})
    runtime['vdsp'] = dict(
        {key: sum(sum(a) for a in stat["RASP_stats_dic"][key]["VDSP_runtime_li_li"]) for key in stat["RASP_stats_dic"]})
    return runtime


def _convergence_curve(instance_name, instance_type):
    highlight_color = '#1f78b4'
    highlight_color = 'k'

    stat = get_solution_stats(instance_name, instance_type)
    iteration_time = max([int(a) for a in stat['lower_bound_dic'].keys()])

    if instance_type == "BAC":
        for key, value in stat['lower_bound_dic'].items():
            stat['lower_bound_dic'][key] = 0 if value < 0 else value
        if stat["SolutionInfo"]["Status"] == 2 and stat["gap_dic"][str(iteration_time)] != 0:
            stat["gap_dic"][str(iteration_time + 1)] = 0
            stat['best_upper_bound_dic'][str(iteration_time + 1)] = stat["SolutionInfo"]["ObjVal"]
            stat['lower_bound_dic'][str(iteration_time + 1)] = stat["SolutionInfo"]["ObjVal"]

    # draw bound
    # fig, ax = plt.subplots(figsize=(4, 3))
    fig, axs = plt.subplots(2, 1, sharex='row', figsize=(4, 3), gridspec_kw={'height_ratios': [3, 1]})
    fig.subplots_adjust(hspace=0)

    ax = axs[0]

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
    ax.scatter(xs, ys, marker='+', color='k', linewidths=.3, s=7, alpha=.8, label="Upper bound",
               zorder=2)
    # ax.plot(xs, ys, c='k', lw='0.5', ls='-', label="Upper bound")

    xs = [int(key) for key, value in stat['upper_bound_dic'].items()
          if stat['upper_bound_dic'][key] == stat['best_upper_bound_dic'][key]
          and int(key) != iteration_time]
    ys = [float(value) for key, value in stat['upper_bound_dic'].items()
          if stat['upper_bound_dic'][key] == stat['best_upper_bound_dic'][key]
          and int(key) != iteration_time]
    ax.scatter(xs, ys, marker='o', color='w', edgecolor='k', linewidths=.7, s=15, zorder=3, label='New best solution')

    xs = [int(key) for key in stat['best_upper_bound_dic'].keys()]
    ys = [float(value) for value in stat['best_upper_bound_dic'].values()]
    ax.plot(xs, ys, c='k', lw='1', ls='-', label="Best upper bound")

    xs = [int(key) for key in stat["lower_bound_dic"].keys()]
    ys = [float(value) for value in stat["lower_bound_dic"].values()]
    ax.plot(xs, ys, c='k', lw='0.5', ls='-',
            label="Lower bound (PMP)" if instance_type == "LBBD" else "Lower bound (LP)")

    if instance_type == "BAC":
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

    # draw runtime
    ax_ = axs[1]
    runtime_dic = _get_runtime_dict(instance_name, instance_type)
    ordered_keys = sorted([int(i) for i in runtime_dic['vdsp'].keys()])
    pmp_time = [runtime_dic['pmp'].get(str(i), 0) for i in ordered_keys]
    rasp_time = [runtime_dic['rasp'][str(i)] for i in ordered_keys]
    vdsp_time = [runtime_dic['vdsp'][str(i)] for i in ordered_keys]
    sum_time = [sum([pmp_time[i], rasp_time[i], vdsp_time[i]]) for i, value in enumerate(rasp_time)]
    ax_.plot(ordered_keys, sum_time, label='Total', ls='-', color='silver', zorder=0, alpha=1, lw=2)
    if instance_type == "LBBD":
        ax_.plot(ordered_keys, pmp_time, label='PMP', c='k', ls='-', lw=0.5)
    ax_.plot([], [], label='PMP', c='k', ls='-', lw=0.5)
    ax_.scatter(ordered_keys, vdsp_time, label='VDSP', c='k', s=.3, marker='+', alpha=.7)
    ax_.plot(ordered_keys, rasp_time, label='RASP', c='k', ls=':', lw=0.5)
    ax_.set_yscale('log', base=10)
    if instance_type == "BAC":
        ax_.legend(ncol=4, fontsize=7, frameon=False, loc='upper right')
    ax_.set_xticks([])

    ax_.set_xlabel("Iteration")
    ax_.set_ylim((10 ** -3, 10 ** 1))
    ax_.set_yticks((10 ** -3, 10 ** -2, 10 ** -1, 10 ** 0, 10 ** 1))
    ax_.set_yticklabels(('', '$10^{-2}$', '$10^{-1}$', '', ''))
    ax_.set_ylabel('Time (s)')

    ax_.grid(alpha=0.3, ls='--', zorder=0, axis='x')
    # x_ticks_num = 10
    # ax.set_xticks(range(0, ceil(xs[-1] / 50) * 50, int(ceil(xs[-1] / 50) * 50 / x_ticks_num)))

    x_ticks = range(0, 410, 50)
    x_lim = (-10, 400)
    ax.set_xticks(x_ticks)
    ax2.set_xticks(x_ticks)
    ax_.set_xticks(x_ticks)
    ax.set_ylim(0, .8 * 10 ** 6)
    ax.set_xlim(x_lim[0], x_lim[1])
    ax2.set_xlim(x_lim[0], x_lim[1])
    ax_.set_xlim(x_lim[0], x_lim[1])
    ax.xaxis.set_ticklabels([])

    # save figure
    if instance_type == "BAC":
        leg = ax.legend(fontsize=7, framealpha=1, loc="upper right", ncol=2)
        leg.get_frame().set_linewidth(0.0)

    fig.savefig(
        FILE_DIR['figures'] + "cvg_{}_{}.pdf".format(instance_name, instance_type),
        bbox_inches='tight', transparent=True, pad_inches=0.05)
    plt.clf()
    plt.close()


def draw_convergence_curve():
    instances_name, instances_type = ["03_10_02_0_0"], ["BAC", "LBBD"]
    # instances_name, instances_type = EXPERIMENTAL_INSTANCES_NAME, ["BAC"]
    # instances_name, instances_type = ALL_INSTANCES_NAME, ["BAC", "LBBD"]
    for instance_type in instances_type:
        for instance_name in instances_name:
            _convergence_curve(instance_name, instance_type)


def _gap_curve_d3(instance_type):
    highlight_color = '#1f78b4'
    highlight_color = 'k'

    fig = plt.figure(figsize=(4, 3))
    ax = fig.add_subplot(projection='3d')

    ax.set_xlim(0, 4)
    ax.set_xlabel("Iteration")

    ax.set_ylabel("Instance")
    ax.set_yticks([0, 1, 2])
    ax.set_yticklabels(['10', '20', '40'])
    # ax.get_proj = lambda: np.dot(Axes3D.get_proj(ax), np.diag([1, 2, 1, 1]))

    ax.set_zlabel("Gap")
    ax.set_zlim(0, 1)

    ax.view_init(15, -80)
    ax.set_title("{}".format("NLBBD-B\&C" if instance_type == "BAC" else "NLBBD"))
    ax.spines.top.set_visible(False)
    ax.spines.right.set_visible(False)
    ax.grid(alpha=0.3, ls='--')

    for i, instance_name in enumerate(ALL_INSTANCES_NAME):
        stat = get_solution_stats(instance_name, instance_type)
        for key, value in stat['gap_dic'].items():
            stat['gap_dic'][key] = 1.0 if value > 1 else value
        iteration_time = max([int(a) for a in stat['lower_bound_dic'].keys()])
        if stat["SolutionInfo"]["Status"] == 2 and stat["gap_dic"][str(iteration_time)] != 0 and instance_type == 'BAC':
            stat["gap_dic"][str(iteration_time + 1)] = 0
            stat['best_upper_bound_dic'][str(iteration_time + 1)] = stat["SolutionInfo"]["ObjVal"]
            stat['lower_bound_dic'][str(iteration_time + 1)] = stat["SolutionInfo"]["ObjVal"]

        customer_num = instance_name[3:5]
        lw = .7 if customer_num == '10' else (.5 if customer_num == '20' else 1)
        xs = [log(int(a), 10) for a in stat['gap_dic'].keys()]
        ys = list(stat['gap_dic'].values())
        zs = 0 if customer_num == '10' else (1 if customer_num == '20' else 2)

        ax.plot(xs, ys, zs=zs, zdir='y',
                color='dimgray', ls='-', linewidth=lw, alpha=.4, label="{} demand points".format(customer_num))
        ax.set_xticks([0, 1, 2, 3, 4])
        ax.set_xticks([0.5, 1.5, 2.5, 3.5], minor=True)
        ax.set_xticklabels(['$10^0$', '$10^1$', '$10^2$', '$10^3$', '$10^4$'])

    def legend_without_duplicate_labels(ax):
        handles, labels = ax.get_legend_handles_labels()
        unique = [(h, l) for i, (h, l) in enumerate(zip(handles, labels)) if l not in labels[:i]]
        ax.legend(*zip(*unique), fontsize=7, frameon=False)

    legend_without_duplicate_labels(ax)

    fig.savefig(
        FILE_DIR['figures'] + "gap_{}_3d.pdf".format(instance_type),
        bbox_inches='tight', transparent=True, pad_inches=0.05)


def _gap_curve(instance_type):
    # plt.style.use('grayscale')
    highlight_color = '#1f78b4'
    colors = itertools.cycle(['#a5acaf', '#60636a', '#414451'])
    fig, ax = plt.subplots(figsize=(4, 3))

    for demand_num in ['10', '20', '40']:
        # ax.set_title("{}".format("NLBBD-B\&C" if instance_type == "BAC" else "NLBBD"))
        ax.spines.top.set_visible(False)
        ax.spines.right.set_visible(False)
        ax.grid(alpha=0.3, ls='--')
        # x_axes
        ax.set_xlabel("Iteration")
        ax.set_xscale('log', base=10)
        ax.set_xlim(1, 10000)
        xticks = list(range(100, 1000, 100)) + list(range(1000, 10000, 1000))
        ax.set_xticks(xticks, minor=True)
        # y_axes
        ax.set_ylabel("Gap")
        ax.set_ylim(0, 1)
        ax.set_yticks([i / 10 for i in range(1, 10, 2)], minor=True)

        # draw lines
        for i, instance_name in enumerate(a for a in ALL_INSTANCES_NAME if a[3:5] == demand_num):
            stat = get_solution_stats(instance_name, instance_type)
            for key, value in stat['gap_dic'].items():
                stat['gap_dic'][key] = 1.0 if value > 1 else value
            iteration_time = max([int(a) for a in stat['lower_bound_dic'].keys()])
            if stat["SolutionInfo"]["Status"] == 2 and stat["gap_dic"][
                str(iteration_time)] != 0 and instance_type == 'BAC':
                stat["gap_dic"][str(iteration_time + 1)] = 0
                stat['best_upper_bound_dic'][str(iteration_time + 1)] = stat["SolutionInfo"]["ObjVal"]
                stat['lower_bound_dic'][str(iteration_time + 1)] = stat["SolutionInfo"]["ObjVal"]

            xs = [int(a) for a in stat['gap_dic'].keys()]
            ys = list(stat['gap_dic'].values())
            lw = .5 if demand_num == '40' else (.5 if demand_num == '20' else .3)
            ls = ':' if demand_num == '40' else '-'
            ax.plot(
                xs, ys,
                color='dimgrey', ls=ls, linewidth=lw, alpha=.6,
                label="{} affected communities".format(demand_num)
            )

        # draw shadow
        lines = []
        for i, instance_name in enumerate(a for a in ALL_INSTANCES_NAME if a[3:5] == demand_num):
            stat = get_solution_stats(instance_name, instance_type)
            for key, value in stat['gap_dic'].items():
                stat['gap_dic'][key] = 1.0 if value > 1 else value
            iteration_time = max([int(a) for a in stat['lower_bound_dic'].keys()])
            if stat["SolutionInfo"]["Status"] == 2 and stat["gap_dic"][
                str(iteration_time)] != 0 and instance_type == 'BAC':
                stat["gap_dic"][str(iteration_time + 1)] = 0
            line = list(stat['gap_dic'].values())
            lines.append(line)

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
        lower_limits = [min(line[i] if i < len(line) else min_ys[i] for line in lines) for i in range(max_iteration)]
        xs = [i for i in range(1, max_iteration + 1)]
        ax.fill_between(xs, upper_limits, lower_limits, zorder=0, alpha=.4, color=next(colors), edgecolor='none')

        # ax.plot(xs, upper_limits, c=highlight_color, lw=.5)
        # ax.plot(xs, lower_limits, c=highlight_color, lw=.5)

        def legend_without_duplicate_labels(ax):
            handles, labels = ax.get_legend_handles_labels()
            unique = [(h, l) for i, (h, l) in enumerate(zip(handles, labels)) if l not in labels[:i]]
            leg = ax.legend(*zip(*unique), fontsize=7, loc="lower left", framealpha=1)
            leg.get_frame().set_linewidth(0.0)

        legend_without_duplicate_labels(ax)

        fig.savefig(FILE_DIR['figures'] + "gap_{}.pdf".format(instance_type),
                    bbox_inches='tight', transparent=True, pad_inches=0.05)


def _r_gap_curve_subs(instance_type):
    fig, axs = plt.subplots(3, 1, figsize=(4, 5))

    for ax_id, demand_num in enumerate(['10', '20', '40']):
        ax = axs[ax_id]
        ax.set_title("$|D| = {}$".format(demand_num), fontsize=7, x=0.88, y=0.78, pad=0)
        ax.spines.top.set_visible(False)
        ax.spines.right.set_visible(False)
        ax.grid(alpha=0.3, ls='--')

        # x_axes
        ax.set_xlabel("Iteration")
        ax.set_xscale('log', base=10)
        ax.set_xlim(1, 10000)
        xticks = list(range(100, 1000, 100)) + list(range(1000, 10000, 1000))
        ax.set_xticks(xticks, minor=True)
        if ax_id in [0, 1]:
            ax.get_xaxis().get_label().set_visible(False)
            ax.get_xaxis().get_label().set_visible(False)
            ax.set_xticklabels([])
            ax.set_xticklabels([])

        # y_axes
        ax.set_ylabel("Gap")
        ax.set_ylim(0, 1)
        ax.set_yticks([0.1, 0.3, 0.5, 0.7, 0.9], minor=False)
        # ax.set_yticklabels(['0%', '25%', '50%', '75%', '100%'], minor=False)
        if ax_id in [0, 2]:
            axs[0].get_yaxis().get_label().set_visible(False)
            axs[2].get_yaxis().get_label().set_visible(False)

        ax.set_prop_cycle(lw=[0.5])

        # draw lines
        for i, instance_name in enumerate(a for a in ALL_INSTANCES_NAME if a[3:5] == demand_num):
            stat = get_solution_stats(instance_name, instance_type)
            for key, value in stat['gap_dic'].items():
                stat['gap_dic'][key] = 1.0 if value > 1 else value
            iteration_time = max([int(a) for a in stat['lower_bound_dic'].keys()])
            if stat["SolutionInfo"]["Status"] == 2 and stat["gap_dic"][
                str(iteration_time)] != 0 and instance_type == 'BAC':
                stat["gap_dic"][str(iteration_time + 1)] = 0
                stat['best_upper_bound_dic'][str(iteration_time + 1)] = stat["SolutionInfo"]["ObjVal"]
                stat['lower_bound_dic'][str(iteration_time + 1)] = stat["SolutionInfo"]["ObjVal"]

            xs = [int(a) for a in stat['gap_dic'].keys()]
            ys = list(stat['gap_dic'].values())
            ax.plot(
                xs, ys,
                color='silver',
                ls='-',
                # linewidth=0.5,
                alpha=0.9,
                # drawstyle="steps-mid",
                label="Instance"
            )

        # draw shadow
        lines = []
        for i, instance_name in enumerate(a for a in ALL_INSTANCES_NAME if a[3:5] == demand_num):
            stat = get_solution_stats(instance_name, instance_type)
            for key, value in stat['gap_dic'].items():
                stat['gap_dic'][key] = 1.0 if value > 1 else value
            iteration_time = max([int(a) for a in stat['lower_bound_dic'].keys()])
            if stat["SolutionInfo"]["Status"] == 2 and stat["gap_dic"][
                str(iteration_time)] != 0 and instance_type == 'BAC':
                stat["gap_dic"][str(iteration_time + 1)] = 0
            line = list(stat['gap_dic'].values())
            lines.append(line)

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
        lower_limits = [min(line[i] if i < len(line) else min_ys[i] for line in lines) for i in range(max_iteration)]
        mean_line = [[] for _ in range(max_iteration)]
        for line in lines:
            for i, value in enumerate(line):
                mean_line[i].append(value)
        mean_line = [sum(line) / len(line) for line in mean_line]
        xs = [i for i in range(1, max_iteration + 1)]
        colors = ['#a5acaf', '#60636a', '#414451']
        ax.fill_between(xs, upper_limits, lower_limits, zorder=0, alpha=.25, color=colors[0], edgecolor='none')
        ax.plot(xs, mean_line, color="k", lw=1, label="Mean")

    def legend_without_duplicate_labels(ax):
        handles, labels = ax.get_legend_handles_labels()
        unique = [(h, l) for i, (h, l) in enumerate(zip(handles, labels)) if l not in labels[:i]]
        leg = ax.legend(*zip(*unique), fontsize=7, loc="lower left", framealpha=0.5)
        leg.get_frame().set_linewidth(0.0)

    legend_without_duplicate_labels(axs[2])

    fig.subplots_adjust(hspace=0)
    fig.savefig(FILE_DIR['figures'] + "gap_{}_subs.pdf".format(instance_type),
                bbox_inches='tight', transparent=True, pad_inches=0.05)


def draw_gap_curve():
    for instance_type in ["LBBD", "BAC"]:
        # _gap_curve_d3(instance_type)
        # _gap_curve(instance_type)
        _r_gap_curve_subs(instance_type)


def _component(instance_name, instance_type):
    stat = get_solution_stats(instance_name, instance_type)
    res = [''] * 13
    if stat:
        runtime_dic = _get_runtime_dict(instance_name, instance_type)

        pmp_call_count = stat['PMP_call_count']
        pmp_runtime = sum(runtime_dic['pmp'].values())
        master_optimality_cut_count = sum(stat['PMP_optimality_cut_count_dic'].values())
        master_feasibility_cut_count = sum(stat['PMP_feasibility_cut_count_dic'].values())

        rasp_call_count = stat['RASP_call_count']
        rasp_runtime = sum(runtime_dic['rasp'].values())
        sub_optimality_cut_count = sum(
            [sum(a['RASP_optimality_cut_count_li']) for a in stat['RASP_stats_dic'].values()])
        sub_feasibility_cut_count = sum(
            [sum(a['RASP_feasibility_cut_count_li']) for a in stat['RASP_stats_dic'].values()])

        vdsp_call_count = stat['VDSP_call_count']
        vdsp_runtime = sum(runtime_dic['vdsp'].values())

        total_runtime = stat["SolutionInfo"]["Runtime"]
        if instance_type == "BAC":
            pmp_runtime = total_runtime - rasp_runtime - vdsp_runtime
        res = [
            pmp_call_count, pmp_runtime, pmp_runtime / total_runtime * 100, master_optimality_cut_count,
            master_feasibility_cut_count,
            rasp_call_count, rasp_runtime, rasp_runtime / total_runtime * 100, sub_optimality_cut_count,
            sub_feasibility_cut_count,
            vdsp_call_count, vdsp_runtime, vdsp_runtime / total_runtime * 100
        ]
    return res


def _tex_format(content):
    lines = ""
    pre_facility_num = None
    pre_demand_num = None

    for instance_row in content:
        instance_name = instance_row[0]
        facility_num = int(instance_name[:2])
        demand_num = int(instance_name[3:5])
        if facility_num != pre_facility_num:
            lines += "\midrule\n"
        if demand_num == 40 and pre_demand_num == 20:
            lines += "\cline{{2-{}}}\n".format(len(instance_row))
        pre_facility_num = facility_num
        pre_demand_num = demand_num

        instance_row = ["%.2f" % a if '.' in str(a) else str(a) for a in instance_row]
        # instance_row = ["\gradientcell{{{}}}{{0}}{{100}}{{snow}}{{silver}}{{100}}".format(a)
        #                 if i in [2,4,6,8,10,12] else a for i,a in enumerate(instance_row) ]
        lines += " & ".join(instance_row) + ' \\\\\n'

    print(lines)


def algorithm_component_table():
    for instance_type in ["LBBD", "BAC"]:
        instances_data = []
        instances_name = sorted(ALL_INSTANCES_NAME)
        for instance_name in instances_name:
            row = [instance_name.replace('_', '-')] + _component(instance_name, instance_type)
            instances_data.append(row)
        print(instance_type)
        _tex_format(instances_data)


def algorithm_component_table_in_one():
    instances_data = []
    instances_name = sorted(ALL_INSTANCES_NAME)
    for instance_name in instances_name:
        row = [instance_name.replace('_', '-')]
        for instance_type in ["LBBD", "BAC"]:
            ins_row = _component(instance_name, instance_type)
            ins_row = [ins_row[1], ins_row[2], ins_row[6], ins_row[7], ins_row[11], ins_row[12]]
            row += ins_row
        instances_data.append(row)
    _tex_format(instances_data)


def visualize_lateral_trans(instance_name):
    demand_points_style = {
        'marker': 'o', 's': 40, 'linewidth': 1,
        'c': 'w', 'edgecolors': 'k',
        'zorder': 2, 'alpha': 1
    }
    isolated_demand_style = {
        'marker': 'X', 's': 40, 'linewidth': 1,
        'c': 'whitesmoke', 'edgecolors': 'k',
        'zorder': 2, 'alpha': 1
    }
    facility_points_style = {
        'marker': 'P', 's': 80, 'linewidth': 1,
        'c': 'r', 'edgecolors': 'k',
        'alpha': 1, 'zorder': 3
    }
    I = Instance(instance_name, "LT", **{"allow_lt": True, "f_cap_times": 1.2})
    LT = GurobiSolution(instance_name, "LT", **{"allow_lt": True, "f_cap_times": 1.2})
    DT = GurobiSolution(instance_name, "LT", **{"allow_lt": False, "f_cap_times": 1.2})
    print(LT.facility_stock, sum(LT.facility_stock.values()), DT.facility_stock, sum(DT.facility_stock.values()))
    print(I.facility_types_info)

    scenario_map = {0: "IV", 2: "III", 3: "II", 1: "I"}
    loc_in_axs_map = {0: 0, 1: 3, 2: 1, 3: 2}

    fig, ax = plt.subplots(2, 4, figsize=(9, 4.5), sharex=True, sharey=True)
    for j, GS in enumerate([LT, DT]):
        for s in [0, 2, 3, 1]:
            row_id = loc_in_axs_map[s]

            isolated_demand = I.isolated_demand_in_scenario[s]
            non_isolated_demand = list(set(I.demand_set) - set(isolated_demand))
            non_isolated_xs = [I.points_loc[i][0] for i in non_isolated_demand]
            non_isolated_ys = [I.points_loc[i][1] for i in non_isolated_demand]
            label = "非孤立点" if LANG_ZH else "Non isolated community"
            ax[j][row_id].scatter(non_isolated_xs, non_isolated_ys, **demand_points_style, label=label)

            isolated_xs = [I.points_loc[i][0] for i in isolated_demand]
            isolated_ys = [I.points_loc[i][1] for i in isolated_demand]
            label = "孤立点" if LANG_ZH else "Isolated community"
            ax[j][row_id].scatter(isolated_xs, isolated_ys, **isolated_demand_style, label=label)

            facility_xs = [I.points_loc[i][0] for i in I.facility_set]
            facility_ys = [I.points_loc[i][1] for i in I.facility_set]
            label = "援助设施" if LANG_ZH else "Relief facility"
            ax[j][row_id].scatter(facility_xs, facility_ys, **facility_points_style, label=label)

            # [s][i][h]
            routes = GS.get_scenarios_routes()[s]
            for i in routes:
                for h in routes[i]:
                    is_special_vehicle = h in I.special_vehicle_types
                    for r in routes[i][h]:
                        # print(i, h, is_special_vehicle, r)
                        verts = [I.points_loc[j] for j in r]
                        codes = [Path.MOVETO] + [Path.LINETO for _ in range(len(verts) - 1)]
                        path = Path(verts, codes)
                        patch = patches.PathPatch(
                            path, facecolor='none', lw=1, linestyle="dotted" if is_special_vehicle else 'solid')
                        ax[j][row_id].add_patch(patch)

            if j == 0:
                ax[j][row_id].set_title("$s={}$".format(scenario_map[s]))
            if row_id == 0:
                if LANG_ZH:
                    ylabel = "横向转运" if j == 0 else "直接运输"
                else:
                    ylabel = "HC" if j == 0 else "DT"
                ax[j][row_id].set_ylabel(ylabel)
            ax[j][row_id].set_xlim(-100, 1100)
            ax[j][row_id].set_ylim(-100, 1100)
            ax[j][row_id].set_xticks([0, 500, 1000])
            ax[j][row_id].set_yticks([0, 500, 1000])
            ax[j][row_id].set_xticklabels([])
            ax[j][row_id].set_yticklabels([])
            ax[j][row_id].grid(which='major', alpha=0.2, ls='--', zorder=-2, c='dimgrey')

            ax[j][row_id].xaxis.set_ticks_position('none')
            ax[j][row_id].yaxis.set_ticks_position('none')
            ax[j][row_id].spines.top.set_visible(False)
            ax[j][row_id].spines.right.set_visible(False)
            ax[j][row_id].spines.bottom.set_visible(False)
            ax[j][row_id].spines.left.set_visible(False)

            # texts = [ax[j][row_id].text(I.points_loc[n][0], I.points_loc[n][1], n, fontsize=10, ha = 'right')
            #          for n in I.facility_set]
            # adjust_text(texts, ax = ax[j][row_id])
            pre = "\(\sum_{i \in T} \mathcal{R}_{is} =\)" if not LANG_ZH else "路径成本: "
            ax[j][row_id].text(500, -50, pre + str(round(GS.routing_cost[s], 2)),
                               ha="center", va="bottom", backgroundcolor="white")

    lines_labels = [[ax.get_legend_handles_labels() for ax in fig.axes][0]]
    print(lines_labels)
    lines, labels = [sum(lol, []) for lol in zip(*lines_labels)]
    leg = fig.legend(lines, labels, ncol=3, loc='lower center', framealpha=0)
    # leg.get_frame().set_linewidth(0.0)

    plt.subplots_adjust(wspace=0.1, hspace=0.1)
    plt.savefig(
        FILE_DIR['figures'] + 'HC_' + instance_name + '.pdf',
        bbox_inches='tight', transparent=True, pad_inches=0.05)


def visualize_lateral_trans_s():
    for instance_name in [
        "05_10_04_0_0",
        # "05_10_04_0_1",
    ]:
        visualize_lateral_trans(instance_name)


def visualize_vehicle_routing(instance_name):
    demand_points_style = {
        'marker': 'o', 's': 40, 'linewidth': 1,
        'c': 'w', 'edgecolors': 'k',
        'zorder': 20, 'alpha': 1
    }
    isolated_demand_style = {
        'marker': 'X', 's': 40, 'linewidth': 1,
        'c': 'whitesmoke', 'edgecolors': 'k',
        'zorder': 20, 'alpha': 1
    }
    facility_points_style = {
        'marker': 'P', 's': 80, 'linewidth': 1,
        'c': 'r', 'edgecolors': 'k',
        'alpha': 1, 'zorder': 21
    }
    info = {"p_type": "ROUTE", "ut_cost": 6.5874, "ut_times": 1.0, "fc_times": 1, "vc_times": 1.0}
    I = Instance(instance_name)
    A = GurobiSolution(instance_name, "NO_VRP", **info)
    B = GurobiSolution(instance_name, "LBBD")
    print(A.facility_stock, sum(A.facility_stock.values()))
    print(B.facility_stock, sum(B.facility_stock.values()))
    print(I.facility_types_info)

    scenario_map = {0: "IV", 1: "I"}
    loc_in_axs_map = {0: 1, 1: 2}

    fig, ax = plt.subplots(2, 3, figsize=(6.75, 4.5), sharex=True, sharey=True)
    for j, GS in enumerate([A, B]):
        row_id = 0
        non_isolated_xs = [I.points_loc[i][0] for i in I.demand_set]
        non_isolated_ys = [I.points_loc[i][1] for i in I.demand_set]
        label = "非孤立点" if LANG_ZH else "Non isolated community"
        ax[j][row_id].scatter(non_isolated_xs, non_isolated_ys, **demand_points_style, label=label)
        opened_facility = GS.opened_facility
        facility_xs = [I.points_loc[i][0] for i in opened_facility]
        facility_ys = [I.points_loc[i][1] for i in opened_facility]
        label = "开放的援助设施" if LANG_ZH else "Opened facility"
        ax[j][row_id].scatter(facility_xs, facility_ys, **facility_points_style, label=label)
        not_opened_facility = list(set(I.facility_set) - set(opened_facility))
        facility_xs = [I.points_loc[i][0] for i in not_opened_facility]
        facility_ys = [I.points_loc[i][1] for i in not_opened_facility]
        facility_points_style['c'] = 'white'
        label = "未开放的援助设施" if LANG_ZH else "Not opened facility"
        ax[j][row_id].scatter(facility_xs, facility_ys, **facility_points_style, label=label)
        facility_points_style['c'] = 'r'
        print(GS.facility_assigned_demand)
        for i in GS.facility_assigned_demand:
            for k in GS.facility_assigned_demand[i]:
                verts = [I.points_loc[i], I.points_loc[k]]
                print(verts)
                path = Path(verts)
                patch = patches.PathPatch(path, facecolor='none', lw=1, linestyle=(0, (1, 1)))
                ax[j][row_id].add_patch(patch)
        if j == 0:
            title = "选址决策" if LANG_ZH else "Location decisions"
            ax[j][row_id].set_title(title)
        ax[j][row_id].set_ylabel("TS" if j == 0 else "RT")
        ax[j][row_id].set_xlim(-100, 1100)
        ax[j][row_id].set_ylim(-100, 1100)
        ax[j][row_id].set_xticks([0, 500, 1000])
        ax[j][row_id].set_yticks([0, 500, 1000])
        ax[j][row_id].set_xticklabels([])
        ax[j][row_id].set_yticklabels([])
        ax[j][row_id].grid(which='major', alpha=0.2, ls='--', zorder=-2, c='dimgrey')
        cs = ['#a5acaf', '#60636a', '#414451']
        # rect = patches.Rectangle((-100, -100), 1200, 1200, linewidth=0, facecolor=cs[0], alpha=.25)
        rect = patches.Rectangle((-100, -100), 1200, 1200, linewidth=1, edgecolor="black", facecolor="dimgrey",
                                 alpha=.1)
        ax[j][row_id].add_patch(rect)

        ax[j][row_id].xaxis.set_ticks_position('none')
        ax[j][row_id].yaxis.set_ticks_position('none')
        ax[j][row_id].spines.top.set_visible(False)
        ax[j][row_id].spines.right.set_visible(False)
        ax[j][row_id].spines.bottom.set_visible(False)
        ax[j][row_id].spines.left.set_visible(False)

        for s in [0, 1]:
            row_id = loc_in_axs_map[s]

            isolated_demand = I.isolated_demand_in_scenario[s]
            non_isolated_demand = list(set(I.demand_set) - set(isolated_demand))
            non_isolated_xs = [I.points_loc[i][0] for i in non_isolated_demand]
            non_isolated_ys = [I.points_loc[i][1] for i in non_isolated_demand]
            ax[j][row_id].scatter(non_isolated_xs, non_isolated_ys, **demand_points_style,
                                  label="Non isolated community")

            isolated_xs = [I.points_loc[i][0] for i in isolated_demand]
            isolated_ys = [I.points_loc[i][1] for i in isolated_demand]
            ax[j][row_id].scatter(isolated_xs, isolated_ys, **isolated_demand_style, label="Isolated community")

            opened_facility = GS.opened_facility
            facility_xs = [I.points_loc[i][0] for i in opened_facility]
            facility_ys = [I.points_loc[i][1] for i in opened_facility]
            ax[j][row_id].scatter(facility_xs, facility_ys, **facility_points_style, label="Opened facility")

            not_opened_facility = list(set(I.facility_set) - set(opened_facility))
            facility_xs = [I.points_loc[i][0] for i in not_opened_facility]
            facility_ys = [I.points_loc[i][1] for i in not_opened_facility]
            facility_points_style['c'] = 'white'
            ax[j][row_id].scatter(facility_xs, facility_ys, **facility_points_style, label="Not opened facility")
            facility_points_style['c'] = 'r'

            # [s][i][h]
            routes = GS.get_scenarios_routes()[s]
            for i in routes:
                for h in routes[i]:
                    is_special_vehicle = h in I.special_vehicle_types
                    for r in routes[i][h]:
                        # print(i, h, is_special_vehicle, r)
                        verts = [I.points_loc[j] for j in r]
                        codes = [Path.MOVETO] + [Path.LINETO for _ in range(len(verts) - 1)]
                        path = Path(verts, codes)
                        patch = patches.PathPatch(
                            path, facecolor='none', lw=1, zorder=19,
                            linestyle=(0, (1, 1)) if is_special_vehicle else 'solid')
                        ax[j][row_id].add_patch(patch)

            if j == 0:
                ax[j][row_id].set_title("$s={}$".format(scenario_map[s]))
            if LANG_ZH:
                ylabel = "运输问题" if j == 0 else "路径优化问题"
            else:
                ylabel = "TS" if j == 0 else "RT"
            ax[j][0].set_ylabel(ylabel)
            ax[j][row_id].set_xlim(-100, 1100)
            ax[j][row_id].set_ylim(-100, 1100)
            ax[j][row_id].set_xticks([0, 500, 1000])
            ax[j][row_id].set_yticks([0, 500, 1000])
            # ax[j][row_id].set_xticklabels([])
            # ax[j][row_id].set_yticklabels([])
            ax[j][row_id].grid(which='major', alpha=0.2, ls='--', zorder=-2, c='dimgrey')

            ax[j][row_id].xaxis.set_ticks_position('none')
            ax[j][row_id].yaxis.set_ticks_position('none')
            ax[j][row_id].spines.top.set_visible(False)
            ax[j][row_id].spines.right.set_visible(False)
            ax[j][row_id].spines.bottom.set_visible(False)
            ax[j][row_id].spines.left.set_visible(False)

            pre = "\(\sum_{i \in T} \mathcal{R}_{is} =\)" if not LANG_ZH else "路径成本: "
            ax[j][row_id].text(
                500, -110, pre + str(round(GS.routing_cost[s], 2)),
                ha="center", va="bottom", backgroundcolor='white')

    lines_labels = [[ax.get_legend_handles_labels() for ax in fig.axes][0]]
    print(lines_labels)
    lines, labels = [sum(lol, []) for lol in zip(*lines_labels)]
    leg = fig.legend(lines, labels, ncol=3, loc='lower center', framealpha=0)

    plt.subplots_adjust(wspace=0.1, hspace=0.1)
    plt.savefig(
        FILE_DIR['figures'] + 'VRP_' + instance_name + '.pdf',
        bbox_inches='tight', transparent=True, pad_inches=0.05)


def visualize_vehicle_routing_s():
    for instance_name in [
        # "03_10_02_0_0",
        "07_10_02_0_0"
    ]:
        visualize_vehicle_routing(instance_name)


def main():
    # draw_route()

    visualize_instances()

    # algorithm_comparison_table()
    # algorithm_component_table()
    # algorithm_component_table_in_one()

    # draw_convergence_curve()
    # draw_gap_curve()

    # draw_pareto_frontiers(get_pareto_frontiers_data())
    # vehicle_routing_table(get_vehicle_routing_data(1.0, transportation_cost_estimate(1.0)))
    # lateral_trans_table(get_exp_lateral_trans_data())
    # stochasticity_table(get_expected_value_data(), get_wait_and_see_data())

    # from rich import print
    # print(get_pareto_frontiers_data())

    visualize_lateral_trans_s()
    visualize_vehicle_routing_s()


if __name__ == '__main__':
    main()
