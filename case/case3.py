import os
from datetime import datetime

import networkx as nx
import numpy as np
from termcolor import cprint

# 导入自定义模块
from mrta import milp, milp_suf, poset, weighted_ts, weighted_ts_suffix
from mrta.buchi_parse import Buchi
from mrta.GMAPP import compute_path_cost, mapp, return_to_initial
from mrta.task import Task
from mrta.util import create_parser, print_red_on_cyan
from mrta.timer import start_timer, log_time, show_summary, get_elapsed_time
from mrta.vis import vis
from mrta.workspace_case3 import Workspace

# 定义 'data' 文件夹的路径
data_folder_path = "./data"

# 检查文件夹是否存在
if not os.path.exists(data_folder_path):
    # 如果文件夹不存在，则创建它
    os.makedirs(data_folder_path)

# ---------------------------- LTL-MRTA 参数解析 ---------------------------
# 创建命令行参数解析器
parser = create_parser()
# 解析命令行参数
args = parser.parse_known_args()[0]


def ltl_mrta():
    """
    LTL-MRTA（Linear Temporal Logic for Multi-Robot Task Allocation）主函数。
    该函数实现了从LTL任务规约到多机器人路径规划的完整流程。
    """
    # 启动全局计时器
    start_timer("LTL-MRTA started")

    # 初始化工作空间，包含机器人、区域等环境信息
    workspace = Workspace(args.robot)

    # 备份初始机器人位置
    type_robot_location = workspace.type_robot_location.copy()
    # 从命令行参数获取配置
    loop = args.loop  # 是否执行循环任务（suffix part）
    one_time = args.only_first  # 是否只找一个解
    draw = args.vis  # 是否可视化结果
    show = args.print  # 是否打印详细信息
    partial_or_full = args.partial_or_full  # 路径规划是部分还是全部

    # 初始化用于记录最优解的变量
    best_cost = np.inf  # 最优成本
    cost = []  # 记录每次找到的解的成本
    time_record = []  # 记录每次找到的解的运行时间
    horizon_record = []  # 记录每次找到的解的路径长度
    number_of_paths = 0  # 期望找到的路径数量
    best_path = dict()  # 最优路径

    # --------------- 构建布奇自动机 (Buchi Automaton) ----------------
    # 从LTL公式构建任务
    task = Task(args.case, args.robot)
    # 初始化布奇自动机对象
    buchi = Buchi(task, workspace)

    # 构建布奇自动机图
    buchi.construct_buchi_graph()
    if show:
        log_time("构建Buchi自动机完成")

    # 获取所有 (初始状态, 接受状态) 对
    init_acpt = buchi.get_init_accept()

    # 遍历所有 (初始状态, 接受状态) 对
    for pair, _ in init_acpt:
        # workspace.type_robot_location = type_robot_location.copy()
        # workspace.update_after_prefix()
        # buchi.atomic_prop = workspace.atomic_prop
        # buchi.regions = workspace.regions

        # 获取当前处理的初始状态和接受状态
        init_state, accept_state = pair[0], pair[1]

        # ======================================= 前缀部分 (Prefix Part) =================================================#
        #                                                                                                      #
        #                                                                                                      #
        #                                                                                                      #
        # ======================================= 前缀部分 (Prefix Part) =================================================#

        # ----------------- 推断偏序集 (Poset) -----------------------
        # 从Buchi自动机中获取从初始状态到接受状态的子图和路径
        pruned_subgraph, unpruned_subgraph, paths = buchi.get_subgraph(init_state, accept_state, "prefix")
        # 获取边到元素以及元素到边的映射
        edge2element, element2edge = buchi.get_element(pruned_subgraph)
        # 如果没有元素（即可行的任务转换），则跳过
        if not edge2element:
            continue

        # 将元素组件映射到标签
        element_component2label = buchi.element2label2eccl(element2edge, pruned_subgraph)

        # 将路径映射为元素序列，并构建哈斯图
        hasse_graphs = buchi.map_path_to_element_sequence(edge2element, paths)

        # 遍历所有可能的偏序集（由不同的路径产生）
        for _, poset_relation, pos, hasse_diagram in hasse_graphs:
            if show:
                print_red_on_cyan("================ prefix part ================")

            # ----------------- 在后缀不成功时恢复状态 -----------------
            # 每次处理新的偏序集时，都重置工作空间和Buchi自动机的状态
            workspace.type_robot_location = type_robot_location.copy()
            workspace.update_after_prefix()
            buchi.atomic_prop = workspace.atomic_prop
            buchi.regions = workspace.regions

            # 将元素映射到负责执行的机器人
            # eccl means element-component-clause-literal
            robot2eccl = poset.element2robot2eccl(pos, element2edge, pruned_subgraph)
            if show:
                log_time("构建偏序集（poset）完成")

            # 打印偏序关系
            if show:
                print("偏序关系：")
                for order in poset_relation:
                    print(
                        pruned_subgraph.edges[element2edge[order[0]]]["formula"],
                        " -> ",
                        pruned_subgraph.edges[element2edge[order[1]]]["formula"],
                    )
                print("----------------------------------------------")

            # 从偏序关系中计算不可比、更大、更小和严格更大的元素关系
            (
                incomparable_element,
                larger_element,
                smaller_element,
                strict_larger_element,
            ) = poset.incomparable_larger(pos, poset_relation, hasse_diagram)

            # --------------- 构建路由图 (Routing Graph) ---------------
            # 构建加权迁移系统（Weighted Transition System）的节点集
            (
                init_type_robot_node,
                element_component_clause_literal_node,
                node_location_type_component_element,
                num_nodes,
            ) = weighted_ts.construct_node_set(pos, element2edge, pruned_subgraph, workspace.type_robot_label)

            # 构建加权迁移系统的边集
            edge_set = weighted_ts.construct_edge_set(
                pos,
                element_component_clause_literal_node,
                element2edge,
                pruned_subgraph,
                element_component2label,
                init_type_robot_node,
                incomparable_element,
                strict_larger_element,
                larger_element,
                buchi.imply,
            )

            # 构建加权迁移系统图
            ts = weighted_ts.construct_graph(num_nodes, node_location_type_component_element, edge_set, workspace.p2p)

            if show:
                log_time("MILP前时间")

            # --------------------- MILP (Mixed-Integer Linear Programming) -------------------------
            # 找到哈斯图中的最大元（没有入度的节点）
            maximal_element = [node for node in hasse_diagram.nodes() if hasse_diagram.in_degree(node) == 0]

            # 构建并求解MILP问题，得到前缀路径的航点和时间
            (
                robot_waypoint_pre,
                robot_time_pre,
                id2robots,
                robot_label_pre,
                robot_waypoint_axis,
                robot_time_axis,
                time_axis,
                acpt_run,
            ) = milp.construct_milp_constraint(
                ts,
                workspace.type_num,
                pos,
                pruned_subgraph,
                element2edge,
                element_component_clause_literal_node,
                poset_relation,
                init_type_robot_node,
                strict_larger_element,
                incomparable_element,
                larger_element,
                robot2eccl,
                init_state,
                buchi,
                maximal_element,
                show,
            )
            # 如果MILP无解，则跳过
            if not robot_waypoint_pre:
                continue

            # 删除未参与任务的机器人
            for robot, time in list(robot_time_pre.items()):
                #  delete such robots that did not participate (the initial location of robots may just satisfies)
                if time[-1] == 0 and len(time) == 1:
                    del robot_time_pre[robot]
                    del robot_waypoint_pre[robot]

            if show:
                print("----------------- Prefix Waypoints and Times -----------------")
                for type_robot, waypoint in robot_waypoint_pre.items():
                    print(type_robot, " : ", waypoint)
                    print(type_robot, " : ", robot_time_pre[type_robot])
                    print(type_robot, " : ", robot_label_pre[type_robot])
                print("----------------------------------------------")

                print("time axis: ", time_axis)

            # 删除未参与任务的机器人（基于时间轴）
            for robot, time in list(robot_time_axis.items()):
                #  delete such robots that did not participate (the initial location of robots may just satisfies)
                if not time:
                    del robot_time_axis[robot]
                    del robot_waypoint_axis[robot]

            if show:
                for type_robot, waypoint in robot_waypoint_axis.items():
                    print(type_robot, " : ", waypoint)
                    print(type_robot, " : ", robot_time_axis[type_robot])

                print("------------------- Acceptance Run (Prefix) --------------------")

                for stage in acpt_run:
                    print(stage)
                print("----------------------------------------------")

            # --------------------- GMRPP (Generalized Multiple Rural Postmen Problem) -------------------------
            # 使用MAPP（Multi-Agent Path Planning）算法为每个机器人规划具体路径
            robot_path_pre = mapp(
                workspace,
                buchi,
                acpt_run,
                robot_waypoint_axis,
                robot_time_axis,
                "simultaneous",
                show,
                partial_or_full,
            )

            # vis(workspace, robot_path_pre, {robot: [len(path)] * 2 for robot, path in robot_path_pre.items()},
            #     [])
            # ----------------- 检查前缀路径的最终位置是否满足接受状态 -----------------
            # 更新机器人的位置
            workspace.type_robot_location = {robot: path[-1] for robot, path in robot_path_pre.items()}
            workspace.update_after_prefix(loop)
            buchi.atomic_prop = workspace.atomic_prop
            buchi.regions = workspace.regions

            # 获取接受状态运行的最后一个子任务
            last_subtask = acpt_run[-1]
            # 恢复初始状态的自环属性（如果在构建Buchi时被移除）
            if buchi.remove_init_attr:
                nx.set_node_attributes(pruned_subgraph, {init_state: buchi.remove_init_attr})
            # 检查最终位置是否满足接受状态的自环条件
            if buchi.ap_sat_label(
                pruned_subgraph.nodes[accept_state]["label"],
                pruned_subgraph.nodes[accept_state]["neg_label"],
            ):
                if show:
                    log_time("前缀部分完成(最终位置满足接受状态自环条件)")
                # 计算前缀路径的成本
                cost_pre = compute_path_cost(robot_path_pre)
                cost.append(cost_pre)
                time_record.append(get_elapsed_time())
                horizon_record.append(len(robot_path_pre[(1, 0)]))
                # 更新最优解
                if best_cost >= cost_pre:
                    best_path = robot_path_pre
                    best_cost = cost_pre
                print("The total cost of the found path (loop) is: ")
                print(f"best total cost: {best_cost}")
                print(f"cost per solution (pre, suf): {cost}")
                print(f"runtimes up to now {time_record}")
                print(f"horizon per solution (pre, suf) {horizon_record}")
                if show:
                    print_red_on_cyan(task.formula)
                    print_red_on_cyan(
                        [
                            init_state,
                            accept_state,
                            buchi.size,
                            [
                                pruned_subgraph.number_of_nodes(),
                                pruned_subgraph.number_of_edges(),
                            ],
                            "A path is found for the case where the accepting state has a self-loop",
                        ]
                    )
                # 可视化结果
                if draw:
                    vis(
                        args.case,
                        workspace,
                        robot_path_pre,
                        {robot: [len(path)] * 2 for robot, path in robot_path_pre.items()},
                        [],
                    )
                # 如果只需要一个解，则返回
                if one_time:
                    return cost, best_path
                elif len(cost) > number_of_paths:
                    return cost, best_path
                else:
                    continue

            log_time("前缀部分完成(最终位置不满足接受状态自环条件，接下来是后缀部分)")
            # ======================================= 后缀部分 (Suffix Part) =================================================#
            #                                                                                                      #
            #                                                                                                      #
            #                                                                                                      #
            # ======================================= 后缀部分 (Suffix Part) =================================================#

            # ----------------- 推断后缀部分的偏序集 -----------------------

            # 获取从接受状态到其自身的子图，用于构建后缀路径
            pruned_subgraph_suf, unpruned_subgraph_suf, paths_suf = buchi.get_subgraph(
                accept_state, accept_state, "suffix", last_subtask
            )
            # 如果没有后缀图（因为前缀部分的最终位置不满足接受状态的出边），则跳过
            if not pruned_subgraph_suf:
                continue

            # 如果没有路径（因为蕴含关系不成立），则跳过
            if not paths_suf:
                continue

            # 获取后缀图的边-元素映射
            edge2element_suf, element2edge_suf = buchi.get_element(pruned_subgraph_suf)

            # 将元素组件映射到标签
            element_component2label_suf = buchi.element2label2eccl(element2edge_suf, pruned_subgraph_suf)

            # 将路径映射为元素序列，并构建哈斯图
            hasse_graphs_suf = buchi.map_path_to_element_sequence(edge2element_suf, paths_suf)

            # 遍历所有可能的后缀偏序集
            for _, poset_relation_suf, pos_suf, hasse_diagram_suf in hasse_graphs_suf:
                if show:
                    print_red_on_cyan("================ suffix part ================")

                    # 打印后缀偏序关系
                    for order_suf in poset_relation_suf:
                        print(
                            pruned_subgraph_suf.edges[element2edge_suf[order_suf[0]]]["formula"],
                            " -> ",
                            pruned_subgraph_suf.edges[element2edge_suf[order_suf[1]]]["formula"],
                        )
                    print("----------------------------------------------")

                # 将元素映射到负责执行的机器人
                robot2eccl_suf = poset.element2robot2eccl(pos_suf, element2edge_suf, pruned_subgraph_suf)

                # 计算不可比、更大、更小和严格更大的元素关系
                (
                    incomparable_element_suf,
                    larger_element_suf,
                    smaller_element_suf,
                    strict_larger_element_suf,
                ) = poset.incomparable_larger(pos_suf, poset_relation_suf, hasse_diagram_suf)

                # --------------- 构建后缀部分的路由图 ---------------
                # 找到哈斯图中的最小元（没有出度的节点）
                minimal_element_suf = [
                    node for node in hasse_diagram_suf.nodes() if hasse_diagram_suf.out_degree(node) == 0
                ]
                # 构建后缀加权迁移系统的节点集
                (
                    init_type_robot_node_suf,
                    element_component_clause_literal_node_suf,
                    node_location_type_component_element_suf,
                    num_nodes_suf,
                    final_element_type_robot_node,
                ) = weighted_ts_suffix.construct_node_set(
                    pos_suf,
                    element2edge_suf,
                    pruned_subgraph_suf,
                    workspace.type_robot_label,
                    minimal_element_suf,
                    last_subtask,
                    loop,
                )

                # 构建后缀加权迁移系统的边集
                edge_set_suf = weighted_ts_suffix.construct_edge_set(
                    pos_suf,
                    element_component_clause_literal_node_suf,
                    element2edge_suf,
                    pruned_subgraph_suf,
                    element_component2label_suf,
                    init_type_robot_node_suf,
                    incomparable_element_suf,
                    strict_larger_element_suf,
                    larger_element_suf,
                    buchi.imply,
                    minimal_element_suf,
                    final_element_type_robot_node,
                )

                # 构建后缀加权迁移系统图
                ts_suf = weighted_ts_suffix.construct_graph(
                    num_nodes_suf,
                    node_location_type_component_element_suf,
                    edge_set_suf,
                    workspace.p2p,
                )

                # --------------------- MILP (后缀部分) -------------------------
                # 找到哈斯图中的最大元
                maximal_element_suf = [
                    node for node in hasse_diagram_suf.nodes() if hasse_diagram_suf.in_degree(node) == 0
                ]

                # 构建并求解后缀部分的MILP问题
                (
                    robot_waypoint_suf,
                    robot_time_suf,
                    _,
                    robot_label_suf,
                    robot_waypoint_axis_suf,
                    robot_time_axis_suf,
                    time_axis_suf,
                    acpt_run_suf,
                ) = milp_suf.construct_milp_constraint(
                    ts_suf,
                    workspace.type_num,
                    pos_suf,
                    pruned_subgraph_suf,
                    element2edge_suf,
                    element_component_clause_literal_node_suf,
                    poset_relation_suf,
                    init_type_robot_node_suf,
                    strict_larger_element_suf,
                    incomparable_element_suf,
                    larger_element_suf,
                    robot2eccl_suf,
                    id2robots,
                    accept_state,
                    buchi,
                    minimal_element_suf,
                    final_element_type_robot_node,
                    workspace.type_robot_label,
                    maximal_element_suf,
                    last_subtask,
                    show,
                    loop,
                )
                # 如果MILP无解，则跳过
                if not robot_waypoint_suf:
                    continue

                # 删除未参与任务的机器人
                for robot, time in list(robot_time_suf.items()):
                    # delete such robots that did not participate (the initial location of robots may just satisfies)
                    if time[-1] == 0 and len(time) == 1:
                        del robot_time_suf[robot]
                        del robot_waypoint_suf[robot]
                if show:
                    print("----------------- Suffix Waypoints and Times -----------------")
                    for type_robot, waypoint in robot_waypoint_suf.items():
                        print(type_robot, " : ", waypoint)
                        print(type_robot, " : ", robot_time_suf[type_robot])
                        print(type_robot, " : ", robot_label_suf[type_robot])
                    print("----------------------------------------------")

                    print("time axis: ", time_axis_suf)

                # 删除未参与任务的机器人（基于时间轴）
                for robot, time in list(robot_time_axis_suf.items()):
                    # delete such robots that did not participate (the initial location of robots may just satisfies)
                    if not time:
                        del robot_time_axis_suf[robot]
                        del robot_waypoint_axis_suf[robot]
                if show:
                    for type_robot, waypoint in robot_waypoint_axis_suf.items():
                        print(type_robot, " : ", waypoint)
                        print(type_robot, " : ", robot_time_axis_suf[type_robot])

                    print("------------------- Acceptance Run (Suffix) --------------------")

                    for stage in acpt_run_suf:
                        print(stage)
                    print("----------------------------------------------")

                # --------------------- GMRPP (后缀部分) -------------------------
                # 为后缀路径规划具体路径
                robot_path_suf = mapp(
                    workspace,
                    buchi,
                    acpt_run_suf,
                    robot_waypoint_axis_suf,
                    robot_time_axis_suf,
                    "simultaneous",
                    show,
                    partial_or_full,
                )

                # 如果不是循环任务，则规划返回初始位置的路径
                if not loop:
                    horizon = workspace.longest_time(
                        {robot: path[-1] for robot, path in robot_path_suf.items()},
                        workspace.type_robot_location,
                    )

                    acpt_run_suf = {
                        "subtask": "return",
                        "time_element": [horizon, -1],
                        "essential_robot_edge": {
                            label: [type_robot] for type_robot, label in workspace.type_robot_label.items()
                        },
                        "essential_clause_edge": last_subtask["essential_clause_edge"],
                        "neg_edge": last_subtask["neg_edge"],
                        "essential_robot_vertex": last_subtask["essential_robot_edge"],
                        "neg_vertex": last_subtask["neg_edge"],
                    }
                    robot_path_return = return_to_initial(
                        workspace,
                        acpt_run_suf,
                        {robot: path[-1] for robot, path in robot_path_suf.items()},
                    )
                    for robot, path in robot_path_suf.items():
                        path += robot_path_return[robot][1:]

                # 合并前缀和后缀路径
                robot_path = {
                    robot: path + robot_path_suf[robot][1:] + robot_path_suf[robot][1:]
                    for robot, path in robot_path_pre.items()
                }

                # 计算前缀和后缀的成本
                cost_pre = compute_path_cost(robot_path_pre)
                cos_suf = compute_path_cost(robot_path_suf)
                if show:
                    log_time("前后缀都完成")

                # 记录成本、时间和路径长度
                cost.append((cost_pre, cos_suf))
                time_record.append(get_elapsed_time())
                horizon_record.append((len(robot_path_pre[(1, 0)]), len(robot_path_suf[(1, 0)])))
                # 更新最优解
                if best_cost >= cost_pre + cos_suf:
                    best_path = robot_path
                    best_cost = cost_pre + cos_suf
                print("The total cost of the found path (loop) is: ")
                print(f"best total cost: {best_cost}")
                print(f"cost per solution (pre, suf): {cost}")
                print(f"runtimes up to now {time_record}")
                print(f"horizon per solution (pre, suf) {horizon_record}")
                if show:
                    print_red_on_cyan(task.formula)
                    print_red_on_cyan(
                        [
                            init_state,
                            accept_state,
                            buchi.size,
                            (
                                [
                                    pruned_subgraph.number_of_nodes(),
                                    pruned_subgraph.number_of_edges(),
                                ],
                                [
                                    pruned_subgraph_suf.number_of_nodes(),
                                    pruned_subgraph_suf.number_of_edges(),
                                ],
                            ),
                            "A path is found for the case where the accepting state does not have a self-loop",
                        ]
                    )
                # 可视化结果
                if draw:
                    vis(
                        args.case,
                        workspace,
                        robot_path,
                        {robot: [len(path)] * 2 for robot, path in robot_path.items()},
                        [],
                    )
                log_time("LTL-MRTA completed")
                show_summary()
                # 如果只需要一个解，则返回
                if one_time:
                    return
                if len(cost) > number_of_paths:
                    return
    return cost


if __name__ == "__main__":
    ltl_mrta()
