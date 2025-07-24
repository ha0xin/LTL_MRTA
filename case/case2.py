import os
import pickle
from datetime import datetime

import networkx as nx
import numpy as np

from mrta import milp, milp_suf, poset, weighted_ts, weighted_ts_suffix
from mrta.buchi_parse import Buchi
from mrta.GMAPP import compute_path_cost, mapp, return_to_initial
from mrta.task import Task
from mrta.util import create_parser, print_red_on_cyan
from mrta.vis import vis
from mrta.workspace_case1 import Workspace

# Define the path for the 'data' folder
data_folder_path = "./data"

# Check if the folder exists
if not os.path.exists(data_folder_path):
    # If the folder does not exist, create it
    os.makedirs(data_folder_path)

# ---------------------------- LTL-MRTA---------------------------
parser = create_parser()
args = parser.parse_known_args()[0]


def ltl_mrta():
    workspace = Workspace()
    with open("data/workspace", "wb") as filehandle:
        pickle.dump(workspace, filehandle)

    # fig = plt.figure()
    # ax = fig.add_subplot(111)
    # plot_workspace(workspace, ax)
    # plt.show()

    # with open('data/workspace', 'rb') as filehandle:
    #     workspace = pickle.load(filehandle)
    # return to initial locations or not
    type_robot_location = workspace.type_robot_location.copy()
    loop = args.loop
    one_time = args.only_first
    draw = args.vis
    show = args.print
    partial_or_full = args.partial_or_full
    show_success = True
    best_cost = np.inf
    cost = []
    time_record = []
    horizon_record = []
    number_of_paths = 10
    best_path = dict()

    start = datetime.now()
    # --------------- constructing the Buchi automaton ----------------
    task = Task(args.case)
    buchi = Buchi(task, workspace)

    buchi.construct_buchi_graph()
    if show:
        print("partial time to build buchi: {0}".format((datetime.now() - start).total_seconds()))

    init_acpt = buchi.get_init_accept()

    for pair, _ in init_acpt:
        # workspace.type_robot_location = type_robot_location.copy()
        # workspace.update_after_prefix()
        # buchi.atomic_prop = workspace.atomic_prop
        # buchi.regions = workspace.regions

        init_state, accept_state = pair[0], pair[1]

        # ======================================= prefix part =================================================#
        #                                                                                                      #
        #                                                                                                      #
        #                                                                                                      #
        # ======================================= prefix part =================================================#

        # ----------------- infer the poset -----------------------
        pruned_subgraph, unpruned_subgraph, paths = buchi.get_subgraph(init_state, accept_state, "prefix")
        edge2element, element2edge = buchi.get_element(pruned_subgraph)
        if not edge2element:
            continue

        element_component2label = buchi.element2label2eccl(element2edge, pruned_subgraph)

        hasse_graphs = buchi.map_path_to_element_sequence(edge2element, paths)
        # loop over all posets
        for _, poset_relation, pos, hasse_diagram in hasse_graphs:
            if show:
                print_red_on_cyan("================ prefix part ================")

            # ----------------- restore after the suffix if not succeed -----------------
            workspace.type_robot_location = type_robot_location.copy()
            workspace.update_after_prefix()
            buchi.atomic_prop = workspace.atomic_prop
            buchi.regions = workspace.regions

            robot2eccl = poset.element2robot2eccl(pos, element2edge, pruned_subgraph)
            if show:
                print("partial time to poset: {0}".format((datetime.now() - start).total_seconds()))

            if show:
                for order in poset_relation:
                    print(
                        pruned_subgraph.edges[element2edge[order[0]]]["formula"],
                        " -> ",
                        pruned_subgraph.edges[element2edge[order[1]]]["formula"],
                    )
                print("----------------------------------------------")

            (
                incomparable_element,
                larger_element,
                smaller_element,
                strict_larger_element,
            ) = poset.incomparable_larger(pos, poset_relation, hasse_diagram)

            # --------------- construct the routing graph ---------------
            (
                init_type_robot_node,
                element_component_clause_literal_node,
                node_location_type_component_element,
                num_nodes,
            ) = weighted_ts.construct_node_set(pos, element2edge, pruned_subgraph, workspace.type_robot_label)

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

            ts = weighted_ts.construct_graph(num_nodes, node_location_type_component_element, edge_set, workspace.p2p)

            if show:
                print("partial time before milp: {0}".format((datetime.now() - start).total_seconds()))

            # --------------------- MILP -------------------------
            maximal_element = [node for node in hasse_diagram.nodes() if hasse_diagram.in_degree(node) == 0]

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
            if not robot_waypoint_pre:
                continue

            for robot, time in list(robot_time_pre.items()):
                #  delete such robots that did not participate (the initial location of robots may just satisfies)
                if time[-1] == 0 and len(time) == 1:
                    del robot_time_pre[robot]
                    del robot_waypoint_pre[robot]

            if show:
                print("----------------------------------------------")
                for type_robot, waypoint in robot_waypoint_pre.items():
                    print(type_robot, " : ", waypoint)
                    print(type_robot, " : ", robot_time_pre[type_robot])
                    print(type_robot, " : ", robot_label_pre[type_robot])
                print("----------------------------------------------")

                print("time axis: ", time_axis)

            for robot, time in list(robot_time_axis.items()):
                #  delete such robots that did not participate (the initial location of robots may just satisfies)
                if not time:
                    del robot_time_axis[robot]
                    del robot_waypoint_axis[robot]

            if show:
                for type_robot, waypoint in robot_waypoint_axis.items():
                    print(type_robot, " : ", waypoint)
                    print(type_robot, " : ", robot_time_axis[type_robot])

                print("----------------------------------------------")

                for stage in acpt_run:
                    print(stage)
                print("----------------------------------------------")

            # --------------------- GMRPP -------------------------
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
            # ----------------- check whether the final locations of the prefix part satisfy the accept state ---------
            workspace.type_robot_location = {robot: path[-1] for robot, path in robot_path_pre.items()}
            workspace.update_after_prefix(loop)
            buchi.atomic_prop = workspace.atomic_prop
            buchi.regions = workspace.regions

            last_subtask = acpt_run[-1]
            # add the removed self-loop of initial state
            if buchi.remove_init_attr:
                nx.set_node_attributes(pruned_subgraph, {init_state: buchi.remove_init_attr})
            # check whether final locations satisfy the self-loop of the accept state
            if buchi.ap_sat_label(
                pruned_subgraph.nodes[accept_state]["label"],
                pruned_subgraph.nodes[accept_state]["neg_label"],
            ):
                end = datetime.now()
                if show:
                    print("total time for the prefix parts: {0}".format((end - start).total_seconds()))
                cost_pre = compute_path_cost(robot_path_pre)
                cost.append(cost_pre)
                time_record.append((end - start).total_seconds())
                horizon_record.append(len(robot_path_pre[(1, 0)]))
                if best_cost >= cost_pre:
                    best_path = robot_path_pre
                    best_cost = cost_pre
                print(f"The total cost of the found path (loop) is: ")
                print(f"best total cost: {best_cost}")
                print(f"cost per solution (pre, suf): {cost}")
                print(f"runtimes up to now {time_record}")
                print(f"horizon per solution (pre, suf) {horizon_record}")
                if show_success:
                    print_red_on_cyan(task.formula)
                    print_red_on_cyan(
                        [
                            init_state,
                            accept_state,
                            buchi.size,
                            [
                                buchi.buchi_graph.number_of_nodes(),
                                buchi.buchi_graph.number_of_edges(),
                            ],
                            [
                                pruned_subgraph.number_of_nodes(),
                                pruned_subgraph.number_of_edges(),
                            ],
                            "A path is found for the case where the accepting state has a self-loop",
                        ]
                    )
                if draw:
                    vis(
                        args.case,
                        workspace,
                        robot_path_pre,
                        {robot: [len(path)] * 2 for robot, path in robot_path_pre.items()},
                        [],
                    )
                if one_time:
                    return
                elif len(cost) > number_of_paths:
                    return
                else:
                    continue

            # ======================================= suffix part =================================================#
            #                                                                                                      #
            #                                                                                                      #
            #                                                                                                      #
            # ======================================= suffix part =================================================#

            # ----------------- infer the poset -----------------------

            pruned_subgraph_suf, unpruned_subgraph_suf, paths_suf = buchi.get_subgraph(
                accept_state, accept_state, "suffix", last_subtask
            )
            # no suffix graph due to that final locations of prefix part do not satisfy the outgoing edges
            # of the accepting vertex
            if not pruned_subgraph_suf:
                continue

            # no paths due to the implication does no t hold
            if not paths_suf:
                continue

            edge2element_suf, element2edge_suf = buchi.get_element(pruned_subgraph_suf)

            element_component2label_suf = buchi.element2label2eccl(element2edge_suf, pruned_subgraph_suf)

            hasse_graphs_suf = buchi.map_path_to_element_sequence(edge2element_suf, paths_suf)

            for _, poset_relation_suf, pos_suf, hasse_diagram_suf in hasse_graphs_suf:
                if show:
                    print_red_on_cyan("================ suffix part ================")

                    for order_suf in poset_relation_suf:
                        print(
                            pruned_subgraph_suf.edges[element2edge_suf[order_suf[0]]]["formula"],
                            " -> ",
                            pruned_subgraph_suf.edges[element2edge_suf[order_suf[1]]]["formula"],
                        )
                    print("----------------------------------------------")

                robot2eccl_suf = poset.element2robot2eccl(pos_suf, element2edge_suf, pruned_subgraph_suf)

                (
                    incomparable_element_suf,
                    larger_element_suf,
                    smaller_element_suf,
                    strict_larger_element_suf,
                ) = poset.incomparable_larger(pos_suf, poset_relation_suf, hasse_diagram_suf)

                # --------------- construct the routing graph ---------------
                minimal_element_suf = [
                    node for node in hasse_diagram_suf.nodes() if hasse_diagram_suf.out_degree(node) == 0
                ]
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

                ts_suf = weighted_ts_suffix.construct_graph(
                    num_nodes_suf,
                    node_location_type_component_element_suf,
                    edge_set_suf,
                    workspace.p2p,
                )

                # --------------------- MILP -------------------------
                maximal_element_suf = [
                    node for node in hasse_diagram_suf.nodes() if hasse_diagram_suf.in_degree(node) == 0
                ]

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
                if not robot_waypoint_suf:
                    continue

                for robot, time in list(robot_time_suf.items()):
                    # delete such robots that did not participate (the initial location of robots may just satisfies)
                    if time[-1] == 0 and len(time) == 1:
                        del robot_time_suf[robot]
                        del robot_waypoint_suf[robot]
                if show:
                    print("----------------------------------------------")
                    for type_robot, waypoint in robot_waypoint_suf.items():
                        print(type_robot, " : ", waypoint)
                        print(type_robot, " : ", robot_time_suf[type_robot])
                        print(type_robot, " : ", robot_label_suf[type_robot])
                    print("----------------------------------------------")

                    print("time axis: ", time_axis_suf)

                for robot, time in list(robot_time_axis_suf.items()):
                    # delete such robots that did not participate (the initial location of robots may just satisfies)
                    if not time:
                        del robot_time_axis_suf[robot]
                        del robot_waypoint_axis_suf[robot]
                if show:
                    for type_robot, waypoint in robot_waypoint_axis_suf.items():
                        print(type_robot, " : ", waypoint)
                        print(type_robot, " : ", robot_time_axis_suf[type_robot])

                    print("----------------------------------------------")

                    for stage in acpt_run_suf:
                        print(stage)
                    print("----------------------------------------------")

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

                # return to initial locations
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

                robot_path = {
                    robot: path + robot_path_suf[robot][1:] + robot_path_suf[robot][1:]
                    for robot, path in robot_path_pre.items()
                }

                cost_pre = compute_path_cost(robot_path_pre)
                cos_suf = compute_path_cost(robot_path_suf)
                end = datetime.now()
                if show:
                    print("total time for the prefix + suffix parts: {0}".format((end - start).total_seconds()))

                cost.append((cost_pre, cos_suf))
                time_record.append((end - start).total_seconds())
                horizon_record.append((len(robot_path_pre[(1, 0)]), len(robot_path_suf[(1, 0)])))
                if best_cost >= cost_pre + cos_suf:
                    best_path = robot_path
                    best_cost = cost_pre + cos_suf
                print(f"The total cost of the found path (loop) is: ")
                print(f"best total cost: {best_cost}")
                print(f"cost per solution (pre, suf): {cost}")
                print(f"runtimes up to now {time_record}")
                print(f"horizon per solution (pre, suf) {horizon_record}")
                if show_success:
                    print_red_on_cyan(task.formula)
                    print_red_on_cyan(
                        [
                            init_state,
                            accept_state,
                            buchi.size,
                            [
                                buchi.buchi_graph.number_of_nodes(),
                                buchi.buchi_graph.number_of_edges(),
                            ],
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
                if draw:
                    vis(
                        args.case,
                        workspace,
                        robot_path,
                        {robot: [len(path)] * 2 for robot, path in robot_path.items()},
                        [],
                    )
                if one_time:
                    return
                if len(cost) > number_of_paths:
                    return
    return cost


if __name__ == "__main__":
    ltl_mrta()
