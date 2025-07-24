from gurobipy import *

from mrta.timer import log_time
from .post_processing import run
from termcolor import colored, cprint
import itertools

print_red_on_cyan = lambda x: cprint(x, "blue", "on_red")


def construct_milp_constraint(
    ts,
    type_num,
    poset,
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
):
    M = 1e5
    epsilon = 1  # edge and previous edge
    m = Model()
    # create variables
    x_vars, t_vars, c_vars, t_edge_vars, b_element_vars, b_immediate_element_vars, b_maximal_element_vars = (
        create_variables(m, ts, poset, pruned_subgraph, element2edge, type_num, maximal_element)
    )
    # create initial constraints
    initial_constraints(m, x_vars, t_vars, ts, init_type_robot_node, type_num)

    # network and schedule constraints
    network_schedule_constraints(
        m, ts, x_vars, t_vars, init_type_robot_node, incomparable_element, larger_element, type_num, M, epsilon
    )

    # edge time constraints -- eq (12)
    for element in poset:
        edge_label = pruned_subgraph.edges[element2edge[element]]["label"]
        # must have vertices
        if edge_label != "1":
            m.addConstr(
                quicksum(
                    t_vars[(element_component_clause_literal_node[(element, 1, c, 0)][0], k, 1)]
                    for c in range(len(edge_label))
                    for k in range(
                        type_num[
                            ts.nodes[element_component_clause_literal_node[(element, 1, c, 0)][0]][
                                "location_type_component_element"
                            ][1]
                        ]
                    )
                )
                == t_edge_vars[element]
            )

    m.update()

    # binary relation between edge time -- eq (19)
    # no matter the edge label is '1' or not
    for pair in itertools.combinations(poset, 2):
        # no subtasks are completed at the same time -- eq (19a)
        m.addConstr(b_element_vars[(pair[0], pair[1])] + b_element_vars[(pair[1], pair[0])] == 1)

    for element in poset:
        for another_element in poset:
            if element != another_element:
                # # no subtasks are completed at the same time -- eq (19a)
                # m.addConstr(b_element_vars[(element, another_element)] + b_element_vars[(another_element, element)] == 1)
                # -- eq (19b)
                m.addConstr(
                    M * (b_element_vars[(element, another_element)] - 1)
                    <= t_edge_vars[element] - t_edge_vars[another_element]
                )

                m.addConstr(
                    t_edge_vars[element] - t_edge_vars[another_element]
                    <= M * b_element_vars[(element, another_element)] - epsilon
                )
    m.update()

    # using same robot (26a) and (26b)
    for robot, eccls in robot2eccl.items():
        clause = [list(group) for key, group in itertools.groupby(eccls, operator.itemgetter(0, 1))]
        num_vertex = len(element_component_clause_literal_node[eccls[0]])
        num_robot = type_num[
            ts.nodes[element_component_clause_literal_node[eccls[0]][0]]["location_type_component_element"][1]
        ]
        for l_base in clause[0]:
            for c in clause[1:]:
                for l in c:
                    for vertex in range(num_vertex):
                        v = element_component_clause_literal_node[l_base][vertex]
                        w = element_component_clause_literal_node[l][vertex]
                        for k in range(num_robot):
                            m.addConstr(
                                quicksum(x_vars[(p, w, k)] for p in ts.predecessors(w)) + M * (c_vars[l[:-1]] - 1)
                                <= quicksum(x_vars[(p, v, k)] for p in ts.predecessors(v))
                                + M * (1 - c_vars[l_base[:-1]])
                            )

                            m.addConstr(
                                quicksum(x_vars[(p, v, k)] for p in ts.predecessors(v)) + M * (c_vars[l_base[:-1]] - 1)
                                <= quicksum(x_vars[(p, w, k)] for p in ts.predecessors(w)) + M * (1 - c_vars[l[:-1]])
                            )

    m.update()
    # focus on label
    for element in poset:
        self_loop_label = pruned_subgraph.nodes[element2edge[element][0]]["label"]
        edge_label = pruned_subgraph.edges[element2edge[element]]["label"]

        edge_constraints(
            m,
            ts,
            x_vars,
            t_vars,
            c_vars,
            t_edge_vars,
            b_element_vars,
            element,
            self_loop_label,
            edge_label,
            poset_relation,
            element_component_clause_literal_node,
            type_num,
            M,
            epsilon,
            buchi,
            strict_larger_element,
            incomparable_element,
        )

        if self_loop_label and self_loop_label != "1":
            self_loop_constraints(
                m,
                ts,
                x_vars,
                t_vars,
                c_vars,
                t_edge_vars,
                b_element_vars,
                b_immediate_element_vars,
                element,
                self_loop_label,
                strict_larger_element,
                incomparable_element,
                poset_relation,
                element_component_clause_literal_node,
                type_num,
                M,
                buchi,
                pruned_subgraph,
                element2edge,
            )

        # activation of the next subtask
        activate_next(
            m,
            ts,
            x_vars,
            t_vars,
            c_vars,
            t_edge_vars,
            b_element_vars,
            b_immediate_element_vars,
            element,
            self_loop_label,
            strict_larger_element,
            incomparable_element,
            poset_relation,
            element_component_clause_literal_node,
            type_num,
            M,
            buchi,
            pruned_subgraph,
            element2edge,
        )
    # activation of the first subtask
    activation_first(
        m,
        ts,
        x_vars,
        t_vars,
        c_vars,
        b_element_vars,
        element_component_clause_literal_node,
        type_num,
        M,
        buchi,
        pruned_subgraph,
        element2edge,
        maximal_element,
        b_maximal_element_vars,
        init_type_robot_node,
    )

    expr = LinExpr([0.7 * ts.edges[tuple(index[:2])]["weight"] for index in x_vars.keys()], list(x_vars.values()))
    expr.add(LinExpr([0.3] * len([key for key in t_edge_vars.keys()]), [value for key, value in t_edge_vars.items()]))
    m.setObjective(expr, GRB.MINIMIZE)
    if not show:
        m.Params.OutputFlag = 0
    # m.Params.MIPGap = 0.1
    m.update()
    if show:
        print("# of variables: {0}".format(m.NumVars))
        print("# of constraints: {0}".format(m.NumConstrs))

    m.setParam("OutputFlag", 0)

    def my_callback(model, where):
        if where == GRB.Callback.MIPSOL:
            obj = model.cbGet(GRB.Callback.MIPSOL_OBJ)
            cprint(f"Intermediate solution found: objective = {obj}", "red")
            log_time("前缀部分MILP找到中间解")

    m.optimize(my_callback)

    if m.status == GRB.Status.OPTIMAL:
        if show:
            print("Optimal objective: %g" % m.objVal)
    elif m.status == GRB.Status.INF_OR_UNBD:
        print("Model is infeasible or unbounded")
    elif m.status == GRB.Status.INFEASIBLE:
        if show:
            print_red_on_cyan("Model is infeasible")
    elif m.status == GRB.Status.UNBOUNDED:
        print("Model is unbounded")
    else:
        print("Optimization ended with status %d" % m.status)
    if m.status != GRB.Status.OPTIMAL:
        return None, None, None, None, None, None, None, None

    goal = 0
    for index in x_vars.keys():
        goal += ts.edges[tuple(index[:2])]["weight"] * x_vars[index].x
    if show:
        print("obj:%g" % goal)

    id2robots = dict()

    get_same_robot(id2robots, robot2eccl, x_vars, element_component_clause_literal_node, type_num, ts)

    # obtain the time axis
    time_axis = get_axis(t_edge_vars)

    robot_waypoint_pre, robot_time_pre, robot_label_pre, robot_waypoint_axis, robot_time_axis = get_waypoint(
        x_vars, t_vars, ts, init_type_robot_node, time_axis
    )

    # extract the run
    acpt_run = run(
        pruned_subgraph,
        time_axis,
        init_state,
        element2edge,
        {"x": x_vars, "c": c_vars, "t": t_edge_vars},
        element_component_clause_literal_node,
        ts,
        type_num,
        dict(),
        buchi,
        show,
    )

    return (
        robot_waypoint_pre,
        robot_time_pre,
        id2robots,
        robot_label_pre,
        robot_waypoint_axis,
        robot_time_axis,
        time_axis,
        acpt_run,
    )


def create_variables(m, ts, poset, pruned_subgraph, element2edge, type_num, maximal_element):
    # clause variable, (element, 0|1, clause_index)
    c_vars = {}
    for element in poset:
        self_loop_label = pruned_subgraph.nodes[element2edge[element][0]]["label"]
        edge_label = pruned_subgraph.edges[element2edge[element]]["label"]
        if self_loop_label and self_loop_label != "1":
            c_vars.update(m.addVars([element], [0], list(range(len(self_loop_label))), vtype=GRB.BINARY))
        if edge_label != "1":
            c_vars.update(m.addVars([element], [1], list(range(len(edge_label))), vtype=GRB.BINARY))

    m.update()

    # time variable (node, robot_index, -|+)
    t_vars = {}
    for node in ts.nodes():
        # vars for self loop
        if ts.nodes[node]["location_type_component_element"][2] == 0:
            t_vars.update(
                m.addVars(
                    [node],
                    list(range(type_num[ts.nodes[node]["location_type_component_element"][1]])),
                    [0, 1],
                    vtype=GRB.INTEGER,
                )
            )
        # vars for edge
        else:
            t_vars.update(
                m.addVars(
                    [node],
                    list(range(type_num[ts.nodes[node]["location_type_component_element"][1]])),
                    [1],
                    vtype=GRB.INTEGER,
                )
            )
    m.update()

    # routing variable (node_i, node_j, robot_index)
    x_vars = {}
    for edge in ts.edges():
        x_vars.update(
            m.addVars(
                [edge[0]],
                [edge[1]],
                list(range(type_num[ts.nodes[edge[1]]["location_type_component_element"][1]])),
                vtype=GRB.BINARY,
            )
        )
    m.update()

    # edge time variable (element)
    # even if the edge label is '1'
    t_edge_vars = {}
    for element in poset:
        t_edge_vars.update(m.addVars([element], vtype=GRB.INTEGER))
    m.update()

    # binary relation between edge time (element, element)
    b_element_vars = {}
    for element in poset:
        for another_element in poset:
            if element != another_element:
                b_element_vars.update(m.addVars([element], [another_element], vtype=GRB.BINARY))
    m.update()

    # binary relation between edge time (element, element) to indicate whether two subtasks are consecutive
    b_immediate_element_vars = {}
    for element in poset:
        for another_element in poset:
            if element != another_element:
                b_immediate_element_vars.update(m.addVars([element], [another_element], vtype=GRB.BINARY))
    m.update()

    b_maximal_element_vars = {}
    for element in maximal_element:
        b_maximal_element_vars.update(m.addVars([element], vtype=GRB.INTEGER))
    m.update()

    return x_vars, t_vars, c_vars, t_edge_vars, b_element_vars, b_immediate_element_vars, b_maximal_element_vars


def initial_constraints(m, x_vars, t_vars, ts, init_type_robot_node, type_num):
    # create initial constraints
    # nodes for initial locations -- eq (5)
    for type_robot, node in init_type_robot_node.items():
        m.addConstr(1 >= quicksum(x_vars[(node, s, type_robot[1])] for s in ts.successors(node)))  # 5a
        m.addConstr(
            0
            == quicksum(
                x_vars[(node, s, k)]
                for s in ts.successors(node)
                for k in range(type_num[ts.nodes[s]["location_type_component_element"][1]])  # 5b
                if k != type_robot[1]
            )
        )
        # initial time -- eq (7)
        for k in range(type_num[type_robot[0]]):
            m.addConstr(t_vars[(node, k, 1)] == 0)
    m.update()


def one_clause_true(m, c_vars, element, component, label, strict_larger_element, incomparable_element, buchi):
    """
    only one clause is true
    """
    strict_incmp = strict_larger_element[element] + incomparable_element[element]
    z = len(strict_incmp) - 1
    # first subtask, the vertex label is not satisfied by the initial robot locations
    if z == -1 and component == 0 and not buchi.sat_vertex:
        m.addConstr(quicksum(c_vars[(element, component, c)] for c in range(len(label))) == 0)
        m.update()
        return
    m.addConstr(quicksum(c_vars[(element, component, c)] for c in range(len(label))) == 1)
    m.update()


def literal_clause(m, x_vars, c_vars, element, component, label, ts, type_num, clause_nodes, c):
    # encode the relation between clause and its literals -- eq (10)
    expr_literal = quicksum(
        x_vars[(p, i, k)]
        for literal_nodes in clause_nodes
        for i in literal_nodes
        for p in ts.predecessors(i)
        for k in range(type_num[ts.nodes[i]["location_type_component_element"][1]])
    )
    mult = sum([l[2] for l in label[c]])
    m.addConstr(expr_literal / mult == c_vars[(element, component, c)])
    m.update()


def network_schedule_constraints(
    m, ts, x_vars, t_vars, init_type_robot_node, incomparable_element, larger_element, type_num, M, epsilon
):
    # focus on nodes
    for i in ts.nodes():
        if i not in init_type_robot_node.values() and list(ts.predecessors(i)):  # the predecessor of i
            # each (replica) location represented by node is visited by at most one robot of type k -- eq (3)
            m.addConstr(
                quicksum(
                    x_vars[(p, i, k)]
                    for p in ts.predecessors(i)
                    for k in range(type_num[ts.nodes[i]["location_type_component_element"][1]])
                )
                <= 1
            )

            for k in range(type_num[ts.nodes[i]["location_type_component_element"][1]]):
                # routing network that the visitation of a robot to a location i
                # is the precondition for its departure from that location -- eq (4)
                m.addConstr(
                    quicksum(x_vars[(p, i, k)] for p in ts.predecessors(i))
                    >= quicksum(x_vars[(i, s, k)] for s in ts.successors(i))
                )
                # t_ik = 0 if location i is not visited by the robot k -- eq (6)
                m.addConstr(t_vars[(i, k, 1)] <= M * quicksum(x_vars[(p, i, k)] for p in ts.predecessors(i)))
                if ts.nodes[i]["location_type_component_element"][2] == 0:
                    m.addConstr(t_vars[(i, k, 0)] <= M * quicksum(x_vars[(p, i, k)] for p in ts.predecessors(i)))

                # The time of robot k visiting p should be larger than that of the same robot
                # visiting i if there is a plan from p to i -- eq (8)
                i_element = ts.nodes[i]["location_type_component_element"][3]
                for p in ts.predecessors(i):
                    p_element = ts.nodes[p]["location_type_component_element"][3]
                    # incomparable elements, using epsilon to avoid loop  (8b)
                    if p_element in incomparable_element[i_element]:
                        if ts.nodes[i]["location_type_component_element"][2] == 0:
                            m.addConstr(
                                t_vars[(p, k, 1)] + (epsilon + ts.edges[(p, i)]["weight"]) * x_vars[(p, i, k)]
                                <= t_vars[(i, k, 0)] + M * (1 - x_vars[(p, i, k)])
                            )
                        else:
                            m.addConstr(
                                t_vars[(p, k, 1)] + (epsilon + ts.edges[(p, i)]["weight"]) * x_vars[(p, i, k)]
                                <= t_vars[(i, k, 1)] + M * (1 - x_vars[(p, i, k)])
                            )
                    # precedent elements, include initial element -1   (8a)
                    elif p_element in larger_element[i_element] + [-1]:
                        if ts.nodes[i]["location_type_component_element"][2] == 0:
                            m.addConstr(
                                t_vars[(p, k, 1)] + ts.edges[(p, i)]["weight"] * x_vars[(p, i, k)]
                                <= t_vars[(i, k, 0)] + M * (1 - x_vars[(p, i, k)])
                            )
                        else:
                            m.addConstr(
                                t_vars[(p, k, 1)] + ts.edges[(p, i)]["weight"] * x_vars[(p, i, k)]
                                <= t_vars[(i, k, 1)] + M * (1 - x_vars[(p, i, k)])
                            )
                    # same element, i corresponds to the edge, p corresponds to the vertex (8a)
                    elif p_element == i_element:
                        m.addConstr(
                            t_vars[(p, k, 1)] + ts.edges[(p, i)]["weight"] * x_vars[(p, i, k)]
                            <= t_vars[(i, k, 1)] + M * (1 - x_vars[(p, i, k)])
                        )

    m.update()


def edge_constraints(
    m,
    ts,
    x_vars,
    t_vars,
    c_vars,
    t_edge_vars,
    b_element_vars,
    element,
    self_loop_label,
    edge_label,
    poset_relation,
    element_component_clause_literal_node,
    type_num,
    M,
    epsilon,
    buchi,
    strict_larger_element,
    incomparable_element,
):
    # one and only one clause is true -- eq (9)
    if edge_label != "1":
        one_clause_true(m, c_vars, element, 1, edge_label, strict_larger_element, incomparable_element, buchi)

        for c in range(len(edge_label)):
            # the nodes corresponding to each clause
            clause_nodes = []  # each item is a set of literal nodes
            for l in range(len(edge_label[c])):
                clause_nodes.append(element_component_clause_literal_node[(element, 1, c, l)])

            # encode the relation between clause and its literals -- eq (10)
            literal_clause(m, x_vars, c_vars, element, 1, edge_label, ts, type_num, clause_nodes, c)

            # encode the synchronization constraints in terms of timing -- eq (11)
            for l in clause_nodes:
                for i in l:
                    m.addConstr(
                        quicksum(
                            t_vars[(clause_nodes[0][0], k, 1)]
                            for k in range(type_num[ts.nodes[clause_nodes[0][0]]["location_type_component_element"][1]])
                        )
                        == quicksum(
                            t_vars[(i, k, 1)]
                            for k in range(type_num[ts.nodes[i]["location_type_component_element"][1]])
                        )
                    )

    m.update()

    # timing constraints between a node and its outgoing edge -- eq (13)
    strict_incmp = strict_larger_element[element] + incomparable_element[element]
    z = len(strict_incmp) - 1
    # has a self-loop with positive literals: if the first subtask, then initial locations must satisfy the self-loop
    # else if not the first subtask, no addtional constraints
    if self_loop_label and (
        (self_loop_label != "1" and z != -1) or (self_loop_label != "1" and z == -1 and buchi.sat_vertex)
    ):
        for c_self_loop in range(len(self_loop_label)):
            for l_self_loop in range(len(self_loop_label[c_self_loop])):
                for i in element_component_clause_literal_node[(element, 0, c_self_loop, l_self_loop)]:
                    m.addConstr(
                        quicksum(
                            t_vars[(i, k, 0)]
                            for k in range(type_num[ts.nodes[i]["location_type_component_element"][1]])
                        )
                        <= t_edge_vars[element]
                    )

                    m.addConstr(
                        t_edge_vars[element]
                        <= quicksum(
                            t_vars[(i, k, 1)]
                            for k in range(type_num[ts.nodes[i]["location_type_component_element"][1]])
                        )
                        + 1
                        + M * (1 - c_vars[(element, 0, c_self_loop)])
                    )
    # the vertex label is false, or the initial robot locations satisfy the not true vertex label of initial vertex,
    # the edge label becomes true at 0, --- (14)
    if z == -1 and (not self_loop_label or (self_loop_label != "1" and not buchi.sat_vertex)):
        m.addConstr(t_edge_vars[element] == 0)

    # precedence timing constraints, between edge and previous edge, with vertex label, covered by -- eq (15)
    for another_element in strict_larger_element[element]:
        m.addConstr(t_edge_vars[another_element] + epsilon <= t_edge_vars[element])

    m.update()


def self_loop_constraints(
    m,
    ts,
    x_vars,
    t_vars,
    c_vars,
    t_edge_vars,
    b_element_vars,
    b_immediate_element_vars,
    element,
    self_loop_label,
    strict_larger_element,
    incomparable_element,
    poset_relation,
    element_component_clause_literal_node,
    type_num,
    M,
    buchi,
    pruned_subgraph,
    element2edge,
):
    # one and only one clause is true, -- eq (9)
    one_clause_true(m, c_vars, element, 0, self_loop_label, strict_larger_element, incomparable_element, buchi)

    for c in range(len(self_loop_label)):
        # the nodes corresponding to each clause
        clause_nodes = []
        for l in range(len(self_loop_label[c])):
            clause_nodes.append(element_component_clause_literal_node[(element, 0, c, l)])

        # encode the relation between clause and its literals -- eq (10)
        literal_clause(m, x_vars, c_vars, element, 0, self_loop_label, ts, type_num, clause_nodes, c)
    m.update()


def activate_next(
    m,
    ts,
    x_vars,
    t_vars,
    c_vars,
    t_edge_vars,
    b_element_vars,
    b_immediate_element_vars,
    element,
    self_loop_label,
    strict_larger_element,
    incomparable_element,
    poset_relation,
    element_component_clause_literal_node,
    type_num,
    M,
    buchi,
    pruned_subgraph,
    element2edge,
):
    # only one subtask
    if not b_immediate_element_vars:
        return
    # subtasks that can immediately follow the current subtask
    strict_smaller = [order[1] for order in poset_relation if order[0] == element]
    # subtask can not be the last one
    if strict_smaller:
        # there exists a subtask that occurs immediately after it -- eq (16)
        m.addConstr(
            quicksum(
                b_immediate_element_vars[(element, another_element)]
                for another_element in strict_smaller + incomparable_element[element]
            )
            == 1
        )
    # subtask can be the last one -- eq (20)
    else:
        z = len(incomparable_element[element])
        m.addConstr(
            quicksum(
                b_immediate_element_vars[(element, another_element)]
                for another_element in incomparable_element[element]
            )
            <= 1
        )
        m.addConstr(
            z
            - quicksum(b_element_vars[(element, another_element)] for another_element in incomparable_element[element])
            - M
            * quicksum(
                b_immediate_element_vars[(element, another_element)]
                for another_element in incomparable_element[element]
            )
            <= 0
        )

        m.addConstr(
            quicksum(
                b_immediate_element_vars[(element, another_element)]
                for another_element in incomparable_element[element]
            )
            - M
            * (
                z
                - quicksum(
                    b_element_vars[(element, another_element)] for another_element in incomparable_element[element]
                )
            )
            <= 0
        )
    m.update()

    # if it can not be the first subtask to be completed, then it has to immediately follow one subtask -- eq (21)
    if strict_larger_element[element]:
        m.addConstr(
            quicksum(
                b_immediate_element_vars[(another_element, element)]
                for another_element in strict_larger_element[element] + incomparable_element[element]
            )
            == 1
        )

    else:
        m.addConstr(
            quicksum(
                b_immediate_element_vars[(another_element, element)]
                for another_element in incomparable_element[element]
            )
            <= 1
        )

        m.addConstr(
            quicksum(b_element_vars[(element, another_element)] for another_element in incomparable_element[element])
            - M
            * quicksum(
                b_immediate_element_vars[(another_element, element)]
                for another_element in incomparable_element[element]
            )
            <= 0
        )
        m.addConstr(
            quicksum(
                b_immediate_element_vars[(another_element, element)]
                for another_element in incomparable_element[element]
            )
            - M
            * quicksum(b_element_vars[(element, another_element)] for another_element in incomparable_element[element])
            <= 0
        )

    # if occurs immediately after subtask, then it should be completed after the current subtask -- eq (17)
    # If subtask e is not the last subtask, there should be a subtask  immediately after e
    for another_element in strict_smaller + incomparable_element[element]:
        m.addConstr(
            t_edge_vars[element] + 1
            <= t_edge_vars[another_element] + M * (1 - b_immediate_element_vars[(element, another_element)])
        )

        # at most one time instant later than the completion of the current subtask -- eq (18)
        self_loop_label_another_ele = pruned_subgraph.nodes[element2edge[another_element][0]]["label"]
        if self_loop_label_another_ele and self_loop_label_another_ele != "1":
            for c_self_loop in range(len(self_loop_label_another_ele)):
                for l_self_loop in range(len(self_loop_label_another_ele[c_self_loop])):
                    for i in element_component_clause_literal_node[(another_element, 0, c_self_loop, l_self_loop)]:
                        m.addConstr(
                            quicksum(
                                t_vars[(i, k, 0)]
                                for k in range(type_num[ts.nodes[i]["location_type_component_element"][1]])
                            )
                            <= t_edge_vars[element] + 1 + M * (1 - b_immediate_element_vars[(element, another_element)])
                        )

    m.update()


def activation_first(
    m,
    ts,
    x_vars,
    t_vars,
    c_vars,
    b_element_vars,
    element_component_clause_literal_node,
    type_num,
    M,
    buchi,
    pruned_subgraph,
    element2edge,
    maximal_element,
    b_maximal_element_vars,
    init_type_robot_node,
):
    #  the first completed subtask has a self-loop then the vertex label  should be activated at time 0 -- eq (23)
    if buchi.sat_vertex:
        for element in maximal_element:
            self_loop_label = pruned_subgraph.nodes[element2edge[element][0]]["label"]
            if self_loop_label and self_loop_label != "1":
                for c in range(len(self_loop_label)):
                    for l in range(len(self_loop_label[c])):
                        for j in element_component_clause_literal_node[(element, 0, c, l)]:
                            m.addConstr(
                                quicksum(
                                    t_vars[(j, k, 0)]
                                    for k in range(type_num[ts.nodes[j]["location_type_component_element"][1]])
                                )
                                <= M
                                * (
                                    quicksum(
                                        b_element_vars[(element, another_element)]
                                        for another_element in maximal_element
                                        if another_element != element
                                    )
                                    + 1
                                    - c_vars[(element, 0, c)]
                                )
                            )

    m.update()
    # -- eq (24)
    for element in maximal_element:
        m.addConstr(
            quicksum(
                b_element_vars[(element, another_element)]
                for another_element in maximal_element
                if another_element != element
            )
            - M * (1 - b_maximal_element_vars[element])
            <= 0
        )
        m.addConstr(
            (1 - b_maximal_element_vars[element])
            - M
            * quicksum(
                b_element_vars[(element, another_element)]
                for another_element in maximal_element
                if another_element != element
            )
            <= 0
        )
    m.update()

    # the constraints regarding the categories of leaving vertices -- eq (25)

    for element in maximal_element:
        self_loop_label = pruned_subgraph.nodes[element2edge[element][0]]["label"]
        if self_loop_label and self_loop_label != "1":
            for c in range(len(self_loop_label)):
                clause_nodes = []
                for l in range(len(self_loop_label[c])):
                    clause_nodes.append(element_component_clause_literal_node[(element, 0, c, l)])
            # -- eq (25a)
            if buchi.sat_vertex:
                m.addConstr(
                    quicksum(
                        x_vars[(p, i, k)]
                        for literal_nodes in clause_nodes
                        for i in literal_nodes
                        for k in range(type_num[ts.nodes[i]["location_type_component_element"][1]])
                        for p in ts.predecessors(i)
                        if p not in init_type_robot_node.values()
                    )
                    <= M * (1 - b_maximal_element_vars[element])
                )
            # -- eq (25b)
            m.addConstr(
                quicksum(
                    x_vars[(p, i, k)]
                    for literal_nodes in clause_nodes
                    for i in literal_nodes
                    for k in range(type_num[ts.nodes[i]["location_type_component_element"][1]])
                    for p in ts.predecessors(i)
                    if p in init_type_robot_node.values()
                )
                <= M * b_maximal_element_vars[element]
            )

        m.update()


def get_waypoint(x_vars, t_vars, ts, init_type_robot_node, time_axis):
    """
    extract the high-level plan
    """
    robot_waypoint = dict()
    robot_time = dict()
    robot_label = dict()

    robot_waypoint_axis = dict()
    robot_time_axis = dict()

    for type_robot, node in init_type_robot_node.items():
        path = [node]
        time = [0]
        label = [0]
        is_found = True
        while is_found:
            pre = path[-1]
            for s in ts.succ[pre]:
                if round(x_vars[(pre, s, type_robot[1])].x) == 1:
                    try:
                        time.append(round(t_vars[(s, type_robot[1], 0)].x))
                        path.append(s)
                        label.append(ts.nodes[s]["location_type_component_element"][2])
                    except KeyError:
                        pass
                    time.append(round(t_vars[(s, type_robot[1], 1)].x))
                    path.append(s)
                    # literal, clause, component, element
                    label.append(ts.nodes[s]["location_type_component_element"][2])
                    break
            if path[-1] == pre:
                is_found = False
        # one-to-one correspondence between the waypoint and time
        robot_waypoint[type_robot] = [ts.nodes[point]["location_type_component_element"][0] for point in path]
        robot_time[type_robot] = time
        robot_label[type_robot] = label
        # one-to-one correspondence between the waypoint and time that aligns with the time axis
        robot_waypoint_axis[type_robot] = [
            ts.nodes[point]["location_type_component_element"][0]
            for point in path
            if ts.nodes[point]["location_type_component_element"][2] == 1
            and ts.nodes[point]["location_type_component_element"][3] > -1
        ]

        robot_time_axis[type_robot] = [
            time_element[0]
            for point in path
            for time_element in time_axis
            if ts.nodes[point]["location_type_component_element"][2] == 1
            and time_element[1] == ts.nodes[point]["location_type_component_element"][3]
        ]

    return robot_waypoint, robot_time, robot_label, robot_waypoint_axis, robot_time_axis


def get_same_robot(id2robots, robot2eccl, x_vars, element_component_clause_literal_node, type_num, ts):
    for robot, eccls in robot2eccl.items():
        robots = []
        num_vertex = len(element_component_clause_literal_node[eccls[0]])
        num_robot = type_num[
            ts.nodes[element_component_clause_literal_node[eccls[0]][0]]["location_type_component_element"][1]
        ]
        # detemine the eccl that the corresponding vertices are visited
        vertices = None
        for eccl in eccls:
            vertices = element_component_clause_literal_node[eccl]
            if sum([x_vars[(p, vertices[0], k)].x for p in ts.predecessors(vertices[0]) for k in range(num_robot)]) > 0:
                break
        for vertex in range(num_vertex):
            v = vertices[vertex]
            for k in range(num_robot):
                if sum([x_vars[(p, v, k)].x for p in ts.predecessors(v)]) == 1:
                    robots.append(k)
                    break
        id2robots[robot] = robots


def get_axis(t_edge_vars):
    time_axis = [[round(t_edge_vars[t].x), t] for t in t_edge_vars.keys()]
    time_axis.sort()

    offset = 0
    value = time_axis[0][0]
    for i in range(1, len(time_axis)):
        if time_axis[i][0] == value:
            offset += 1
            time_axis[i][0] = time_axis[i][0] + offset
        else:
            value = time_axis[i][0]
            time_axis[i][0] = time_axis[i][0] + offset
    return time_axis
