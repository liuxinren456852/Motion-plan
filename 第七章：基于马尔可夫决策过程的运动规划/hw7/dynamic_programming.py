import pickle
from racetracks import *
from graph_node import Node
import matplotlib.pyplot as plt

seed = np.random.seed(5678)
graph = {}


# 用于根据环境地图构建状态图
def build_up_graph(grid, save_path):
    max_vel = 5                                         # 最大速度

    # 可以执行的速度列表
    # velocity dimension            
    vel_list = []
    for i_vel in range(-max_vel+1, max_vel):
        for j_vel in range(-max_vel+1, max_vel):
            vel_list.append([i_vel, j_vel])

    # position dimension
    x_idx, y_idx = np.where(grid == FREE)                       # 寻找安全位置
    coord = np.stack([x_idx, y_idx], axis=1)                    # 合并数组
    for p_idx in range(coord.shape[0]):                         # 获取每个位置
        pnt = coord[p_idx]
        for vel in vel_list:                                    # 对每个位置遍历每个速度
            state = Node(pnt[0], pnt[1], vel[0], vel[1])        # 根据位置和速度链接到图中
            state.connect_to_graph(grid)
            graph[state.key] = state                            # 获取对应的状态标识

    # 加入起点
    for pnt in START_LINE:
        state = Node(pnt[0], pnt[1], 0, 0)
        state.connect_to_graph(grid)
        graph[state.key] = state

    # 加入终点
    for pnt in FINISH_LINE:
        state = Node(pnt[0], pnt[1], 0, 0)
        state.is_goal = True
        graph[state.key] = state

    output = open(save_path, 'wb')
    pickle.dump(graph, output)



def check_graph(grid):
    plt.figure(figsize=(4.5, 16))
    plt.pcolor(grid, edgecolors='k', linewidths=1)
    for key in graph.keys():
        for child_idx, child_key in enumerate(graph[key].next_prob_1): # or next_prob_1
            ux, uy = ACTION_SPACE[child_idx]
            vx, vy = graph[key].vx + ux,  graph[key].vy + uy
            child = graph[child_key]
            # check a specific connection
            # plt.title(str(vy) + '_' + str(vx))
            # plt.show()
            if [child.px, child.py] in START_LINE:
                print('found')
                continue
            plt.arrow(graph[key].py + 0.5, graph[key].px + 0.5,
                      child.py - graph[key].py, child.px - graph[key].px,
                      color='r', head_width=0.3, head_length=0.1)
            print(key, child_idx)
        # end for
    # end for
    plt.show()


def track_the_best_plan(idx = 0):
    start_node = Node(START_LINE[idx][0], START_LINE[idx][1], 0, 0)
    start_key = start_node.key
    state = graph[start_key]
    trajectory = [state]
    # for i in range(grid.shape[0]+grid.shape[1]) a safer condition
    while not state.is_goal:
        value_uk = []
        for child_idx in range(len(ACTION_SPACE)):
            # 在执行时只选取最大概率的那个
            child_key_9 = state.next_prob_9[child_idx]
            child_9 = graph[child_key_9]
            value_uk.append(child_9.g_value)
        child_key = state.next_prob_9[np.argmin(value_uk)]
        state = graph[child_key]
        trajectory.append(state)
        print(state.px, state.py)
    return trajectory



def visualize_the_best_plan(plan, grid_para):
    assert isinstance(plan, list)
    plt.figure(figsize=(4.5, 16))
    plt.pcolor(grid_para, edgecolors='k', linewidths=1)
    plan_len = len(plan)
    plan.append(plan[-1])
    for i in range(plan_len):
        # 绘制箭头
        plt.arrow(plan[i].py + 0.5, plan[i].px + 0.5,
                  plan[i+1].py - plan[i].py, plan[i+1].px - plan[i].px,
                  color='r', head_width=0.3, head_length=0.1)
    plt.show()



def dynamic_programming():
    itr_num = 0
    bellman_error = np.inf
    bellman_error_list = []
    while bellman_error > 0.0001:
        itr_num += 1
        bellman_error = 0.0
        # 遍历所有状态
        for key in graph.keys():
            state = graph[key]
            # 如果是目标节点
            if state.is_goal:
                state.g_value = 0
            else:
                # 遍历输入空间
                value_uk = []
                for child_idx in range(len(ACTION_SPACE)):
                    # 根据概率更新相应的值
                    child_key_9 = state.next_prob_9[child_idx]
                    child_9 = graph[child_key_9]
                    child_key_1 = state.next_prob_1[child_idx]
                    child_1 = graph[child_key_1]
                    expected_cost_uk = 0.9 * (1 + child_9.g_value) + 0.1 * (1 + child_1.g_value)
                    value_uk.append(expected_cost_uk)
                # 选择对应于cost最小的uk
                current_value = min(value_uk)
                # 贝尔曼误差
                bellman_error += np.linalg.norm(state.g_value - current_value)
                state.g_value = min(value_uk)
            # end if
        # end for
        bellman_error_list.append(bellman_error)
        print("{}th iteration: {}".format(itr_num, bellman_error))
    # end while

    # 绘制误差变化曲线
    plt.figure()
    x_axis = range(len(bellman_error_list))
    plt.plot(x_axis, bellman_error_list)
    plt.show()



if __name__ == '__main__':
    # 状态图
    path = './solution/graph_dp.dat'
    # 环境地图
    track_map = race_track
    # 根据环境地图构建状态图
    #build_up_graph(track_map, path)
    #
    graph = pickle.load(open(path, 'rb'))
    # solve
    # 求解动态规划问题
    dynamic_programming()
    # 获取最优的路径
    plan = track_the_best_plan(idx = 0)
    # 可视化路径
    visualize_the_best_plan(plan, track_map)
