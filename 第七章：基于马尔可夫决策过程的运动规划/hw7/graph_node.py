from racetracks import *

def Bresenham(x1, y1, x2, y2):
	pointList = []
	if x1 == x2: # Special Case: Horizenal Line
		if y1 <= y2:
			return [[x1, y] for y in range(y1, y2 + 1)]
		else:
			pointList = [[x1, y] for y in range(y2, y1 + 1)]
			pointList.reverse()
			return pointList
	elif abs(y2 - y1) <= abs(x2 - x1): # abs(slope) <= 1
		return BresenhamLine(x1, y1, x2, y2)
	else: # abs(slope) >= 1: Axis Reverse
		pointList = BresenhamLine(y1, x1, y2, x2)
		return [[p[1], p[0]] for p in pointList]

def BresenhamLine(x1, y1, x2, y2): # abs(slope) <= 1
	# Parameter of Drawing
	slope = (y2 - y1) / (x2 - x1)
	# Initialize
	p = 2 * slope - 1
	[x, y] = [x1, y1]
	pointList = []
	if x1 < x2:
		if slope >= 0:
			# Real Bresenham Algorithm
			while True:
				pointList.append([x, y])
				if x == x2:
					return pointList
				if p <= 0:
					[x, y, p] = [x + 1, y, p + 2 * slope]
				else:
					[x, y, p] = [x + 1, y + 1, p + 2 * slope - 2]
		else: # Up-Down Symmetry
			pointList = BresenhamLine(x1, -y1, x2, -y2)
			return [[p[0], -p[1]] for p in pointList]
	else:# Left-Right Symmetry
		pointList = BresenhamLine(x2, y2, x1, y1)
		pointList.reverse()
		return pointList

# 图节点的定义
class Node:
    def __init__(self, px, py, vx, vy):
        # state
        # 状态，包含速度和位置
        self.px = px
        self.py = py
        self.vx = vx
        self.vy = vy
        # value
        # 代价值
        self.g_value = 0.0
        # successor
        # 环境干扰
        self.next_prob_9 = []
        self.next_prob_1 = []
        # key
        self.key = self.get_key()
        # 判断是否为目标
        self.is_goal = False
    


    # 根据位置和速度生成唯一的key
    @staticmethod
    def generate_key(px, py, vx, vy):
        return "%02d" % px + "%02d" % py + "%02d" % vx + "%02d" % vy

    # 获取每个状态对应的key
    def get_key(self):
        return self.generate_key(self.px, self.py, self.vx, self.vy)


    def connect_to_graph(self, grid):
        # 遍历所有输入空间
        for u in ACTION_SPACE:
            self.next_prob_9.append(self.control(u[0], u[1], grid, success=True))
            self.next_prob_1.append(self.control(u[0], u[1], grid, success=False))


    @staticmethod
    def velocity_constraints(vx, vy):
        return np.sign(vx) * min(abs(vx), 4), np.sign(vy) * min(abs(vy), 4)


    # 判断是否符合安全限制
    def safety_constraints(self, px2, py2, grid):
        assert  0 <= self.px < grid.shape[0]
        assert  0 <= self.py < grid.shape[1]

        # way_points = np.asarray(Bresenham(self.px, self.py, px2, py2)).astype(np.int)
        # print(points)

        x_dist = np.abs(px2 - self.px)
        y_dist = np.abs(py2 - self.py)


        step = max(x_dist, y_dist)
        x_way_points = np.linspace(self.px, px2, step + 1, endpoint=True)
        y_way_points = np.linspace(self.py, py2, step + 1, endpoint=True)
        way_points = np.stack([np.ceil(x_way_points), np.ceil(y_way_points)], axis=1).astype(np.int)
        # print(way_points)

        for idx in range(way_points.shape[0]):
            point = way_points[idx]
            if (0 <= point[0] < grid.shape[0]) and (0 <= point[1] < grid.shape[1]):
                if grid[point[0], point[1]] == FINISH:
                    return FINISH, point
                elif grid[point[0], point[1]] == OCCUPIED:
                    return OCCUPIED, point
                # else:
                #  free and start: continue
            else:
                return OUTBOUND, point

        if grid[way_points[-1][0], way_points[-1][1]] == START:
            return START, way_points[-1]
        else:
            return FREE, way_points[-1]
    # end definition

    # 对当前状态施加控制
    def control(self, ux, uy, grid, success):
        assert ux in action_assert_list
        assert uy in action_assert_list

        # success with probability of 0.9
        if not success:
            ux = 0
            uy = 0

        # 系统的动态模型
        # dynamic model
        vx = self.vx + ux
        vy = self.vy + uy
        vx, vy = self.velocity_constraints(vx, vy)
        px = self.px + vx
        py = self.py + vy

        # check collision
        status, point = self.safety_constraints(px, py, grid)
        if status == FREE:
            assert px == point[0] and py == point[1]
            return self.generate_key(px, py, vx, vy)
        elif status == START:
            assert grid[point[0], point[1]] == START
            assert px == point[0] and py == point[1]
            return self.generate_key(point[0], point[1], 0, 0)
        elif status == FINISH:
            assert grid[point[0], point[1]] == FINISH
            return self.generate_key(point[0], point[1], 0, 0)
        else: # out of bound or occupied
            assert status == OUTBOUND or status == OCCUPIED
            rand_start = START_LINE[np.random.randint(low=0, high=3, size=1)[0]]
            return self.generate_key(rand_start[0], rand_start[1], 0, 0)