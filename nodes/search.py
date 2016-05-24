
class SpaceHeapValue(object):

    def __init__(self, point, area, polygons, current_time, yaw):
        self.point = point
        self.area = area
        self.polys = polygons
        self.ct = current_time
        self.yaw = yaw

    def __repr__(self):
        return "SpaceHeapValue(area={}, ...)".format(self.area)

    def __str__(self):
        return repr(self)

    def __cmp__(self, other):
        return -cmp(self.area, other.area)

    def __hash__(self):
        return hash("{} {}".format(str(self.point), self.ct))

    def __eq__(self, other):
        return hash(self) == hash(other)


class TreeSearchResult(object):

    def __init__(self, path, optimality, path_exec_time, planner_time):
        self.path = path
        self.optimality = optimality
        self.path_exec_time = path_exec_time
        self.planner_time = planner_time

    def __str__(self):
        tsr_str = """
            TreeSearchResult:
                optimality: {},
                path_exec_time: {},
                planner_time: {}
                """
        return tsr_str.format(self.optimality, self.path_exec_time,
                              self.planner_time)


class Heap(object):

    def __init__(self, hq):
        self.hq = hq
        self.parents = dict()
