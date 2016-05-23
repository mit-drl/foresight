
import shapely.geometry


class Point(shapely.geometry.Point):

    def __len__(self):
        if self.has_z:
            return 3
        else:
            return 2

    def __getitem__(self, i):
        if i == 0:
            return self.x
        elif i == 1:
            return self.y
        elif self.has_z and i == 2:
            return self.z
        else:
            raise IndexError()

    def __repr__(self):
        if self.has_z:
            return "Point({}, {}, {})".format(self.x, self.y, self.z)
        else:
            return "Point({}, {})".format(self.x, self.y)

    def __str__(self):
        return repr(self)

    def __hash__(self):
        return hash(str(self))


class SpaceHeapValue(object):

    def set_point(self, point):
        self.point = point
        return self

    def set_area(self, area):
        self.area = area
        return self

    def set_polygons(self, polys):
        self.polys = polys
        return self

    def set_current_time(self, ct):
        self.ct = ct
        return self

    def set_yaw(self, yaw):
        self.yaw = yaw
        return self

    def get_point(self):
        return self.point

    def get_area(self):
        return self.area

    def get_polygons(self):
        return self.polys

    def get_current_time(self):
        return self.ct

    def get_yaw(self):
        return self.yaw

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
