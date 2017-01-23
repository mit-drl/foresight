
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
