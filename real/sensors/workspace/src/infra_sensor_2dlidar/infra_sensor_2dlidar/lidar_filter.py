class InfraSensorLidarFilter:
    def __init__(self, max):
        self.max_deg = max

    def filter_ranges(self, intensities, ranges):
        degrees = []
        values = []
        i = 0
        while i < self.max_deg:
            degrees.append(i)
            values.append(ranges[i])
            i = i + 1
        return degrees, values
