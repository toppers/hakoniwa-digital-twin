class InfraSensorLidarFilter:
    def __init__(self, max, threshold, f_range):
        self.max_deg = max
        self.threshold_intensity = threshold
        self.filter_range = f_range

    def filter_ranges(self, intensities, ranges):
        degrees = []
        values = []
        i = 0
        while i < self.max_deg:
            if intensities[i] > self.threshold_intensity and ranges[i] > 0.0:
                if ranges[i] < self.filter_range:
                    degrees.append(i)
                    values.append(ranges[i])
                    print(f"{i} {ranges[i]} {intensities[i]}")
            i = i + 1
        return degrees, values
