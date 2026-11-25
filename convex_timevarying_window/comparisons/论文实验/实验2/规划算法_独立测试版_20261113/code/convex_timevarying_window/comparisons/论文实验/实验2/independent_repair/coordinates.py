"""An exact change from log durations to absolute arrival times inside SQP."""
import numpy as np


class ArrivalCoordinates:
    def __init__(self, origin, scale, pieces, bounds):
        self.origin = np.asarray(origin)
        self.scale = np.asarray(scale)
        self.pieces = pieces
        bounds = np.asarray(bounds)
        self.duration_lower = np.exp(self.origin[:pieces]+self.scale[:pieces]*bounds[:pieces, 0])
        self.duration_upper = np.exp(self.origin[:pieces]+self.scale[:pieces]*bounds[:pieces, 1])
        difference = np.eye(pieces)-np.eye(pieces, k=-1)
        self.difference = np.pad(difference, ((0, 0), (0, len(origin)-pieces)))
        self.bounds = list(zip(np.cumsum(self.duration_lower), np.cumsum(self.duration_upper)))+list(map(tuple, bounds[pieces:]))

    def pack(self, y):
        result = np.array(y, copy=True)
        result[:self.pieces] = np.cumsum(np.exp(self.origin[:self.pieces]+self.scale[:self.pieces]*result[:self.pieces]))
        return result

    def unpack(self, z):
        duration = self.difference@z
        # An extension for invalid QP trials. Separate linear inequalities
        # enforce the original positive duration bounds on accepted results.
        guarded = np.maximum(duration, 1e-8)
        y = np.array(z, copy=True)
        y[:self.pieces] = (np.log(guarded)-self.origin[:self.pieces])/self.scale[:self.pieces]
        jacobian = np.eye(len(z))
        jacobian[:self.pieces] = self.difference*((duration > 1e-8)/(guarded*self.scale[:self.pieces]))[:, None]
        return y, jacobian

    def margins(self, z):
        duration = self.difference@z
        return np.r_[duration-self.duration_lower, self.duration_upper-duration]

    def margin_jacobian(self, z):
        return np.vstack((self.difference, -self.difference))
