"""Two intrinsic point coordinates, same convex aperture interior.

Polygon radial gauge is a bijection R² -> interior(P). It avoids redundant
normalized barycentric variables in the native hard-constrained solve. Circle
retains the original TOGT disk map. This is an explicit parameterization change,
not a change of the aperture or of the physical optimization objective.
"""
import casadi as ca
import numpy as np
from ..geometry import halfplanes


def symbolic(aperture,z):
    if aperture.kind=='circle':return 2*aperture.radius*z/(1+ca.sumsqr(z))
    n,b=halfplanes(aperture)
    gauge=ca.mmax((ca.DM(n)@z)/ca.DM(b))
    return z/(1+gauge)


def inverse(aperture,p):
    p=np.asarray(p)
    if aperture.kind=='circle':
        radius=aperture.radius
        return p/(radius+np.sqrt(max(0.,radius**2-p@p)))
    n,b=halfplanes(aperture);gauge=np.max(n@p/b)
    if gauge>=1:raise ValueError('Inverse requires an interior point')
    return p/(1-gauge)
