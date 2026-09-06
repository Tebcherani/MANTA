"""
Salinity, density, and sound velocity computations.

Wraps the modern `gsw` (TEOS-10) library where available, with a self-contained
UNESCO-1980 fallback so the package works in minimal environments. The two
formulations agree to ~0.01 psu / ~0.02 kg/m^3 for typical AUV operating
ranges, which is well below sensor noise.

The original MATLAB used `salsp.m` (Practical Salinity Scale 1978) and
`sw_dens` from the CSIRO seawater toolbox.
"""
from __future__ import annotations

import numpy as np

try:
    import gsw  # type: ignore

    _HAVE_GSW = True
except ImportError:  # pragma: no cover
    _HAVE_GSW = False


# UNESCO 1983 conductivity at S=35, T=15, P=0
# Stored in mS/cm to match the CSIRO seawater toolbox convention used in
# the original CTD_process.m: ratio = (10 * cond_S_per_m) / sw_c3515.
SW_C3515 = 42.914  # mS/cm


def _practical_salinity_unesco(c_ratio: np.ndarray, t: np.ndarray, p_dbar: np.ndarray) -> np.ndarray:
    """Practical Salinity from conductivity ratio. UNESCO 1978 / PSS-78.

    c_ratio: conductivity ratio (sample / C3515), dimensionless
    t: in-situ temperature, deg C (ITS-68 ~ ITS-90 to within rounding here)
    p_dbar: pressure, decibars
    """
    a0, a1, a2, a3, a4, a5 = 0.0080, -0.1692, 25.3851, 14.0941, -7.0261, 2.7081
    b0, b1, b2, b3, b4, b5 = 0.0005, -0.0056, -0.0066, -0.0375, 0.0636, -0.0144
    c0, c1, c2, c3, c4 = 0.6766097, 2.00564e-2, 1.104259e-4, -6.9698e-7, 1.0031e-9
    d1, d2, d3, d4 = 3.426e-2, 4.464e-4, 4.215e-1, -3.107e-3
    e1, e2, e3 = 2.070e-5, -6.370e-10, 3.989e-15
    k = 0.0162

    R = np.asarray(c_ratio, dtype=float)
    t = np.asarray(t, dtype=float)
    p = np.asarray(p_dbar, dtype=float)

    rt = c0 + c1 * t + c2 * t**2 + c3 * t**3 + c4 * t**4
    Rp = 1 + (p * (e1 + e2 * p + e3 * p**2)) / (1 + d1 * t + d2 * t**2 + (d3 + d4 * t) * R)
    Rt = R / (Rp * rt)
    sqrtRt = np.sqrt(np.clip(Rt, 0, None))

    s = (
        a0 + a1 * sqrtRt + a2 * Rt + a3 * Rt * sqrtRt + a4 * Rt**2 + a5 * Rt**2 * sqrtRt
    )
    ds = (
        (t - 15.0) / (1 + k * (t - 15.0))
    ) * (b0 + b1 * sqrtRt + b2 * Rt + b3 * Rt * sqrtRt + b4 * Rt**2 + b5 * Rt**2 * sqrtRt)
    return s + ds


def _density_unesco(s: np.ndarray, t: np.ndarray, p_dbar: np.ndarray) -> np.ndarray:
    """In-situ density from salinity, temperature, pressure (UNESCO 1980)."""
    s = np.asarray(s, dtype=float)
    t = np.asarray(t, dtype=float)
    p = np.asarray(p_dbar, dtype=float) / 10.0  # dbar -> bar for K(s,t,p)

    # Pure water density
    rho_w = (
        999.842594
        + 6.793952e-2 * t
        - 9.095290e-3 * t**2
        + 1.001685e-4 * t**3
        - 1.120083e-6 * t**4
        + 6.536332e-9 * t**5
    )
    A = (
        8.24493e-1
        - 4.0899e-3 * t
        + 7.6438e-5 * t**2
        - 8.2467e-7 * t**3
        + 5.3875e-9 * t**4
    )
    B = -5.72466e-3 + 1.0227e-4 * t - 1.6546e-6 * t**2
    C = 4.8314e-4
    rho_st0 = rho_w + A * s + B * s ** 1.5 + C * s**2

    # Secant bulk modulus K(s, t, p)
    Kw = (
        19652.21
        + 148.4206 * t
        - 2.327105 * t**2
        + 1.360477e-2 * t**3
        - 5.155288e-5 * t**4
    )
    Aw = 3.239908 + 1.43713e-3 * t + 1.16092e-4 * t**2 - 5.77905e-7 * t**3
    Bw = 8.50935e-5 - 6.12293e-6 * t + 5.2787e-8 * t**2
    K0 = (
        Kw
        + s * (54.6746 - 0.603459 * t + 1.09987e-2 * t**2 - 6.1670e-5 * t**3)
        + s ** 1.5 * (7.944e-2 + 1.6483e-2 * t - 5.3009e-4 * t**2)
    )
    A_ = (
        Aw
        + s * (2.2838e-3 - 1.0981e-5 * t - 1.6078e-6 * t**2)
        + 1.91075e-4 * s ** 1.5
    )
    B_ = Bw + s * (-9.9348e-7 + 2.0816e-8 * t + 9.1697e-10 * t**2)
    K = K0 + A_ * p + B_ * p**2

    return rho_st0 / (1 - p / K)


def _sound_velocity_chen_millero(s: np.ndarray, t: np.ndarray, p_dbar: np.ndarray) -> np.ndarray:
    """Sound speed from salinity, temperature, pressure (Chen & Millero 1977)."""
    s = np.asarray(s, dtype=float)
    t = np.asarray(t, dtype=float)
    p = np.asarray(p_dbar, dtype=float) / 10.0  # dbar -> bar

    # Pure-water sound speed (Cw)
    c00, c01, c02, c03, c04, c05 = 1402.388, 5.03830, -5.81090e-2, 3.3432e-4, -1.47797e-6, 3.1419e-9
    c10, c11, c12, c13, c14 = 0.153563, 6.8999e-4, -8.1829e-6, 1.3632e-7, -6.1260e-10
    c20, c21, c22, c23, c24 = 3.1260e-5, -1.7111e-6, 2.5986e-8, -2.5353e-10, 1.0415e-12
    c30, c31, c32 = -9.7729e-9, 3.8513e-10, -2.3654e-12

    Cw = (
        (c00 + c01 * t + c02 * t**2 + c03 * t**3 + c04 * t**4 + c05 * t**5)
        + (c10 + c11 * t + c12 * t**2 + c13 * t**3 + c14 * t**4) * p
        + (c20 + c21 * t + c22 * t**2 + c23 * t**3 + c24 * t**4) * p**2
        + (c30 + c31 * t + c32 * t**2) * p**3
    )

    # Salinity correction
    a00, a01, a02, a03, a04 = 1.389, -1.262e-2, 7.166e-5, 2.008e-6, -3.21e-8
    a10, a11, a12, a13, a14 = 9.4742e-5, -1.2580e-5, -6.4885e-8, 1.0507e-8, -2.0122e-10
    a20, a21, a22, a23 = -3.9064e-7, 9.1041e-9, -1.6002e-10, 7.988e-12
    a30, a31, a32 = 1.100e-10, 6.649e-12, -3.389e-13

    A = (
        (a00 + a01 * t + a02 * t**2 + a03 * t**3 + a04 * t**4)
        + (a10 + a11 * t + a12 * t**2 + a13 * t**3 + a14 * t**4) * p
        + (a20 + a21 * t + a22 * t**2 + a23 * t**3) * p**2
        + (a30 + a31 * t + a32 * t**2) * p**3
    )

    b0, b1 = -1.922e-2, -4.42e-5
    b2 = 7.3637e-5
    b3 = 1.7945e-7
    B = b0 + b1 * t + (b2 + b3 * t) * p

    d0, d1 = 1.727e-3, -7.9836e-6
    D = d0 + d1 * p

    return Cw + A * s + B * s ** 1.5 + D * s**2


def salinity_and_sound_velocity(
    cond: np.ndarray, temp: np.ndarray, p_dbar: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """Compute (practical salinity, sound velocity) from C, T, P.

    cond: conductivity in S/m (typical AUV CTD output)
    temp: in-situ temperature in deg C
    p_dbar: pressure in decibars
    """
    cond = np.asarray(cond, dtype=float)
    temp = np.asarray(temp, dtype=float)
    p_dbar = np.asarray(p_dbar, dtype=float)
    c_ratio = 10.0 * cond / SW_C3515  # MATLAB salsp() called with 10*cond

    if _HAVE_GSW:
        # gsw uses the Reference (g/kg, TEOS-10) salinity scale internally,
        # but SP_from_C gives Practical Salinity, which matches PSS-78.
        sp = gsw.SP_from_C(cond * 10.0, temp, p_dbar)  # gsw takes mS/cm
        svel = gsw.sound_speed_t_exact(sp, temp, p_dbar)
        return np.asarray(sp), np.asarray(svel)

    sp = _practical_salinity_unesco(c_ratio, temp, p_dbar)
    svel = _sound_velocity_chen_millero(sp, temp, p_dbar)
    return sp, svel


def density(sal: np.ndarray, temp: np.ndarray, p_dbar: np.ndarray) -> np.ndarray:
    """In-situ density (kg/m^3)."""
    sal = np.asarray(sal, dtype=float)
    temp = np.asarray(temp, dtype=float)
    p_dbar = np.asarray(p_dbar, dtype=float)
    if _HAVE_GSW:
        return np.asarray(gsw.rho_t_exact(sal, temp, p_dbar))
    return _density_unesco(sal, temp, p_dbar)
