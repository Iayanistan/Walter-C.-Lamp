"""ASTM D4607 section 11 (calculation) and 11.4 (carbon dosage estimate).

Symbols follow the standard:
    N1  sodium thiosulfate normality (from 9.1)        N2  iodine normality (from 9.2)
    I   iodine added, mL (100.0)                        H   5% HCl added, mL (10.0)
    F   filtrate pipetted for titration, mL (50.0)      S   thiosulfate used in the 10.6 titration, mL
    M   carbon mass, g
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

C_TARGET = 0.02          # 11.3: iodine number is X/M at a residual concentration of 0.02 N
C_RANGE = (0.008, 0.040)  # 11.1: filtrate normalities outside this range mean re-run with other masses
R_MIN = 0.995            # 11.3: least-squares regression coefficient should exceed this


def a_value(n2: float) -> float:            # eq. 3
    return n2 * 12693.0


def b_value(n1: float) -> float:            # eq. 4
    return n1 * 126.93


def dilution_factor(i_ml: float = 100.0, h_ml: float = 10.0, f_ml: float = 50.0) -> float:   # eq. 5
    return (i_ml + h_ml) / f_ml


def x_over_m(n1, n2, s_ml, m_g, i_ml=100.0, h_ml=10.0, f_ml=50.0) -> float:   # eq. 6, mg/g
    return (a_value(n2) - dilution_factor(i_ml, h_ml, f_ml) * b_value(n1) * s_ml) / m_g


def residual_c(n1, s_ml, f_ml=50.0) -> float:     # eq. 7, N
    return n1 * s_ml / f_ml


def carbon_dosage(n2, estimated_iodine_number, c, i_ml=100.0, h_ml=10.0, f_ml=50.0) -> float:  # eq. 8, g
    df = dilution_factor(i_ml, h_ml, f_ml)
    return (a_value(n2) - df * c * 126.93 * f_ml) / estimated_iodine_number


def dosages(n2, estimated_iodine_number, cs=(0.01, 0.02, 0.03), **kw) -> list[float]:
    """11.4: the three carbon masses to weigh, usually for C = 0.01, 0.02 and 0.03 N."""
    return [carbon_dosage(n2, estimated_iodine_number, c, **kw) for c in cs]


@dataclass
class Isotherm:
    points: list[tuple[float, float]]   # (C, X/M) per flask
    slope: float                        # of log10(X/M) vs log10(C)
    intercept: float
    r: float                            # correlation coefficient of the log-log fit
    iodine_number: float                # X/M at C = 0.02 N, mg/g
    warnings: list[str]

    def as_dict(self):
        return {"points": [{"C_N": c, "X_over_M_mg_per_g": xm} for c, xm in self.points],
                "slope": self.slope, "intercept": self.intercept, "r": self.r,
                "iodine_number_mg_per_g": self.iodine_number, "warnings": self.warnings}


def fit_isotherm(points: list[tuple[float, float]]) -> Isotherm:
    """11.3: least-squares fit of log(X/M) against log(C); iodine number = X/M at C = 0.02 N."""
    if len(points) < 2:
        raise ValueError("need at least two (C, X/M) points")
    c = np.array([p[0] for p in points], float)
    xm = np.array([p[1] for p in points], float)
    if np.any(c <= 0) or np.any(xm <= 0):
        raise ValueError("C and X/M must be positive to take logs")
    lx, ly = np.log10(c), np.log10(xm)
    slope, intercept = np.polyfit(lx, ly, 1)
    r = float(np.corrcoef(lx, ly)[0, 1])
    value = float(10 ** (intercept + slope * np.log10(C_TARGET)))
    warnings = []
    for ci in c:
        if not C_RANGE[0] <= ci <= C_RANGE[1]:
            warnings.append(f"C = {ci:.4f} N is outside {C_RANGE[0]} to {C_RANGE[1]} N: repeat with a different carbon mass (11.1)")
    if r <= R_MIN:
        warnings.append(f"regression coefficient {r:.4f} is not above {R_MIN} (11.3)")
    return Isotherm(list(zip(c.tolist(), xm.tolist())), float(slope), float(intercept), r, value, warnings)


def iodine_number(n1, n2, flasks: list[dict], i_ml=100.0, h_ml=10.0, f_ml=50.0) -> Isotherm:
    """flasks: [{'mass_g': M, 'thio_ml': S}, ...] -> fitted isotherm (sections 11.2 and 11.3)."""
    pts = [(residual_c(n1, f["thio_ml"], f_ml), x_over_m(n1, n2, f["thio_ml"], f["mass_g"], i_ml, h_ml, f_ml))
           for f in flasks]
    return fit_isotherm(pts)
