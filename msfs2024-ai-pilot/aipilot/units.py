"""Unit constants and conversions.

Everything internal to the AI Pilot uses these canonical units:

    distance   nautical miles (nm)
    altitude   feet (ft)
    speed      knots (kt)
    vertical    feet per minute (fpm)
    angles     degrees (deg), true unless the name says ``_mag``
    time       seconds (s)

SimConnect hands us a grab-bag of radians, metres and metres/second, so all
conversion happens at the backend boundary (see :mod:`aipilot.sim`) and never
leaks into the guidance code.
"""

from __future__ import annotations

import math

# --- Length -----------------------------------------------------------------
FT_PER_NM = 6076.11548556
M_PER_NM = 1852.0
M_PER_FT = 0.3048
FT_PER_M = 1.0 / M_PER_FT

# --- Speed ------------------------------------------------------------------
MPS_PER_KT = 0.514444444
KT_PER_MPS = 1.0 / MPS_PER_KT
FPM_PER_MPS = 196.850393701

# --- Earth ------------------------------------------------------------------
EARTH_RADIUS_NM = 3440.06479482
STD_TEMP_C = 15.0
STD_LAPSE_C_PER_FT = 0.0019812  # 1.98 degC per 1000 ft
SPEED_OF_SOUND_SL_KT = 661.4788


def m_to_nm(m: float) -> float:
    return m / M_PER_NM


def nm_to_m(nm: float) -> float:
    return nm * M_PER_NM


def m_to_ft(m: float) -> float:
    return m * FT_PER_M


def ft_to_m(ft: float) -> float:
    return ft * M_PER_FT


def mps_to_kt(mps: float) -> float:
    return mps * KT_PER_MPS


def kt_to_mps(kt: float) -> float:
    return kt * MPS_PER_KT


def mps_to_fpm(mps: float) -> float:
    return mps * FPM_PER_MPS


def rad_to_deg(rad: float) -> float:
    return math.degrees(rad)


def deg_to_rad(deg: float) -> float:
    return math.radians(deg)


def nm_to_ft(nm: float) -> float:
    return nm * FT_PER_NM


def ft_to_nm(ft: float) -> float:
    return ft / FT_PER_NM


# --- Atmosphere -------------------------------------------------------------
def isa_temp_c(altitude_ft: float) -> float:
    """ISA static air temperature, held constant above the tropopause."""
    if altitude_ft <= 36089.0:
        return STD_TEMP_C - STD_LAPSE_C_PER_FT * altitude_ft
    return -56.5


def speed_of_sound_kt(altitude_ft: float) -> float:
    temp_k = isa_temp_c(altitude_ft) + 273.15
    return SPEED_OF_SOUND_SL_KT * math.sqrt(temp_k / 288.15)


def mach_to_tas(mach: float, altitude_ft: float) -> float:
    return mach * speed_of_sound_kt(altitude_ft)


def tas_to_mach(tas_kt: float, altitude_ft: float) -> float:
    return tas_kt / speed_of_sound_kt(altitude_ft)


def _density_ratio(altitude_ft: float) -> float:
    """Ratio of local to sea-level density under ISA (sigma)."""
    if altitude_ft <= 36089.0:
        return (1.0 - 6.87535e-6 * altitude_ft) ** 4.2561
    return 0.297076 * math.exp((36089.0 - altitude_ft) / 20806.0)


def cas_to_tas(cas_kt: float, altitude_ft: float) -> float:
    """Low-speed approximation: TAS = CAS / sqrt(sigma).

    Good to a couple of percent through the airliner envelope, which is well
    inside the tolerance of everything that consumes it here (speed targets are
    ultimately handed to the aircraft's own autothrottle as a CAS or Mach
    number, so this is only ever used for time and distance estimates).
    """
    return cas_kt / math.sqrt(_density_ratio(altitude_ft))


def tas_to_cas(tas_kt: float, altitude_ft: float) -> float:
    return tas_kt * math.sqrt(_density_ratio(altitude_ft))


def crossover_altitude_ft(cas_kt: float, mach: float) -> float:
    """Altitude where a CAS schedule becomes a Mach schedule.

    Found by bisection rather than the closed form: the closed form needs the
    compressible relations, and a dozen bisection steps costs nothing on a
    once-per-flight computation.
    """
    lo, hi = 0.0, 45000.0
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        if tas_to_mach(cas_to_tas(cas_kt, mid), mid) < mach:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)
