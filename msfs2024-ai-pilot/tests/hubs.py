"""Major airports of the world, as test fixtures.

Coordinates, elevations and one runway pair each, chosen to exercise the
geometry rather than to be a chart. The set is picked for the awkward cases as
much as for the traffic: fields two miles below and two miles above where most
of the world sits, airports either side of the antimeridian and inside both
polar circles, and the extremes of magnetic variation.

Runway headings are TRUE, computed from the designator and the local magnetic
variation at each field, because that is the frame everything downstream uses.
Lengths and elevations are real to the nearest sensible figure. Nothing here is
survey data and none of it should be flown from.
"""

from __future__ import annotations

from dataclasses import dataclass

from aipilot.geo import LatLon, destination_point, normalize_deg
from aipilot.navdata.base import Airport, Runway
from aipilot.units import FEET_PER_NM


@dataclass(frozen=True)
class Hub:
    icao: str
    name: str
    lat: float
    lon: float
    elevation_ft: float
    magvar_deg: float          # east positive
    runway: str                # lower-numbered end
    length_ft: float
    region: str


#: (icao, name, lat, lon, elevation, magvar, runway, length, region)
HUBS: tuple[Hub, ...] = tuple(Hub(*row) for row in [
    # --- North America -------------------------------------------------------
    ("KATL", "Atlanta", 33.6367, -84.4281, 1026, -5.8, "09L", 12390, "NAM"),
    ("KJFK", "New York Kennedy", 40.6398, -73.7789, 13, -13.2, "04L", 12079, "NAM"),
    ("KLAX", "Los Angeles", 33.9425, -118.4081, 125, 11.6, "06L", 8926, "NAM"),
    ("KORD", "Chicago O'Hare", 41.9786, -87.9048, 672, -3.5, "10L", 13000, "NAM"),
    ("KDFW", "Dallas Fort Worth", 32.8968, -97.0380, 607, 2.7, "17C", 13401, "NAM"),
    ("KDEN", "Denver", 39.8617, -104.6732, 5431, 8.2, "16L", 12000, "NAM"),
    ("KSFO", "San Francisco", 37.6189, -122.3750, 13, 13.3, "10L", 11870, "NAM"),
    ("KSEA", "Seattle Tacoma", 47.4489, -122.3094, 433, 15.4, "16L", 11901, "NAM"),
    ("KMIA", "Miami", 25.7932, -80.2906, 8, -7.0, "08L", 8600, "NAM"),
    ("CYYZ", "Toronto Pearson", 43.6777, -79.6248, 569, -10.3, "05", 11050, "NAM"),
    ("CYVR", "Vancouver", 49.1939, -123.1844, 14, 15.8, "08L", 9940, "NAM"),
    ("MMMX", "Mexico City", 19.4363, -99.0721, 7316, 4.5, "05L", 12966, "NAM"),
    ("PANC", "Anchorage", 61.1743, -149.9962, 152, 15.2, "15", 10600, "NAM"),
    # --- South America -------------------------------------------------------
    ("SBGR", "Sao Paulo Guarulhos", -23.4356, -46.4731, 2461, -21.5, "09R", 12139, "SAM"),
    ("SAEZ", "Buenos Aires Ezeiza", -34.8222, -58.5358, 67, -8.5, "11", 10827, "SAM"),
    ("SKBO", "Bogota", 4.7016, -74.1469, 8361, -7.0, "13L", 12467, "SAM"),
    ("SPJC", "Lima", -12.0219, -77.1143, 113, -2.0, "16", 11510, "SAM"),
    ("SCEL", "Santiago", -33.3930, -70.7858, 1555, 1.5, "17L", 12303, "SAM"),
    # --- Europe --------------------------------------------------------------
    ("EGLL", "London Heathrow", 51.4706, -0.4619, 83, 0.6, "09L", 12802, "EUR"),
    ("LFPG", "Paris Charles de Gaulle", 49.0097, 2.5479, 392, 1.0, "08L", 13829, "EUR"),
    ("EHAM", "Amsterdam Schiphol", 52.3086, 4.7639, -11, 2.4, "18R", 12467, "EUR"),
    ("EDDF", "Frankfurt", 50.0333, 8.5706, 364, 3.3, "07C", 13123, "EUR"),
    ("LEMD", "Madrid Barajas", 40.4719, -3.5626, 1998, -0.6, "14L", 14468, "EUR"),
    ("LIRF", "Rome Fiumicino", 41.8003, 12.2389, 15, 4.0, "16L", 12795, "EUR"),
    ("LTFM", "Istanbul", 41.2753, 28.7519, 325, 6.2, "16L", 13451, "EUR"),
    ("UUEE", "Moscow Sheremetyevo", 55.9726, 37.4146, 622, 12.4, "06L", 12139, "EUR"),
    ("ENGM", "Oslo Gardermoen", 60.1939, 11.1004, 681, 5.5, "01L", 11811, "EUR"),
    ("BIKF", "Keflavik", 63.9850, -22.6056, 171, -13.5, "10", 10056, "EUR"),
    ("ENSB", "Svalbard Longyearbyen", 78.2461, 15.4656, 88, 8.0, "10", 7605, "EUR"),
    # --- Middle East and Africa ----------------------------------------------
    ("OMDB", "Dubai", 25.2528, 55.3644, 62, 2.0, "12L", 13124, "MEA"),
    ("OTHH", "Doha Hamad", 25.2731, 51.6081, 13, 2.4, "16L", 15912, "MEA"),
    ("OERK", "Riyadh", 24.9576, 46.6988, 2049, 2.8, "15L", 13780, "MEA"),
    ("LLBG", "Tel Aviv", 32.0114, 34.8867, 135, 4.6, "12", 11982, "MEA"),
    ("HECA", "Cairo", 30.1219, 31.4056, 382, 4.7, "05L", 10827, "MEA"),
    ("FAOR", "Johannesburg", -26.1392, 28.2460, 5558, -18.5, "03L", 14495, "MEA"),
    ("HAAB", "Addis Ababa", 8.9779, 38.7993, 7625, 1.5, "07L", 12467, "MEA"),
    ("DNMM", "Lagos", 6.5774, 3.3212, 135, -2.5, "18L", 12795, "MEA"),
    ("FACT", "Cape Town", -33.9648, 18.6017, 151, -26.0, "01", 10502, "MEA"),
    # --- Asia ----------------------------------------------------------------
    ("VHHH", "Hong Kong", 22.3089, 113.9145, 28, -2.8, "07L", 12467, "ASI"),
    ("RJTT", "Tokyo Haneda", 35.5533, 139.7811, 21, -7.7, "16L", 9843, "ASI"),
    ("RKSI", "Seoul Incheon", 37.4691, 126.4505, 23, -8.4, "15L", 12303, "ASI"),
    ("ZBAA", "Beijing Capital", 40.0801, 116.5846, 116, -7.0, "18L", 12467, "ASI"),
    ("ZSPD", "Shanghai Pudong", 31.1434, 121.8052, 13, -5.4, "16L", 13123, "ASI"),
    ("WSSS", "Singapore Changi", 1.3502, 103.9944, 22, 0.2, "02L", 13123, "ASI"),
    ("VTBS", "Bangkok Suvarnabhumi", 13.6900, 100.7501, 5, -0.5, "01L", 13123, "ASI"),
    ("VIDP", "Delhi", 28.5665, 77.1031, 777, 0.9, "09", 12500, "ASI"),
    ("VABB", "Mumbai", 19.0887, 72.8679, 39, 0.4, "09", 11447, "ASI"),
    ("WIII", "Jakarta", -6.1256, 106.6559, 34, 0.6, "07L", 12008, "ASI"),
    ("VNKT", "Kathmandu", 27.6966, 85.3591, 4390, 0.4, "02", 10007, "ASI"),
    # --- Oceania -------------------------------------------------------------
    ("YSSY", "Sydney", -33.9461, 151.1772, 21, 12.7, "16R", 12999, "OCE"),
    ("YMML", "Melbourne", -37.6733, 144.8433, 434, 11.6, "16", 11998, "OCE"),
    ("NZAA", "Auckland", -37.0081, 174.7917, 23, 19.8, "05R", 11926, "OCE"),
    ("NFFN", "Nadi", -17.7554, 177.4434, 59, 12.3, "02", 10500, "OCE"),
    ("PHNL", "Honolulu", 21.3187, -157.9224, 13, 9.7, "08L", 12300, "OCE"),
    ("NZCM", "McMurdo Phoenix", -77.9575, 166.7450, 30, 137.0, "15", 10000, "OCE"),
])

HUBS_BY_ICAO = {hub.icao: hub for hub in HUBS}


def _runway_pair(hub: Hub) -> tuple[Runway, ...]:
    """The named runway and its reciprocal, as true headings.

    Runway designators are magnetic, so the variation has to come off to get
    the true heading everything downstream works in. At Cape Town that is
    twenty-six degrees; at McMurdo the magnetic frame is meaningless, which is
    the point of including it.
    """
    digits = "".join(c for c in hub.runway if c.isdigit())
    side = hub.runway[len(digits):]
    magnetic = float(digits) * 10.0
    heading = normalize_deg(magnetic + hub.magvar_deg)
    threshold = LatLon(hub.lat, hub.lon)
    length_nm = hub.length_ft / FEET_PER_NM
    far_end = destination_point(threshold, heading, length_nm)

    reciprocal_number = int(digits) + 18
    if reciprocal_number > 36:
        reciprocal_number -= 36
    other_side = {"L": "R", "R": "L", "C": "C", "": ""}[side]

    return (
        Runway(hub.runway, threshold, heading, hub.length_ft, hub.elevation_ft,
               width_ft=150.0, ils_freq_mhz=110.30,
               ils_course_true_deg=heading),
        Runway(f"{reciprocal_number:02d}{other_side}", far_end,
               normalize_deg(heading + 180.0), hub.length_ft, hub.elevation_ft,
               width_ft=150.0, ils_freq_mhz=110.70,
               ils_course_true_deg=normalize_deg(heading + 180.0)),
    )


def airport(icao: str) -> Airport:
    hub = HUBS_BY_ICAO[icao.strip().upper()]
    return Airport(hub.icao, hub.name, LatLon(hub.lat, hub.lon),
                   hub.elevation_ft, magvar_deg=hub.magvar_deg,
                   runways=_runway_pair(hub))
