"""Little Navmap scenery-database provider.

Little Navmap compiles its database from the *installed* scenery, which makes
it the most accurate source available for this job: the runway thresholds and
ILS frequencies it reports are the ones the aeroplane will actually fly to and
tune, including third-party airports the public datasets have never heard of.
Most simmers already have it.

The database usually lives at::

    %APPDATA%\\ABarthel\\little_navmap_db\\little_navmap_msfs24.sqlite
    %APPDATA%\\ABarthel\\little_navmap_db\\little_navmap_msfs.sqlite

It is opened strictly read-only through a SQLite URI, so a running Little
Navmap is never disturbed.

The schema is not a stable published contract, so rather than assuming column
names this provider inspects the tables it finds and adapts, and reports a
clear reason when a table it needs is missing instead of raising SQL errors
from somewhere deep in a query.
"""

from __future__ import annotations

import os
import sqlite3
from typing import Optional

from ..geo import LatLon, normalize_deg
from .base import Airport, NavDataProvider, Runway, Waypoint

#: Little Navmap's database file names, newest simulator first.
DEFAULT_DB_NAMES = (
    "little_navmap_msfs24.sqlite",
    "little_navmap_msfs_2024.sqlite",
    "little_navmap_msfs.sqlite",
)

#: Which of those belong to which simulator. Someone with both installed has
#: both files, and flying 2020 against the 2024 database means approaches built
#: to runways that moved between the two.
DB_NAMES_BY_SIM = {
    "2024": ("little_navmap_msfs24.sqlite", "little_navmap_msfs_2024.sqlite"),
    "2020": ("little_navmap_msfs.sqlite",),
}


def database_names_for(msfs_version: Optional[str] = None) -> tuple[str, ...]:
    """Database names to try, with the chosen simulator's first."""
    if not msfs_version:
        return DEFAULT_DB_NAMES
    preferred = DB_NAMES_BY_SIM.get(str(msfs_version), ())
    return preferred + tuple(n for n in DEFAULT_DB_NAMES if n not in preferred)


def default_database_paths(msfs_version: Optional[str] = None) -> list[str]:
    """Every place a Little Navmap scenery database is likely to be."""
    roots = []
    appdata = os.environ.get("APPDATA")
    if appdata:
        roots.append(os.path.join(appdata, "ABarthel", "little_navmap_db"))
    home = os.path.expanduser("~")
    roots.append(os.path.join(home, ".config", "ABarthel", "little_navmap_db"))
    roots.append(os.path.join(home, "AppData", "Roaming", "ABarthel", "little_navmap_db"))
    found = []
    for root in roots:
        for name in database_names_for(msfs_version):
            candidate = os.path.join(root, name)
            if os.path.isfile(candidate):
                found.append(candidate)
    return found


class LittleNavmapProvider(NavDataProvider):
    """Read-only access to a Little Navmap scenery database."""

    name = "littlenavmap"

    def __init__(self, db_path: str) -> None:
        self.db_path = db_path
        self._conn: Optional[sqlite3.Connection] = None
        self._columns: dict[str, set[str]] = {}
        self._error: Optional[str] = None

    @property
    def available(self) -> bool:
        return os.path.isfile(self.db_path)

    def describe(self) -> str:
        return f"littlenavmap({os.path.basename(self.db_path)})"

    @property
    def error(self) -> Optional[str]:
        return self._error

    def _connect(self) -> Optional[sqlite3.Connection]:
        if self._conn is not None:
            return self._conn
        if not self.available:
            self._error = f"No database at {self.db_path}"
            return None
        try:
            uri = "file:" + self.db_path.replace("?", "%3f").replace("#", "%23") + "?mode=ro"
            self._conn = sqlite3.connect(uri, uri=True, check_same_thread=False)
            self._conn.row_factory = sqlite3.Row
        except sqlite3.Error as exc:
            self._error = f"Could not open the database: {exc}"
            return None
        for table in ("airport", "runway", "runway_end", "ils", "waypoint", "vor", "ndb"):
            self._columns[table] = self._table_columns(table)
        if not self._columns.get("airport"):
            self._error = "This SQLite file has no 'airport' table -- is it a Little Navmap database?"
            self._conn.close()
            self._conn = None
        return self._conn

    def _table_columns(self, table: str) -> set[str]:
        assert self._conn is not None
        try:
            rows = self._conn.execute(f"PRAGMA table_info({table})").fetchall()
        except sqlite3.Error:
            return set()
        return {row["name"] for row in rows}

    def _pick(self, table: str, *candidates: str) -> Optional[str]:
        """First column name that exists, so schema drift does not break us."""
        columns = self._columns.get(table, set())
        for candidate in candidates:
            if candidate in columns:
                return candidate
        return None

    def close(self) -> None:
        if self._conn is not None:
            self._conn.close()
            self._conn = None

    # -- Queries -------------------------------------------------------------
    def airport(self, icao: str) -> Optional[Airport]:
        conn = self._connect()
        if conn is None:
            return None
        ident_col = self._pick("airport", "ident", "icao")
        lat_col = self._pick("airport", "laty", "lat")
        lon_col = self._pick("airport", "lonx", "lon")
        alt_col = self._pick("airport", "altitude", "elevation")
        if not (ident_col and lat_col and lon_col):
            self._error = "The airport table is missing the columns we need."
            return None
        magvar_col = self._pick("airport", "mag_var", "magvar")
        name_col = self._pick("airport", "name")
        columns = ", ".join(
            c for c in ("airport_id", ident_col, lat_col, lon_col, alt_col, magvar_col, name_col)
            if c
        )
        try:
            row = conn.execute(
                f"SELECT {columns} FROM airport WHERE {ident_col} = ? COLLATE NOCASE LIMIT 1",
                (icao.strip().upper(),),
            ).fetchone()
        except sqlite3.Error as exc:
            self._error = str(exc)
            return None
        if row is None:
            return None
        elevation = float(row[alt_col]) if alt_col and row[alt_col] is not None else 0.0
        return Airport(
            icao=str(row[ident_col]).upper(),
            name=str(row[name_col]) if name_col and row[name_col] else "",
            position=LatLon(float(row[lat_col]), float(row[lon_col])),
            elevation_ft=elevation,
            magvar_deg=float(row[magvar_col]) if magvar_col and row[magvar_col] is not None else 0.0,
            runways=self._runways(conn, row["airport_id"], elevation),
        )

    def _runways(self, conn: sqlite3.Connection, airport_id: int,
                 field_elevation: float) -> tuple[Runway, ...]:
        if not self._columns.get("runway") or not self._columns.get("runway_end"):
            return ()
        end_lat = self._pick("runway_end", "laty", "lat")
        end_lon = self._pick("runway_end", "lonx", "lon")
        end_name = self._pick("runway_end", "name", "ident")
        end_heading = self._pick("runway_end", "heading", "true_heading")
        if not (end_lat and end_lon and end_name and end_heading):
            return ()
        length_col = self._pick("runway", "length")
        width_col = self._pick("runway", "width")
        surface_col = self._pick("runway", "surface")
        offset_col = self._pick("runway_end", "offset_threshold")
        alt_col = self._pick("runway_end", "altitude")
        ils_ident_col = self._pick("runway_end", "ils_ident")

        try:
            runway_rows = conn.execute(
                "SELECT * FROM runway WHERE airport_id = ?", (airport_id,)
            ).fetchall()
        except sqlite3.Error as exc:
            self._error = str(exc)
            return ()

        out: list[Runway] = []
        for rw in runway_rows:
            length = float(rw[length_col]) if length_col and rw[length_col] is not None else 0.0
            width = float(rw[width_col]) if width_col and rw[width_col] is not None else 150.0
            surface = str(rw[surface_col]) if surface_col and rw[surface_col] else "unknown"
            for end_key in ("primary_end_id", "secondary_end_id"):
                if end_key not in rw.keys() or rw[end_key] is None:
                    continue
                try:
                    end = conn.execute(
                        "SELECT * FROM runway_end WHERE runway_end_id = ?", (rw[end_key],)
                    ).fetchone()
                except sqlite3.Error:
                    continue
                if end is None:
                    continue
                heading = float(end[end_heading]) if end[end_heading] is not None else 0.0
                ils_ident = end[ils_ident_col] if ils_ident_col and ils_ident_col in end.keys() else None
                freq, course, gs_angle = self._ils(conn, ils_ident, str(end[end_name]))
                out.append(
                    Runway(
                        ident=str(end[end_name]).upper().replace("RW", ""),
                        threshold=LatLon(float(end[end_lat]), float(end[end_lon])),
                        heading_true_deg=normalize_deg(heading),
                        length_ft=length,
                        elevation_ft=float(end[alt_col]) if alt_col and end[alt_col] is not None
                        else field_elevation,
                        width_ft=width,
                        surface=surface,
                        displaced_threshold_ft=float(end[offset_col])
                        if offset_col and end[offset_col] is not None else 0.0,
                        ils_freq_mhz=freq,
                        ils_course_true_deg=course,
                        glideslope_deg=gs_angle,
                    )
                )
        return tuple(out)

    def _ils(self, conn: sqlite3.Connection, ils_ident: Optional[str],
             runway_name: str) -> tuple[Optional[float], Optional[float], float]:
        """Frequency (MHz), true course and glideslope angle for a runway end."""
        if not self._columns.get("ils") or not ils_ident:
            return (None, None, 3.0)
        freq_col = self._pick("ils", "frequency")
        course_col = self._pick("ils", "loc_heading", "heading")
        pitch_col = self._pick("ils", "gs_pitch", "glideslope_pitch")
        ident_col = self._pick("ils", "ident")
        if not (freq_col and ident_col):
            return (None, None, 3.0)
        try:
            row = conn.execute(
                f"SELECT * FROM ils WHERE {ident_col} = ? LIMIT 1", (ils_ident,)
            ).fetchone()
        except sqlite3.Error:
            return (None, None, 3.0)
        if row is None:
            return (None, None, 3.0)
        raw_freq = row[freq_col]
        if raw_freq is None:
            return (None, None, 3.0)
        # Little Navmap stores frequencies in whole kHz (110300 == 110.30 MHz).
        freq = float(raw_freq)
        if freq > 1000.0:
            freq /= 1000.0
        course = float(row[course_col]) if course_col and row[course_col] is not None else None
        pitch = float(row[pitch_col]) if pitch_col and row[pitch_col] is not None else 3.0
        return (freq, normalize_deg(course) if course is not None else None, pitch or 3.0)

    def waypoint(self, ident: str, near: Optional[LatLon] = None) -> Optional[Waypoint]:
        conn = self._connect()
        if conn is None:
            return None
        best: Optional[Waypoint] = None
        best_distance = float("inf")
        from ..geo import distance_nm

        for table, kind in (("waypoint", "fix"), ("vor", "vor"), ("ndb", "ndb")):
            if not self._columns.get(table):
                continue
            ident_col = self._pick(table, "ident")
            lat_col = self._pick(table, "laty", "lat")
            lon_col = self._pick(table, "lonx", "lon")
            if not (ident_col and lat_col and lon_col):
                continue
            try:
                rows = conn.execute(
                    f"SELECT {ident_col}, {lat_col}, {lon_col} FROM {table} "
                    f"WHERE {ident_col} = ? COLLATE NOCASE",
                    (ident.strip().upper(),),
                ).fetchall()
            except sqlite3.Error:
                continue
            for row in rows:
                point = Waypoint(str(row[ident_col]).upper(),
                                 LatLon(float(row[lat_col]), float(row[lon_col])), kind)
                if near is None:
                    return point
                d = distance_nm(near, point.position)
                if d < best_distance:
                    best, best_distance = point, d
        return best
