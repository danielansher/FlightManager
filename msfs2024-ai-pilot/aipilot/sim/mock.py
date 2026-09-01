"""An offline point-mass simulator that speaks the same protocol as the sim.

This is not a flight model in any serious sense -- there is no aerodynamics in
it. What it *does* reproduce faithfully is the closed loop the AI Pilot flies
against: an autopilot that chases a heading bug at a limited bank angle, an
altitude capture that respects a climb-rate ceiling, an autothrottle with
finite acceleration, wind that produces real drift, and configuration changes
that take time to run.

That is enough to catch the failures that actually matter in guidance code --
waypoints that never sequence, descents that start too late, turns that
overshoot, phase transitions that latch -- and it lets the whole thing be
tested in CI on a machine with no Microsoft Flight Simulator anywhere near it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Optional

from ..geo import LatLon, destination_point, normalize_deg, signed_diff_deg
from ..units import cas_to_tas, mach_to_tas, tas_to_cas, tas_to_mach
from .base import SimBackend, SimCapabilities, SimState, approach_value, clamp


#: Height above which the full free-stream wind is felt.
BOUNDARY_LAYER_FT = 2500.0

#: Fraction of the free-stream wind still blowing at the surface.
SURFACE_WIND_FRACTION = 0.35

#: Height at which an autoland-capable aeroplane rounds out by itself, and the
#: rate it holds through the flare.
AUTOLAND_FLARE_AGL_FT = 50.0
AUTOLAND_FLARE_VS_FPM = -150.0


@dataclass
class MockAircraftModel:
    """Coarse performance envelope for the point-mass integration."""

    max_bank_deg: float = 25.0
    bank_rate_deg_s: float = 5.0
    max_climb_fpm: float = 2600.0
    service_ceiling_ft: float = 43000.0
    max_descent_fpm: float = 3500.0
    accel_kt_s: float = 1.4          # thrust-limited acceleration
    decel_kt_s: float = 1.2          # drag/idle deceleration
    ground_accel_kt_s: float = 3.2
    braking_kt_s: float = 4.5
    rotate_speed_kt: float = 155.0
    flap_transit_s: float = 8.0
    gear_transit_s: float = 10.0
    touchdown_agl_ft: float = 10.0


class MockSim(SimBackend):
    """Point-mass simulator driven through the standard SimConnect event names."""

    name = "mock"

    def __init__(
        self,
        start: LatLon,
        heading_deg: float = 0.0,
        field_elevation_ft: float = 0.0,
        model: Optional[MockAircraftModel] = None,
        terrain: Optional[Callable[[LatLon], float]] = None,
        wind_from_deg: float = 0.0,
        wind_kt: float = 0.0,
        start_airborne_at_ft: Optional[float] = None,
    ) -> None:
        self.model = model or MockAircraftModel()
        self._terrain = terrain or (lambda _pos: field_elevation_ft)
        self.state = SimState(
            lat=start.lat,
            lon=start.lon,
            altitude_ft=field_elevation_ft,
            ground_elevation_ft=field_elevation_ft,
            heading_true_deg=normalize_deg(heading_deg),
            heading_mag_deg=normalize_deg(heading_deg),
            track_true_deg=normalize_deg(heading_deg),
            on_ground=True,
            wind_from_deg=wind_from_deg,
            wind_kt=wind_kt,
            total_weight_lb=500000.0,
            connected=True,
            engines_running=True,
            parking_brake=False,
        )
        # Autopilot targets the model chases.
        self.target_heading = normalize_deg(heading_deg)
        self.target_altitude = field_elevation_ft
        self.target_speed_kt = 0.0
        self.target_mach: Optional[float] = None
        self.target_vs_fpm: Optional[float] = None
        self.throttle_pct = 0.0
        self.autothrottle = False
        self.speed_is_mach = False
        self._flaps_target = 0
        self._flaps_detents = [0.0, 12.5, 25.0, 50.0, 75.0, 100.0]
        self._gear_target_pct = 100.0
        self._spoilers_armed = False
        self._events: list[tuple[str, int]] = []
        self._lvars: dict[str, float] = {}
        self._vars: dict[str, float] = {}
        self.touchdown_vs_fpm: Optional[float] = None
        self.landed = False

        if start_airborne_at_ft is not None:
            self._start_airborne(start_airborne_at_ft)

    # -- Backend protocol ----------------------------------------------------
    def connect(self) -> None:
        self.state.connected = True

    def close(self) -> None:
        self.state.connected = False

    def capabilities(self) -> SimCapabilities:
        return SimCapabilities(simvars=True, events=True, lvars=True, calculator_code=True)

    def get_lvar(self, name: str) -> Optional[float]:
        return self._lvars.get(name)

    def set_lvar(self, name: str, value: float) -> bool:
        self._lvars[name] = float(value)
        return True

    def exec_calculator_code(self, code: str) -> bool:
        # Understands only the one pattern the adapters emit: "<value> (>L:NAME)".
        parts = code.strip().split()
        if len(parts) == 2 and parts[1].startswith("(>L:") and parts[1].endswith(")"):
            try:
                self._lvars[parts[1][4:-1]] = float(parts[0])
            except ValueError:
                return False
        return True

    def list_lvars(self) -> list[str]:
        return sorted(self._lvars)

    def set_var(self, name: str, value: float, unit: str = "number") -> None:
        self._vars[name.upper()] = value
        upper = name.upper()
        if upper == "NAV ACTIVE FREQUENCY:1":
            self.state.nav1_freq_mhz = value
        elif upper == "NAV OBS:1":
            self.state.nav1_obs_deg = value

    @property
    def events_sent(self) -> list[tuple[str, int]]:
        return list(self._events)

    def send_event(self, event: str, value: int = 0) -> None:
        self._events.append((event, value))
        st = self.state
        e = event.upper()
        if e == "AP_MASTER":
            st.ap_master = not st.ap_master
        elif e == "AUTOPILOT_ON":
            st.ap_master = True
        elif e in ("AUTOPILOT_OFF", "AP_OFF"):
            st.ap_master = False
        elif e == "HEADING_BUG_SET":
            self.target_heading = normalize_deg(value)
            st.ap_heading_bug_deg = self.target_heading
        elif e in ("AP_HDG_HOLD_ON", "AP_PANEL_HEADING_HOLD"):
            st.ap_heading_lock = True
            st.ap_nav_lock = False
        elif e == "AP_HDG_HOLD_OFF":
            st.ap_heading_lock = False
        elif e in ("AP_ALT_VAR_SET_ENGLISH",):
            self.target_altitude = float(value)
            st.ap_altitude_target_ft = float(value)
        elif e in ("AP_ALT_HOLD_ON", "AP_PANEL_ALTITUDE_HOLD"):
            st.ap_altitude_lock = True
            self.target_vs_fpm = None
        elif e == "AP_ALT_HOLD_OFF":
            st.ap_altitude_lock = False
        elif e == "AP_VS_VAR_SET_ENGLISH":
            self.target_vs_fpm = float(value)
            st.ap_vs_target_fpm = float(value)
        elif e == "AP_PANEL_VS_HOLD" or e == "AP_VS_HOLD_ON":
            st.ap_altitude_lock = True
        elif e == "AP_SPD_VAR_SET":
            self.target_speed_kt = float(value)
            self.speed_is_mach = False
            self.target_mach = None
            st.ap_airspeed_target_kt = float(value)
        elif e == "AP_MACH_VAR_SET":
            self.target_mach = float(value) / 100.0
            self.speed_is_mach = True
        elif e in ("AUTO_THROTTLE_ARM", "AUTO_THROTTLE_TO_GA"):
            self.autothrottle = True
            self.throttle_pct = 100.0
        elif e == "AP_APR_HOLD_ON" or e == "AP_APR_HOLD":
            st.ap_approach_hold = True
        elif e == "AP_NAV1_HOLD_ON":
            st.ap_nav_lock = True
            st.ap_heading_lock = False
        elif e == "GEAR_UP":
            self._gear_target_pct = 0.0
        elif e == "GEAR_DOWN":
            self._gear_target_pct = 100.0
        elif e == "FLAPS_SET":
            # FLAPS_SET takes 0..16383 across the flap range.
            frac = clamp(value / 16383.0, 0.0, 1.0)
            self._flaps_target = int(round(frac * (len(self._flaps_detents) - 1)))
        elif e == "FLAPS_INCR":
            self._flaps_target = min(self._flaps_target + 1, len(self._flaps_detents) - 1)
        elif e == "FLAPS_DECR":
            self._flaps_target = max(self._flaps_target - 1, 0)
        elif e == "SPOILERS_ARM_ON":
            self._spoilers_armed = True
        elif e == "SPOILERS_ON":
            st.spoilers_pct = 100.0
        elif e == "SPOILERS_OFF":
            st.spoilers_pct = 0.0
        elif e == "THROTTLE_SET":
            self.throttle_pct = clamp(value / 16383.0 * 100.0, 0.0, 100.0)
        elif e == "PARKING_BRAKES":
            st.parking_brake = not st.parking_brake
        elif e == "THROTTLE_CUT":
            self.throttle_pct = 0.0

    # -- Integration ---------------------------------------------------------
    def poll(self, dt: float) -> SimState:
        if dt > 0:
            self.step(dt)
        return self.state

    def _start_airborne(self, altitude_ft: float) -> None:
        st = self.state
        st.on_ground = False
        st.altitude_ft = altitude_ft
        st.ias_kt = 280.0
        st.tas_kt = cas_to_tas(280.0, altitude_ft)
        st.ground_speed_kt = st.tas_kt
        self._gear_target_pct = 0.0
        st.gear_down_pct = 0.0
        self.target_altitude = altitude_ft
        self.autothrottle = True
        self.throttle_pct = 85.0

    def step(self, dt: float) -> SimState:
        st = self.state
        m = self.model
        st.sim_time_s += dt
        st.ground_elevation_ft = self._terrain(st.position)

        self._step_config(dt)
        self._step_speed(dt)
        self._step_vertical(dt)
        self._step_lateral(dt)
        self._step_position(dt)

        st.mach = tas_to_mach(st.tas_kt, max(st.altitude_ft, 0.0))
        st.altitude_agl_ft = max(0.0, st.altitude_ft - st.ground_elevation_ft)
        st.heading_mag_deg = normalize_deg(st.heading_true_deg - st.magvar_deg)
        return st

    def _step_config(self, dt: float) -> None:
        st = self.state
        m = self.model
        gear_step = 100.0 / m.gear_transit_s * dt
        st.gear_down_pct = approach_value(st.gear_down_pct, self._gear_target_pct, gear_step)
        target_pct = self._flaps_detents[self._flaps_target]
        flap_step = 100.0 / m.flap_transit_s * dt
        st.flaps_pct = approach_value(st.flaps_pct, target_pct, flap_step)
        # Report the detent actually reached, so guidance sees real configuration.
        reached = min(
            range(len(self._flaps_detents)),
            key=lambda i: abs(self._flaps_detents[i] - st.flaps_pct),
        )
        st.flaps_index = reached
        if self._spoilers_armed and st.on_ground and st.ground_speed_kt > 40:
            st.spoilers_pct = 100.0

    def _target_ias(self) -> float:
        """The commanded speed expressed as CAS, whatever units it was set in."""
        st = self.state
        if self.speed_is_mach and self.target_mach:
            alt = max(st.altitude_ft, 0.0)
            return tas_to_cas(mach_to_tas(self.target_mach, alt), alt)
        return self.target_speed_kt

    def _step_speed(self, dt: float) -> None:
        st = self.state
        m = self.model
        if st.on_ground and not self.landed:
            if self.throttle_pct > 50.0:
                st.ias_kt += m.ground_accel_kt_s * dt
            else:
                st.ias_kt = max(0.0, st.ias_kt - m.braking_kt_s * dt)
        elif st.on_ground and self.landed:
            brake = m.braking_kt_s * (1.6 if st.spoilers_pct > 50 else 1.0)
            st.ias_kt = max(0.0, st.ias_kt - brake * dt)
        else:
            target = self._target_ias()
            if target > 0:
                # Climbing eats into available thrust, so acceleration degrades.
                climbing = max(0.0, st.vertical_speed_fpm) / 2000.0
                accel = m.accel_kt_s * max(0.25, 1.0 - 0.6 * climbing)
                decel = m.decel_kt_s * (1.0 + st.spoilers_pct / 100.0 + st.flaps_pct / 120.0)
                step = accel * dt if target > st.ias_kt else decel * dt
                st.ias_kt = approach_value(st.ias_kt, target, step)
        st.tas_kt = cas_to_tas(st.ias_kt, max(st.altitude_ft, 0.0))

    def _max_climb_fpm(self) -> float:
        st = self.state
        m = self.model
        margin = clamp(1.0 - st.altitude_ft / m.service_ceiling_ft, 0.05, 1.0)
        return m.max_climb_fpm * margin

    def _step_vertical(self, dt: float) -> None:
        st = self.state
        m = self.model
        # Height is recomputed here rather than read from the field set at the
        # end of the previous step. At 800 fpm and a two-second step the
        # aeroplane falls fifty feet per cycle, so a stale height can skip the
        # entire flare window and put it on the runway at the approach rate.
        agl = max(0.0, st.altitude_ft - st.ground_elevation_ft)
        if st.on_ground:
            if st.ias_kt >= m.rotate_speed_kt and not self.landed:
                st.on_ground = False
                st.vertical_speed_fpm = 1800.0
                st.pitch_deg = 12.0
            else:
                st.vertical_speed_fpm = 0.0
                st.altitude_ft = st.ground_elevation_ft
            return

        if self.target_vs_fpm is not None:
            commanded = self.target_vs_fpm
            # Do not fly through the selected altitude even in vertical-speed mode.
            if (commanded > 0 and st.altitude_ft >= self.target_altitude) or (
                commanded < 0 and st.altitude_ft <= self.target_altitude
            ):
                commanded = 0.0
        else:
            error = self.target_altitude - st.altitude_ft
            # Proportional capture, gentle in the last 1000 ft.
            commanded = clamp(error * 4.0, -m.max_descent_fpm, self._max_climb_fpm())
        commanded = clamp(commanded, -m.max_descent_fpm, self._max_climb_fpm())
        st.vertical_speed_fpm = approach_value(st.vertical_speed_fpm, commanded, 900.0 * dt)
        st.altitude_ft += st.vertical_speed_fpm / 60.0 * dt
        st.pitch_deg = math.degrees(math.atan2(st.vertical_speed_fpm / 60.0,
                                               max(1.0, st.tas_kt * 1.68781)))

        agl = max(0.0, st.altitude_ft - st.ground_elevation_ft)
        if st.ap_approach_hold and agl <= AUTOLAND_FLARE_AGL_FT \
                and st.vertical_speed_fpm < AUTOLAND_FLARE_VS_FPM:
            # An aeroplane certified for autoland flares itself. Modelling that
            # here is what makes an autoland in the mock land rather than
            # arrive: the AI Pilot deliberately keeps its hands off the
            # vertical channel once the glideslope is captured, so if the
            # aeroplane does not round out, nothing does.
            st.vertical_speed_fpm = AUTOLAND_FLARE_VS_FPM

        if st.altitude_ft <= st.ground_elevation_ft + m.touchdown_agl_ft and st.vertical_speed_fpm < 0:
            self.touchdown_vs_fpm = st.vertical_speed_fpm
            st.altitude_ft = st.ground_elevation_ft
            st.vertical_speed_fpm = 0.0
            st.on_ground = True
            self.landed = True
            self.throttle_pct = 0.0
            self.autothrottle = False

    def _step_lateral(self, dt: float) -> None:
        st = self.state
        m = self.model
        if st.on_ground and st.ias_kt < 40:
            st.heading_true_deg = normalize_deg(self.target_heading)
            st.bank_deg = 0.0
            return
        error = signed_diff_deg(self.target_heading, st.heading_true_deg)
        commanded_bank = clamp(error * 1.5, -m.max_bank_deg, m.max_bank_deg)
        if st.on_ground:
            commanded_bank = 0.0
        st.bank_deg = approach_value(st.bank_deg, commanded_bank, m.bank_rate_deg_s * dt)
        if st.tas_kt > 1.0:
            # Rate of turn for a coordinated turn: 1091 * tan(bank) / TAS deg/s.
            rate = 1091.0 * math.tan(math.radians(st.bank_deg)) / max(st.tas_kt, 1.0)
            st.heading_true_deg = normalize_deg(st.heading_true_deg + rate * dt)

    def _surface_wind_factor(self) -> float:
        """Fraction of the free-stream wind felt at the present height.

        Surface friction kills the wind in the lowest couple of thousand feet;
        without modelling that, a jet-stream-strength wind applied all the way
        to the runway leaves the aeroplane hovering over the threshold at
        forty knots of ground speed, which is not a guidance problem but does
        make every test of one meaningless.
        """
        agl = max(0.0, self.state.altitude_agl_ft)
        if agl >= BOUNDARY_LAYER_FT:
            return 1.0
        return SURFACE_WIND_FRACTION + (1.0 - SURFACE_WIND_FRACTION) * (agl / BOUNDARY_LAYER_FT)

    def _step_position(self, dt: float) -> None:
        st = self.state
        # Wind triangle: air vector plus wind vector gives the ground vector.
        hdg = math.radians(st.heading_true_deg)
        air_n = st.tas_kt * math.cos(hdg)
        air_e = st.tas_kt * math.sin(hdg)
        wind_kt = st.wind_kt * self._surface_wind_factor()
        wind_to = math.radians(st.wind_from_deg + 180.0)
        wind_n = wind_kt * math.cos(wind_to)
        wind_e = wind_kt * math.sin(wind_to)
        gnd_n, gnd_e = air_n + wind_n, air_e + wind_e
        st.ground_speed_kt = math.hypot(gnd_n, gnd_e)
        if st.ground_speed_kt > 0.1:
            st.track_true_deg = normalize_deg(math.degrees(math.atan2(gnd_e, gnd_n)))
        distance_nm = st.ground_speed_kt * (dt / 3600.0)
        if distance_nm > 0:
            new_pos = destination_point(st.position, st.track_true_deg, distance_nm)
            st.lat, st.lon = new_pos.lat, new_pos.lon
