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

#: Which simulator event moves which light or sign, and how.
LIGHT_EVENTS = {
    "LANDING_LIGHTS_ON": ("light_landing", "on"),
    "LANDING_LIGHTS_OFF": ("light_landing", "off"),
    "LANDING_LIGHTS_TOGGLE": ("light_landing", "toggle"),
    "STROBES_ON": ("light_strobe", "on"),
    "STROBES_OFF": ("light_strobe", "off"),
    "STROBES_TOGGLE": ("light_strobe", "toggle"),
    "TOGGLE_TAXI_LIGHTS": ("light_taxi", "toggle"),
    "TOGGLE_BEACON_LIGHTS": ("light_beacon", "toggle"),
    "TOGGLE_NAV_LIGHTS": ("light_nav", "toggle"),
    "TOGGLE_WING_LIGHTS": ("light_wing", "toggle"),
    "TOGGLE_LOGO_LIGHTS": ("light_logo", "toggle"),
    "CABIN_SEATBELTS_ALERT_SWITCH_TOGGLE": ("seatbelt_sign", "toggle"),
    "CABIN_NO_SMOKING_ALERT_SWITCH_TOGGLE": ("no_smoking_sign", "toggle"),
}

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
    #: Degrees per second of heading change at full nosewheel deflection, at
    #: normal taxi speed.
    nosewheel_rate_deg_s: float = 6.0
    pushback_speed_kt: float = 3.0
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
        autothrottle_effective: bool = True,
    ) -> None:
        #: Whether arming the autothrottle actually makes it fly the speed.
        #: Set False to reproduce the aeroplane that reports an armed
        #: autothrottle and then leaves the thrust levers exactly where they
        #: were, which is what turns a descent into a four-hundred-knot dive.
        self.autothrottle_effective = autothrottle_effective
        self.model = model or MockAircraftModel()
        #: The wind aloft. What the aeroplane feels, and what the state
        #: reports, is this reduced through the boundary layer.
        self.free_stream_wind_kt = wind_kt
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
        self.tug_heading_deg = 0.0
        #: A tug asked for but not yet reported attached, applied on the next
        #: poll. None when there is nothing pending.
        self._pushback_pending: Optional[bool] = None
        self.steering = 0.0
        self.wheel_brakes = 0.0
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
        elif e == "AUTO_THROTTLE_ARM":
            # Arming the autothrottle does not move the thrust levers. An
            # earlier version set them to full here, which meant the mock
            # aeroplane accelerated and took off the moment the AI Pilot armed
            # the autothrottle at the gate -- and hid the fact that the real
            # one was applying takeoff thrust on the apron.
            self.autothrottle = True
            st.ap_autothrottle = self.autothrottle_effective
        elif e == "AUTO_THROTTLE_TO_GA":
            self.autothrottle = True
            st.ap_autothrottle = self.autothrottle_effective
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
        elif e == "PARKING_BRAKE_SET":
            st.parking_brake = bool(value)
        elif e == "RUDDER_SET":
            # Negated, because RUDDER_SET positive turns the aeroplane LEFT and
            # self.steering is positive to the right. Reading the axis straight
            # through gave this mock the opposite plant sign from the simulator
            # it stands in for, so every taxi test passed here for the wrong
            # reason while no real taxi ever reached a runway.
            self.steering = clamp(-value / 16383.0, -1.0, 1.0)
        elif e in ("AXIS_LEFT_BRAKE_SET", "AXIS_RIGHT_BRAKE_SET"):
            # A brake axis runs from -16383 (off) to +16383 (hard on), so zero
            # is HALF braking, not none. Reading it as 0..16383 made a released
            # brake of zero look fully off -- kinder than the simulator, and it
            # hid a 787 sitting at a Kennedy gate at 65% N1 with the wheel
            # brakes half on, unable to move and with nothing in the trace to
            # say why.
            self.wheel_brakes = clamp((value + 16383.0) / 32766.0, 0.0, 1.0)
        elif e == "THROTTLE_CUT":
            self.throttle_pct = 0.0
        elif e in LIGHT_EVENTS:
            field, action = LIGHT_EVENTS[e]
            if action == "on":
                setattr(st, field, True)
            elif action == "off":
                setattr(st, field, False)
            else:
                setattr(st, field, not getattr(st, field))
        elif e == "TOGGLE_PUSHBACK":
            # A tug does not appear in the same breath as the request for it.
            # The simulator attaches one and reports it on a later state
            # update, and the gap matters: anything sent to the tug in the
            # meantime is sent to nothing. Attaching instantly here made the
            # mock kinder than the simulator and hid a tug heading that was
            # only ever sent before there was a tug to hear it.
            self._pushback_pending = not st.pushback_attached
        elif e == "KEY_TUG_HEADING":
            # The heading the aeroplane is to end up facing, scaled across the
            # range of a 32-bit unsigned integer.
            self.tug_heading_deg = (value / 4294967296.0) * 360.0
            # In the simulator this event *summons* the tug: sending it is how
            # a pushback is started, not merely how a pushback already running
            # is steered. Modelling that matters, because it means anything
            # still sending a tug heading after asking the tug to leave gets
            # it straight back, and the aeroplane never moves under its own
            # power again. An earlier version of this mock let the heading
            # through without attaching, so that deadlock could not be
            # reproduced here -- only in the real simulator, on a real stand.
            st.pushback_attached = True
            if st.pushback_state == 3:
                st.pushback_state = 0

    # -- Integration ---------------------------------------------------------
    def poll(self, dt: float) -> SimState:
        if self._pushback_pending is not None:
            self.state.pushback_attached = self._pushback_pending
            self.state.pushback_state = 0 if self._pushback_pending else 3
            self._pushback_pending = None
        if dt > 0:
            self.step(dt)
        # The lever position the simulator reports back, as against the one
        # that was commanded. The flight recorder compares the two.
        self.state.throttle_percent = self.throttle_pct
        self.state.engine_n1_pct = min(100.0, 20.0 + self.throttle_pct * 0.8)
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

        self._step_wind()
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

    #: Speed at which the landing rollout is over and the aeroplane is simply
    #: an aeroplane on the ground again, under its own power. Below the speed
    #: the controller hands over to the taxi at, so the two do not fight.
    ROLLOUT_END_KT = 20.0

    def _step_speed(self, dt: float) -> None:
        st = self.state
        m = self.model
        if st.pushback_attached:
            st.ias_kt = m.pushback_speed_kt
            return
        if st.on_ground and not self.landed:
            if self.throttle_pct > 50.0:
                st.ias_kt += m.ground_accel_kt_s * dt
            elif self.throttle_pct > 3.0:
                # Taxi power: a modest acceleration that drag and brakes fight.
                thrust = m.ground_accel_kt_s * (self.throttle_pct / 60.0)
                drag = 0.5 + m.braking_kt_s * self.wheel_brakes
                st.ias_kt = max(0.0, st.ias_kt + (thrust - drag) * dt)
            else:
                st.ias_kt = max(0.0, st.ias_kt
                                - (m.braking_kt_s * max(0.25, self.wheel_brakes)) * dt)
        elif st.on_ground and self.landed:
            brake = m.braking_kt_s * (1.6 if st.spoilers_pct > 50 else 1.0)
            st.ias_kt = max(0.0, st.ias_kt - brake * dt)
            if st.ias_kt <= self.ROLLOUT_END_KT:
                # The landing rollout is over. Until this was here the flag
                # stayed set for good, the automatic braking never stopped and
                # the throttle was ignored, so an aeroplane that had landed
                # could never taxi again -- which meant the taxi to the stand
                # could not be tested at all, only written.
                self.landed = False
        elif self.autothrottle and self.autothrottle_effective:
            target = self._target_ias()
            if target > 0:
                # Climbing eats into available thrust, so acceleration degrades.
                climbing = max(0.0, st.vertical_speed_fpm) / 2000.0
                accel = m.accel_kt_s * max(0.25, 1.0 - 0.6 * climbing)
                decel = m.decel_kt_s * (1.0 + st.spoilers_pct / 100.0 + st.flaps_pct / 120.0)
                step = accel * dt if target > st.ias_kt else decel * dt
                st.ias_kt = approach_value(st.ias_kt, target, step)
        else:
            # No working autothrottle: the speed is whatever the thrust levers
            # and the drag make it, which is the point. Levers left at takeoff
            # power after departure accelerate the aeroplane until something
            # else stops it.
            drag = m.decel_kt_s * (1.0 + st.spoilers_pct / 100.0
                                   + st.flaps_pct / 60.0
                                   + max(0.0, st.vertical_speed_fpm) / 1500.0)
            thrust = m.accel_kt_s * (self.throttle_pct / 60.0)
            st.ias_kt = max(0.0, st.ias_kt + (thrust - drag) * dt)
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
        if st.on_ground:
            st.bank_deg = 0.0
            if st.pushback_attached:
                # The tug turns the aeroplane; it does not teleport it.
                error = signed_diff_deg(self.tug_heading_deg, st.heading_true_deg)
                st.heading_true_deg = normalize_deg(
                    st.heading_true_deg + max(-4.0 * dt, min(4.0 * dt, error))
                )
                return
            # Nosewheel steering, which does nothing at a standstill and less
            # as the aeroplane speeds up -- both of which matter to a taxi
            # controller, and neither of which an earlier version modelled: it
            # simply set the heading to whatever was asked for.
            speed_factor = min(1.0, max(0.0, st.ground_speed_kt) / 12.0)
            if st.ground_speed_kt > 45.0:
                speed_factor *= 45.0 / st.ground_speed_kt
            rate = self.steering * m.nosewheel_rate_deg_s * speed_factor
            st.heading_true_deg = normalize_deg(st.heading_true_deg + rate * dt)
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

    def _step_wind(self) -> None:
        """Report the wind the aeroplane is actually in.

        The simulator's AMBIENT WIND VELOCITY is the wind where the aeroplane
        is, not the free-stream value aloft. This mock used to reduce the wind
        for its own integration and then report the unreduced figure, so the
        guidance computed a drift correction for forty knots while fourteen
        were blowing, crabbed three times too much, and arrived a thousand
        feet upwind of the centreline. The guidance was not at fault and could
        not have been fixed, because the numbers it was given were not the
        ones being flown.
        """
        st = self.state
        st.wind_kt = self.free_stream_wind_kt * self._surface_wind_factor()

    def _step_position(self, dt: float) -> None:
        st = self.state
        if st.pushback_attached and st.on_ground:
            st.ground_speed_kt = self.model.pushback_speed_kt
            st.track_true_deg = normalize_deg(st.heading_true_deg + 180.0)
            moved = st.ground_speed_kt * (dt / 3600.0)
            new_position = destination_point(st.position, st.track_true_deg, moved)
            st.lat, st.lon = new_position.lat, new_position.lon
            return
        if st.on_ground:
            # On the ground the wheels decide where the aeroplane goes, so it
            # travels along its heading at its own speed and the wind does not
            # push it sideways. Running the wind triangle here instead meant a
            # stationary aeroplane with the brakes on drifted across the apron
            # at a few knots for ever, which -- among other things -- meant it
            # could never satisfy "stopped on the stand" and so never finished
            # a flight on a windy day.
            # The simplification here is that on the ground the speed being
            # integrated is treated as speed over the ground rather than
            # through the air, so a takeoff roll into a headwind is a little
            # longer than it should be. That is the right way round to be
            # wrong, and nothing on the ground is measured against it.
            st.ground_speed_kt = max(0.0, st.ias_kt)
            st.track_true_deg = st.heading_true_deg
            moved = st.ground_speed_kt * (dt / 3600.0)
            if moved > 0:
                new_pos = destination_point(st.position, st.track_true_deg, moved)
                st.lat, st.lon = new_pos.lat, new_pos.lon
            return
        # Wind triangle: air vector plus wind vector gives the ground vector.
        hdg = math.radians(st.heading_true_deg)
        air_n = st.tas_kt * math.cos(hdg)
        air_e = st.tas_kt * math.sin(hdg)
        wind_kt = st.wind_kt
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
