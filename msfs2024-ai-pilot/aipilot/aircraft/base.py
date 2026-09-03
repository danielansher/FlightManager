"""Aircraft adapters: the translation from intent to simulator commands.

The guidance says *fly heading 271, climb to 37,000, slow to 250*. What that
means in button presses differs between a Boeing and an Airbus, and between one
add-on Airbus and another. Adapters absorb that difference so the autopilot
logic never has to care which aeroplane it is in.

The base adapter uses only standard SimConnect key events. That is not a
lowest-common-denominator fallback -- it is the *right* implementation for a
large part of the fleet, the default 787 included, because Asobo wire the stock
events straight into the aircraft's autoflight system. Where an add-on needs
more (Airbus FCU push/pull semantics, say) a subclass adds it.

Two design choices are worth calling out:

*Closed-loop configuration changes.* Flaps are moved one detent at a time
against the reported handle index rather than commanded absolutely, because
``FLAPS_SET`` scaling differs between aircraft with different numbers of
detents. Asking for "one more" and checking is correct everywhere.

*Magnetic versus true.* All guidance is computed in true degrees; the heading
bug is magnetic. The conversion happens here, once, using the magnetic
variation the simulator reports for the aeroplane's present position.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional

from ..geo import normalize_deg
from ..perf.profiles import AircraftProfile
from ..sim.base import SimBackend, SimState

Logger = Callable[[str], None]

#: A simulator axis runs -16383 to +16383, and the wheel brake axes are no
#: exception: the centre of the axis is half braking. Naming the ends stops
#: "no brakes" being written as the zero it looks like it should be.
BRAKE_AXIS_OFF = -16383
BRAKE_AXIS_FULL = 16383


@dataclass
class AdapterCapabilities:
    """What this adapter can actually do in the aeroplane it is driving."""

    autopilot: bool = True
    autothrottle: bool = True
    vertical_speed: bool = True
    approach_mode: bool = True
    autoland: bool = True
    autobrake: bool = False
    managed_modes: bool = False       # Airbus-style push/pull
    needs_lvars: bool = False
    lvars_available: bool = True

    @property
    def degraded(self) -> bool:
        return self.needs_lvars and not self.lvars_available


class AircraftAdapter:
    """Standard-event adapter. Correct for the default 787 and most others."""

    key = "generic"

    #: How much the heading bug must be out before it is worth resending. Every
    #: event is a network round trip and the sim's own bug has finite
    #: resolution, so re-sending an unchanged value every frame is pure noise.
    heading_deadband_deg = 0.5
    altitude_deadband_ft = 40.0
    speed_deadband_kt = 1.0
    vs_deadband_fpm = 90.0

    def __init__(self, sim: SimBackend, profile: AircraftProfile,
                 log: Optional[Logger] = None) -> None:
        self.sim = sim
        self.profile = profile
        self.log = log or (lambda _message: None)
        self._last_heading: Optional[float] = None
        self._last_altitude: Optional[float] = None
        self._last_speed: Optional[float] = None
        self._last_mach: Optional[float] = None
        self._last_vs: Optional[float] = None
        self._flap_command_index: Optional[int] = None
        self._gear_down: Optional[bool] = None
        self._spoilers_armed = False
        self._approach_armed = False
        self._autothrottle_on = False
        self._toga = False
        self._throttle: Optional[float] = None
        self._speedbrake: Optional[float] = None
        self._vs_mode = False
        self._level_change_pending = False
        self._announced_engage = False
        self._switches: dict[str, bool] = {}
        self._switch_sent: dict[str, int] = {}
        self._switch_clock = 0
        self._steering: Optional[float] = None
        self._brakes: Optional[float] = None
        self._tug_heading: Optional[float] = None

    # -- Introspection -------------------------------------------------------
    def capabilities(self) -> AdapterCapabilities:
        return AdapterCapabilities(autoland=self.profile.autoland_capable)

    def describe(self) -> str:
        return f"{self.profile.name} via standard SimConnect events"

    def prepare(self) -> None:
        """Register anything the adapter needs before the first command."""

    # -- Autoflight ----------------------------------------------------------
    def engage_autopilot(self, state: SimState) -> None:
        """Engage the autopilot if it is not already in.

        Announced only the first time. This is called every control cycle, and
        an aeroplane that keeps dropping the autopilot would otherwise fill the
        log with "Autopilot engaged" instead of saying the useful thing, which
        is that it keeps being lost. The controller's watchdog says that.
        """
        if not state.ap_master:
            self.sim.send_event("AP_MASTER")
            if not self._announced_engage:
                self._announced_engage = True
                self.log("Autopilot engaged")

    def disengage_autopilot(self, state: SimState) -> None:
        if state.ap_master:
            self.sim.send_event("AP_MASTER")
            self.log("Autopilot disengaged")

    def set_heading_true(self, true_deg: float, state: SimState) -> None:
        """Command a true heading; converts to the magnetic bug the sim wants."""
        magnetic = normalize_deg(true_deg - state.magvar_deg)
        self.set_heading_magnetic(magnetic)

    def set_heading_magnetic(self, magnetic_deg: float) -> None:
        target = normalize_deg(magnetic_deg)
        if self._last_heading is not None and \
                abs(_angle_gap(target, self._last_heading)) < self.heading_deadband_deg:
            return
        self._last_heading = target
        self.sim.send_event("HEADING_BUG_SET", int(round(target)) % 360)

    def select_heading_mode(self, state: SimState) -> None:
        if not state.ap_heading_lock:
            self.sim.send_event("AP_HDG_HOLD_ON")

    def set_altitude(self, altitude_ft: float) -> None:
        target = round(altitude_ft / 100.0) * 100.0
        if self._last_altitude is not None and \
                abs(target - self._last_altitude) < self.altitude_deadband_ft:
            return
        self._last_altitude = target
        self.sim.send_event("AP_ALT_VAR_SET_ENGLISH", int(round(target)))

    def select_altitude_mode(self, state: SimState) -> None:
        """Put the vertical channel back under altitude capture.

        The condition has to include our own vertical-speed flag, not just the
        aeroplane's altitude-lock flag. Selecting a vertical speed *also* sets
        altitude lock -- both are "the autopilot is controlling the vertical
        channel" -- so testing the flag alone means that once a vertical speed
        has been commanded, altitude capture can never be selected again. The
        aeroplane then holds the last rate it was given, and a go-around that
        commands a climb to three thousand feet sits at five hundred instead.
        """
        if not state.ap_altitude_lock or self._vs_mode:
            self.sim.send_event("AP_ALT_HOLD_ON")
            self._vs_mode = False
            self._last_vs = None

    def set_vertical_speed(self, fpm: float) -> None:
        target = round(fpm / 50.0) * 50.0
        if self._vs_mode and self._last_vs is not None and \
                abs(target - self._last_vs) < self.vs_deadband_fpm:
            return
        self._last_vs = target
        self._vs_mode = True
        self.sim.send_event("AP_VS_VAR_SET_ENGLISH", int(round(target)))
        self.sim.send_event("AP_PANEL_VS_HOLD", 1)

    def clear_vertical_speed(self) -> None:
        """Leave vertical-speed mode, so the next rate command is not filtered.

        Only clears the cache. Leaving the mode itself is
        :meth:`select_altitude_mode`, because that is a command to the
        aeroplane and this is not.
        """
        self._last_vs = None

    def set_speed_kt(self, kt: float) -> None:
        target = round(kt)
        if self._last_speed is not None and abs(target - self._last_speed) < self.speed_deadband_kt:
            return
        self._last_speed = target
        self._last_mach = None
        self.sim.send_event("AP_SPD_VAR_SET", int(target))

    def set_mach(self, mach: float) -> None:
        target = round(mach, 3)
        if self._last_mach is not None and abs(target - self._last_mach) < 0.004:
            return
        self._last_mach = target
        self._last_speed = None
        # The event takes Mach scaled by 100, so 0.85 goes across as 85.
        self.sim.send_event("AP_MACH_VAR_SET", int(round(target * 100)))

    def set_autothrottle(self, on: bool) -> None:
        if on and not self._autothrottle_on:
            self.sim.send_event("AUTO_THROTTLE_ARM")
            self._autothrottle_on = True
            self.log("Autothrottle armed")
        elif not on and self._autothrottle_on:
            self.sim.send_event("AUTO_THROTTLE_ARM")
            self._autothrottle_on = False

    def takeoff_thrust(self) -> None:
        """Set takeoff or go-around thrust. Safe to call every cycle.

        TOGA is pressed once per takeoff or go-around, not once per cycle.
        It used to be sent unconditionally, which put nearly three hundred
        presses into a single takeoff roll: harmless on most aeroplanes,
        since the second press does nothing, but not on the ones where
        pressing it again cycles the mode -- and it buries a trace in noise
        that makes a real fault harder to see. ``clear_takeoff_thrust``
        resets the flag so a go-around presses it again.
        """
        if self._throttle != 100.0:
            self._throttle = 100.0
            self.sim.send_event("THROTTLE_SET", 16383)
        if not self._toga:
            self._toga = True
            self.sim.send_event("AUTO_THROTTLE_TO_GA")
            self.log("Takeoff thrust set")

    def clear_takeoff_thrust(self) -> None:
        """Forget that takeoff thrust was set, so a go-around announces itself."""
        self._toga = False

    def idle_thrust(self) -> None:
        self._toga = False
        self.set_throttle_percent(0.0)

    def set_throttle_percent(self, percent: float) -> None:
        """Move the thrust levers directly, as a percentage.

        Needed because an armed autothrottle is not the same as an autothrottle
        that is flying the aeroplane. When it does not take the levers, whatever
        the last commanded position was stays -- and after takeoff that is full
        power, which turns the descent into an overspeed.
        """
        target = max(0.0, min(100.0, percent))
        if self._throttle is not None and abs(target - self._throttle) < 1.5:
            return
        self._throttle = target
        self._toga = False
        self.sim.send_event("THROTTLE_SET", int(round(target / 100.0 * 16383)))

    # -- Approach ------------------------------------------------------------
    def tune_nav1(self, frequency_mhz: float, course_true_deg: Optional[float],
                  state: SimState) -> None:
        hz = int(round(frequency_mhz * 1_000_000))
        self.sim.send_event("NAV1_RADIO_SET_HZ", hz)
        if course_true_deg is not None:
            magnetic = normalize_deg(course_true_deg - state.magvar_deg)
            self.sim.send_event("VOR1_SET", int(round(magnetic)) % 360)
        self.log(f"NAV1 tuned {frequency_mhz:.2f}")

    def arm_approach(self, state: SimState) -> None:
        if self._approach_armed or state.ap_approach_hold:
            return
        self.sim.send_event("AP_APR_HOLD_ON")
        self._approach_armed = True
        self.log("Approach mode armed")

    def approach_is_captured(self, state: SimState) -> bool:
        """Whether the aeroplane's own ILS guidance has taken over."""
        return bool(state.ap_approach_hold and state.ap_glideslope_hold)

    # -- Configuration -------------------------------------------------------
    def set_flaps(self, index: int, state: SimState) -> bool:
        """Move the flap handle one detent towards ``index``.

        Returns ``True`` once the handle is where it was asked to be. Called
        every cycle by the configuration logic, so it walks the handle out one
        notch at a time as speed allows.
        """
        wanted = max(0, min(index, self._max_flap_index()))
        self._flap_command_index = wanted
        current = state.flaps_index
        if current == wanted:
            return True
        self.sim.send_event("FLAPS_INCR" if current < wanted else "FLAPS_DECR")
        return False

    def _max_flap_index(self) -> int:
        return max((f.index for f in self.profile.flaps), default=0)

    def set_gear(self, down: bool, state: SimState) -> None:
        if self._gear_down is down:
            return
        self._gear_down = down
        self.sim.send_event("GEAR_DOWN" if down else "GEAR_UP")
        self.log("Gear down" if down else "Gear up")

    def arm_spoilers(self) -> None:
        if self._spoilers_armed:
            return
        self.sim.send_event("SPOILERS_ARM_ON")
        self._spoilers_armed = True
        self.log("Speedbrake armed")

    def deploy_spoilers(self) -> None:
        self.sim.send_event("SPOILERS_ON")

    def set_speedbrake_percent(self, percent: float) -> None:
        """Partial speedbrake, for shedding speed without fully deploying."""
        target = max(0.0, min(100.0, percent))
        if self._speedbrake is not None and abs(target - self._speedbrake) < 8.0:
            return
        self._speedbrake = target
        self.sim.send_event("SPOILERS_SET", int(round(target / 100.0 * 16383)))

    def retract_spoilers(self) -> None:
        self.sim.send_event("SPOILERS_OFF")

    def set_autobrake(self, level: int) -> None:
        """Best effort: the stock events only step the selector up and down."""
        for _ in range(max(0, level)):
            self.sim.send_event("INCREASE_AUTOBRAKE_CONTROL")

    def set_parking_brake(self, on: bool, state: SimState) -> None:
        if state.parking_brake != on:
            self.sim.send_event("PARKING_BRAKES")

    def release_brakes_hard(self) -> None:
        """Let everything go, without asking the simulator what it thinks.

        The state-guarded setters are right almost always: they stop the AI
        Pilot fighting a switch the pilot moved, and stop a toggle event being
        sent when the switch is already where it should be. But they trust
        what the aeroplane reports, and a study-level add-on running its own
        hydraulics may not report BRAKE PARKING POSITION at all -- in which
        case "already off" is a guess, the toggle is never sent, and the
        aeroplane sits at the gate with the thrust up and the brake on, which
        is exactly what was reported from a 787 at Kennedy.

        So this is the escape hatch: send everything, unconditionally, and
        forget the cached values so the next ordinary call is not suppressed
        as a no-op.

        Unconditionally, however, only works with events that say what they
        mean. PARKING_BRAKES is a TOGGLE: fired at an aeroplane whose brake is
        already off it turns the brake ON, which is the opposite of the job.
        Flown at Kennedy this fired every eight seconds and left the parking
        brake on for half of every cycle. PARKING_BRAKE_SET takes the state
        wanted and is idempotent -- confirmed on a Horizon 787-9, where 0 twice
        stays released and 1 twice stays set -- so it needs no guess about what
        the aeroplane currently reports, which is the whole point here.
        """
        self.sim.send_event("PARKING_BRAKE_SET", 0)
        self.sim.send_event("AXIS_LEFT_BRAKE_SET", BRAKE_AXIS_OFF)
        self.sim.send_event("AXIS_RIGHT_BRAKE_SET", BRAKE_AXIS_OFF)
        self._brakes = None

    def apply_brakes(self) -> None:
        self.sim.send_event("BRAKES")

    # -- Ground handling -----------------------------------------------------
    def set_steering(self, value: float) -> None:
        """Nosewheel steering, -1 full left to +1 full right.

        Sent as rudder, which is what steers an airliner's nosewheel below
        taxi speed in the simulator.
        """
        target = max(-1.0, min(1.0, value))
        if self._steering is not None and abs(target - self._steering) < 0.02:
            return
        self._steering = target
        self.sim.send_event("RUDDER_SET", int(round(target * 16383)))

    def set_wheel_brakes(self, amount: float) -> None:
        """Proportional wheel braking, 0 to 1, applied evenly.

        The brake axis is a full axis, -16383 to +16383, so its centre is half
        braking. Scaling 0..1 onto 0..16383 therefore never released anything:
        asking for no brakes sent zero, which is half on. A 787 pushed back at
        Kennedy would not roll afterwards at 65% N1, and the trace showed the
        release being sent exactly as intended.
        """
        target = max(0.0, min(1.0, amount))
        if self._brakes is not None and abs(target - self._brakes) < 0.05:
            return
        self._brakes = target
        raw = int(round(BRAKE_AXIS_OFF + target * (BRAKE_AXIS_FULL - BRAKE_AXIS_OFF)))
        self.sim.send_event("AXIS_LEFT_BRAKE_SET", raw)
        self.sim.send_event("AXIS_RIGHT_BRAKE_SET", raw)

    def set_pushback(self, on: bool, state: SimState) -> None:
        """Attach or release the tug. The event is a toggle, so check first."""
        if bool(state.pushback_attached) is on:
            return
        self.sim.send_event("TOGGLE_PUSHBACK")
        if not on:
            # The tug is gone; the next push starts from no known heading.
            self._tug_heading = None
        self.log("Pushback started" if on else "Pushback complete")

    def set_tug_heading(self, true_deg: float) -> None:
        """Which way the tug should push, as a true heading.

        The event takes an angle scaled across the full range of a 32-bit
        unsigned integer rather than in degrees.

        Sent once per heading, not once per cycle. A real pushback showed
        this event going out three hundred and thirty times in eighty-four
        seconds -- four times a second, for a value that never changed. The
        heading is a decision, and re-taking it every quarter of a second is
        at best noise on the connection and at worst an instruction to the
        tug to start again.
        """
        target = normalize_deg(true_deg)
        if self._tug_heading is not None and \
                abs(_angle_gap(target, self._tug_heading)) < 1.0:
            return
        self._tug_heading = target
        raw = int(target / 360.0 * 4294967296.0) & 0xFFFFFFFF
        self.sim.send_event("KEY_TUG_HEADING", raw)

    # -- Lights and cabin signs ----------------------------------------------
    # Only some of these have explicit on and off events; the rest are toggles.
    # A toggle sent without knowing the current state does the right thing half
    # the time, which for a landing light means arriving at night with it off.
    # So each one reads the state the simulator reports and acts only when the
    # switch is actually in the wrong position -- which also means the AI Pilot
    # never fights a setting the pilot changed by hand.
    def _set_switch(self, name: str, on: bool, state: SimState,
                    on_event: Optional[str] = None,
                    off_event: Optional[str] = None,
                    toggle_event: Optional[str] = None) -> None:
        current = bool(getattr(state, name, False))
        if current is on:
            self._switches[name] = on
            return
        # Do not re-send while the simulator has yet to report the change; a
        # toggle repeated every cycle simply flickers the switch.
        if self._switches.get(name) is on and \
                self._switch_sent.get(name, 0) > self._switch_clock - 8:
            return
        self._switches[name] = on
        self._switch_sent[name] = self._switch_clock
        if on and on_event:
            self.sim.send_event(on_event)
        elif not on and off_event:
            self.sim.send_event(off_event)
        elif toggle_event:
            self.sim.send_event(toggle_event)

    def tick_switches(self) -> None:
        """Advance the switch clock. Called once per control cycle."""
        self._switch_clock += 1

    def set_landing_lights(self, on: bool, state: SimState) -> None:
        self._set_switch("light_landing", on, state,
                         "LANDING_LIGHTS_ON", "LANDING_LIGHTS_OFF",
                         "LANDING_LIGHTS_TOGGLE")

    def set_strobes(self, on: bool, state: SimState) -> None:
        self._set_switch("light_strobe", on, state,
                         "STROBES_ON", "STROBES_OFF", "STROBES_TOGGLE")

    def set_taxi_lights(self, on: bool, state: SimState) -> None:
        self._set_switch("light_taxi", on, state,
                         toggle_event="TOGGLE_TAXI_LIGHTS")

    def set_beacon(self, on: bool, state: SimState) -> None:
        self._set_switch("light_beacon", on, state,
                         toggle_event="TOGGLE_BEACON_LIGHTS")

    def set_nav_lights(self, on: bool, state: SimState) -> None:
        self._set_switch("light_nav", on, state,
                         toggle_event="TOGGLE_NAV_LIGHTS")

    def set_wing_lights(self, on: bool, state: SimState) -> None:
        self._set_switch("light_wing", on, state,
                         toggle_event="TOGGLE_WING_LIGHTS")

    def set_logo_lights(self, on: bool, state: SimState) -> None:
        self._set_switch("light_logo", on, state,
                         toggle_event="TOGGLE_LOGO_LIGHTS")

    def set_seatbelt_sign(self, on: bool, state: SimState) -> None:
        self._set_switch("seatbelt_sign", on, state,
                         toggle_event="CABIN_SEATBELTS_ALERT_SWITCH_TOGGLE")

    def set_no_smoking_sign(self, on: bool, state: SimState) -> None:
        self._set_switch("no_smoking_sign", on, state,
                         toggle_event="CABIN_NO_SMOKING_ALERT_SWITCH_TOGGLE")


def _angle_gap(a: float, b: float) -> float:
    return (a - b + 180.0) % 360.0 - 180.0
