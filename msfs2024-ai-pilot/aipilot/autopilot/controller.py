"""The AI Pilot itself: the loop that flies the aeroplane gate to gate.

Everything else in this package computes numbers. This is the part that decides
*when* -- when to rotate, when to raise the gear, when to leave the cruise
level, when to put the flaps out, when to hand back control -- and it is the
part that has to be conservative, because it is the part that can break an
otherwise perfectly good flight.

Three principles run through it:

**Phases only move forward.** A momentary blip in altitude or distance must
never send the aeroplane back to a phase it has left. Transitions are checked
against :data:`~aipilot.autopilot.phases.PHASE_ORDER`.

**Configuration follows speed, not just distance.** Flaps come out when the
aeroplane is slow enough for them, not when it reaches a mileage. Getting that
backwards is how you get a flap overspeed twelve miles out.

**It says when it cannot do something.** Without an ILS and an autoland-capable
aeroplane there is no honest way to complete a landing automatically, so the AI
Pilot flies a stabilised approach and hands over, loudly, rather than pretending
and dropping the aeroplane on the runway.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Callable, Optional

from ..aircraft.base import AircraftAdapter
from ..geo import LatLon, distance_nm
from ..perf.profiles import AircraftProfile
from ..route.plan import FlightPlan
from ..route.profile import build_vertical_profile
from ..sim.base import SimBackend, SimState
from .lateral import LateralGuidance
from .phases import EventLog, FlightEvent, Phase, phase_rank
from .vertical import VerticalGuidance, should_start_descent

#: Height at which the autopilot is engaged after takeoff. Below this the
#: aeroplane is flown by the flight director on runway heading.
AP_ENGAGE_AGL_FT = 400.0

#: Gear up once safely airborne and climbing.
GEAR_UP_AGL_FT = 200.0

#: Climb phase begins here, after the initial climb-out is established.
CLIMB_PHASE_AGL_FT = 1000.0

#: Cruise is reached when within this of the selected level.
CRUISE_CAPTURE_FT = 300.0

#: Approach phase begins at this distance to run.
APPROACH_ENTRY_NM = 25.0

#: The stabilisation gate. Below this the approach must be configured, on
#: speed and on the centreline, or it is a go-around.
STABILISATION_AGL_FT = 500.0

#: Landing phase, where the flare and touchdown are managed.
LANDING_AGL_FT = 900.0

#: Where control is handed back when the AI Pilot is not landing it.
HANDOVER_AGL_FT = 200.0

#: Where the handover is announced. Two hundred feet is about eight seconds
#: from the runway, which is not enough notice for someone who has been
#: watching rather than flying; a thousand gives them time to get their hands
#: on the controls before they are needed.
HANDOVER_WARNING_AGL_FT = 1000.0

#: Height above the field to climb to on a missed approach.
MISSED_APPROACH_HEIGHT_FT = 3000.0

#: Height at which the flare begins on a landing the AI Pilot is flying itself.
#: Chosen so the exponential law below hands over at roughly the rate the
#: approach was already being flown at, which makes the transition invisible.
FLARE_AGL_FT = 80.0

#: Flare time constant, in seconds. The flare law is the standard exponential
#: one -- descend at height divided by tau -- which is what makes a landing
#: soft: the rate goes to zero as the height does, instead of the aeroplane
#: arriving at whatever fixed rate it was told to hold. Eight seconds gives
#: about 600 fpm at eighty feet and 75 fpm at ten.
FLARE_TAU_S = 8.0

#: Bounds on the commanded flare rate.
FLARE_MIN_VS_FPM = -60.0
FLARE_MAX_VS_FPM = -900.0

#: Retard the thrust levers at this height, as the callout says.
RETARD_AGL_FT = 30.0


@dataclass
class PilotOptions:
    """Behaviour switches the user can set."""

    #: How the landing is finished.
    #: ``"auto"``    -- ILS autoland where there is an ILS, and the AI Pilot's
    #:                 own path-and-flare landing where there is not. This is
    #:                 what the MSFS 2020 AI Pilot did, and it is the default.
    #: ``"ils"``     -- ILS autoland only; hand over on approaches without one.
    #: ``"handover"``-- always hand over at :data:`HANDOVER_AGL_FT`, leaving the
    #:                 aeroplane stable on the centreline for you to land.
    autoland: str = "auto"
    #: Retract flaps and raise gear automatically after takeoff.
    manage_configuration: bool = True
    #: Turn the lights on and off at the usual points.
    manage_lights: bool = True
    #: Go around from an unstable approach rather than continuing.
    go_around_if_unstable: bool = True
    #: Maximum number of go-arounds before giving up and handing over.
    max_go_arounds: int = 1
    #: Start already airborne (the aeroplane is in the air when you engage).
    start_airborne: bool = False


@dataclass
class PilotStatus:
    """A snapshot for the user interface and the command line."""

    phase: Phase = Phase.PREFLIGHT
    engaged: bool = False
    message: str = ""
    position: LatLon = field(default_factory=lambda: LatLon(0.0, 0.0))
    altitude_ft: float = 0.0
    altitude_agl_ft: float = 0.0
    ias_kt: float = 0.0
    mach: float = 0.0
    ground_speed_kt: float = 0.0
    vertical_speed_fpm: float = 0.0
    heading_true_deg: float = 0.0
    track_true_deg: float = 0.0
    active_waypoint: str = ""
    active_index: int = 0
    distance_to_waypoint_nm: float = 0.0
    distance_to_destination_nm: float = 0.0
    cross_track_nm: float = 0.0
    time_enroute_s: float = 0.0
    eta_s: Optional[float] = None
    target_altitude_ft: float = 0.0
    target_speed: float = 0.0
    target_speed_is_mach: bool = False
    commanded_heading_deg: float = 0.0
    commanded_vs_fpm: Optional[float] = None
    path_deviation_ft: float = 0.0
    top_of_descent_nm: float = 0.0
    flaps_index: int = 0
    gear_down: bool = False
    autoland: bool = False
    go_arounds: int = 0

    @property
    def eta_text(self) -> str:
        if self.eta_s is None:
            return "--:--"
        minutes, seconds = divmod(int(self.eta_s), 60)
        hours, minutes = divmod(minutes, 60)
        return f"{hours:d}:{minutes:02d}:{seconds:02d}"


class AIPilot:
    """Flies a :class:`~aipilot.route.plan.FlightPlan` from start to finish."""

    def __init__(
        self,
        sim: SimBackend,
        adapter: AircraftAdapter,
        profile: AircraftProfile,
        plan: FlightPlan,
        options: Optional[PilotOptions] = None,
        listener: Optional[Callable[[FlightEvent], None]] = None,
    ) -> None:
        self.sim = sim
        self.adapter = adapter
        self.profile = profile
        self.plan = plan
        self.options = options or PilotOptions()
        self.log = EventLog(listener=listener)
        # Everything the adapter has to say -- gear, flaps, tuning, capability
        # warnings -- belongs in the same log as everything else.
        adapter.log = self._event

        self.vertical_profile = build_vertical_profile(
            plan.cruise_altitude_ft,
            plan.arrival_runway.elevation_ft if plan.arrival_runway
            else plan.destination.elevation_ft,
            profile,
        )
        self.lateral = LateralGuidance(plan, profile.max_bank_deg)
        self.vertical = VerticalGuidance(plan, profile, self.vertical_profile)

        self.phase = Phase.PREFLIGHT
        self.engaged = False
        self.status = PilotStatus(top_of_descent_nm=self.vertical_profile.top_of_descent_nm)
        self.elapsed_s = 0.0
        self._go_arounds = 0
        self._autoland_active = False
        self._handed_over = False
        self._nav_tuned = False
        self._preflight_done = False
        self._max_agl_seen = 0.0
        self._flaring = False
        self._gate_checked = False
        self._handover_warned = False
        self._missed_approach_alt: Optional[float] = None
        self._last_airborne_vs = 0.0
        self._commanded_flaps = 0
        self._touchdown_vs: Optional[float] = None

    # -- Lifecycle -----------------------------------------------------------
    def engage(self, state: Optional[SimState] = None) -> None:
        """Start flying. Equivalent to pressing the AI Pilot button."""
        self.engaged = True
        state = state or self.sim.poll(0.0)
        self.adapter.prepare()
        dep = self.plan.departure_runway.ident if self.plan.departure_runway else "?"
        arr = self.plan.arrival_runway.ident if self.plan.arrival_runway else "?"
        self._event(
            f"AI Pilot engaged: {self.plan.origin.icao}/{dep} to "
            f"{self.plan.destination.icao}/{arr}, {self.plan.total_distance_nm:.0f} nm "
            f"at FL{self.plan.cruise_altitude_ft / 100:.0f}"
        )
        self._event(self.adapter.describe())
        for warning in self.plan.warnings:
            self._event(warning, "warning")

        if self.options.start_airborne or not state.on_ground:
            self._enter_phase(Phase.CLIMB, "engaged in flight")
            self._establish_airborne(state)
        else:
            self.phase = Phase.PREFLIGHT

    def disengage(self, reason: str = "disengaged by the user") -> None:
        self.engaged = False
        self._event(reason, "warning")

    # -- Main loop -----------------------------------------------------------
    def update(self, dt: float) -> PilotStatus:
        """One control cycle. Call at a few hertz."""
        state = self.sim.poll(dt)
        self.elapsed_s += dt
        if not self.engaged:
            self._fill_status(state, None, None)
            return self.status

        self._max_agl_seen = max(self._max_agl_seen, state.altitude_agl_ft)
        if not state.on_ground:
            # The rate at touchdown has to be sampled before the wheels are
            # down; by the time the simulator reports on-ground it reads zero.
            self._last_airborne_vs = state.vertical_speed_fpm
        distance_to_go = self._distance_to_threshold_nm(state)

        self._update_phase(state, distance_to_go)

        lateral_command = None
        vertical_command = None
        if self.phase in (Phase.CLIMB, Phase.CRUISE, Phase.DESCENT,
                          Phase.APPROACH, Phase.LANDING):
            lateral_command = self._fly_lateral(state)
            vertical_command = self._fly_vertical(state, distance_to_go)
            if self.phase is Phase.LANDING:
                self._fly_flare(state)
        elif self.phase is Phase.TAKEOFF:
            self._fly_takeoff(state)
        elif self.phase is Phase.ROLLOUT:
            self._fly_rollout(state)

        if self.options.manage_configuration:
            self._manage_configuration(state, distance_to_go)
        if self.options.manage_lights:
            self._manage_lights(state)

        self._fill_status(state, lateral_command, vertical_command, distance_to_go)
        return self.status

    # -- Phase machine -------------------------------------------------------
    def _enter_phase(self, phase: Phase, reason: str = "") -> None:
        if phase_rank(phase) < phase_rank(self.phase) and phase is not Phase.APPROACH:
            return                      # phases do not run backwards
        if phase is self.phase:
            return
        self.phase = phase
        self._event(f"{phase.label}{f' -- {reason}' if reason else ''}")

    def _update_phase(self, state: SimState, distance_to_go: float) -> None:
        phase = self.phase
        if self._missed_approach_alt is not None:
            return          # the missed approach owns the phase until it levels

        if phase is Phase.PREFLIGHT:
            self._do_preflight(state)
            self._enter_phase(Phase.TAKEOFF, "cleared for takeoff")
            return

        if phase is Phase.TAKEOFF:
            if state.altitude_agl_ft >= CLIMB_PHASE_AGL_FT:
                self._enter_phase(Phase.CLIMB, "climb-out established")
            return

        if phase is Phase.CLIMB:
            if abs(state.altitude_ft - self.plan.cruise_altitude_ft) <= CRUISE_CAPTURE_FT:
                self._enter_phase(Phase.CRUISE,
                                  f"level at FL{self.plan.cruise_altitude_ft / 100:.0f}")
            elif should_start_descent(distance_to_go, self.vertical_profile, state.altitude_ft):
                # A very short sector can reach top of descent while still climbing.
                self._enter_phase(Phase.DESCENT, "top of descent reached during the climb")
            return

        if phase is Phase.CRUISE:
            if should_start_descent(distance_to_go, self.vertical_profile, state.altitude_ft):
                self._enter_phase(
                    Phase.DESCENT,
                    f"top of descent, {distance_to_go:.0f} nm to run",
                )
            return

        if phase is Phase.DESCENT:
            # Distance decides, not which leg is active: on a short sector the
            # approach fixes are the active leg from almost the moment of
            # takeoff, and keying off that flies the whole trip at 210 knots.
            if distance_to_go <= APPROACH_ENTRY_NM or \
                    self.lateral.active_leg.phase in ("final", "landing"):
                self._enter_phase(Phase.APPROACH, f"{distance_to_go:.0f} nm to run")
            return

        if phase is Phase.APPROACH:
            if state.on_ground:
                self._touchdown_vs = self._last_airborne_vs
                self._enter_phase(Phase.ROLLOUT, "touchdown")
            else:
                self._check_stabilisation(state)
                if self.phase is Phase.APPROACH and \
                        state.altitude_agl_ft <= LANDING_AGL_FT:
                    self._enter_phase(Phase.LANDING, "landing")
            return

        if phase is Phase.LANDING:
            self._check_stabilisation(state)
            if state.on_ground:
                # The state already reads zero once on the ground, so use the
                # rate from the cycle before touchdown.
                self._touchdown_vs = self._last_airborne_vs
                self._enter_phase(Phase.ROLLOUT, "touchdown")
            return

        if phase is Phase.ROLLOUT:
            if state.ground_speed_kt <= 30.0:
                self._enter_phase(Phase.COMPLETE, "clear of the runway speed")
                self._finish()
            return

    # -- Phase behaviour -----------------------------------------------------
    def _do_preflight(self, state: SimState) -> None:
        if self._preflight_done:
            return
        self._preflight_done = True
        runway = self.plan.departure_runway
        self.adapter.set_parking_brake(False, state)
        self._command_flaps(self.profile.takeoff_flaps_index, state)
        self.adapter.set_autothrottle(True)
        self.adapter.set_altitude(self.plan.cruise_altitude_ft)
        self.adapter.set_speed_kt(self.profile.v2_kt)
        if runway is not None:
            self.adapter.set_heading_true(runway.heading_true_deg, state)
        self.adapter.set_autobrake(2)
        if self.options.manage_lights:
            self.adapter.set_strobes(True)
            self.adapter.set_landing_lights(True)
            self.adapter.set_taxi_lights(True)
        self._tune_arrival_ils(state)
        self._event(
            f"Set up for departure: flaps {self.profile.takeoff_flaps_index}, "
            f"cruise FL{self.plan.cruise_altitude_ft / 100:.0f}, "
            f"top of descent {self.vertical_profile.top_of_descent_nm:.0f} nm out"
        )

    def _establish_airborne(self, state: SimState) -> None:
        """Set up when the AI Pilot is engaged with the aeroplane already flying."""
        self.adapter.set_autothrottle(True)
        self.adapter.engage_autopilot(state)
        self.adapter.select_heading_mode(state)
        self.adapter.set_altitude(self.plan.cruise_altitude_ft)
        self._tune_arrival_ils(state)
        # Fly to whichever fix is genuinely ahead rather than back to the start.
        self.lateral.direct_to(self._closest_useful_leg(state.position),
                               state.position)

    def _distance_to_threshold_nm(self, state: SimState) -> float:
        """Track miles to the landing threshold.

        The whole descent and approach is planned against distance to the
        *threshold*, so that is what has to be measured -- not distance to the
        end of the route, which runs three miles past it, and not straight-line
        distance, which ignores the miles still to be flown around the
        remaining turns. Past the threshold the result goes negative, which is
        what the flare and the rollout expect.
        """
        from ..geo import along_track_nm

        threshold = self.plan.threshold_index
        active = self.lateral.active_index
        threshold_position = self.plan.threshold_position

        if active > threshold:
            return -distance_nm(state.position, threshold_position)

        if active == threshold:
            start = self.plan[threshold - 1].position if threshold > 0 else state.position
            leg_length = distance_nm(start, threshold_position)
            if leg_length < 0.05:
                return distance_nm(state.position, threshold_position)
            return leg_length - along_track_nm(state.position, start, threshold_position)

        return distance_nm(state.position, self.plan[active].position) + \
            sum(distance_nm(a.position, b.position)
                for a, b in zip(self.plan.legs[active:threshold],
                                self.plan.legs[active + 1:threshold + 1]))

    def _closest_useful_leg(self, position: LatLon) -> int:
        """The fix to fly to when the AI Pilot is engaged already in the air.

        This is the "nearest leg" problem, and the obvious formulations get it
        badly wrong. Scoring each fix by how far away it is plus how much route
        is left after it looks reasonable and always picks the *last* fix,
        because along a route that is very nearly a straight line those two
        terms trade off exactly -- and the last fix wins the tie by being the
        end of a slightly longer path. An aeroplane over the Atlantic then
        decides its next waypoint is the threshold at Kennedy.

        So instead: find the leg whose centreline the aeroplane is actually
        nearest to, ignoring legs it has already flown past, and fly to the end
        of that one. That is what a crew looking at the map would pick.
        """
        from ..geo import along_track_nm, cross_track_nm

        best_index, best_distance = None, float("inf")
        for index in range(1, len(self.plan)):
            start = self.plan[index - 1].position
            end = self.plan[index].position
            leg_length = distance_nm(start, end)
            if leg_length < 0.1:
                continue
            travelled = along_track_nm(position, start, end)
            if travelled > leg_length:
                continue                      # this leg is behind us
            if travelled < 0.0:
                offset = distance_nm(position, start)   # not yet at the leg
            else:
                offset = abs(cross_track_nm(position, start, end))
            if offset < best_distance:
                best_index, best_distance = index, offset
        if best_index is None:
            # Past the end of every leg: aim at the last fix and let the
            # approach logic sort it out.
            return len(self.plan) - 1
        return best_index

    def _fly_takeoff(self, state: SimState) -> None:
        runway = self.plan.departure_runway
        self.adapter.takeoff_thrust()
        self.adapter.set_flaps(self._commanded_flaps, state)
        if runway is not None:
            self.adapter.set_heading_true(runway.heading_true_deg, state)
        self.adapter.set_speed_kt(self.profile.initial_climb_speed_kt)
        if state.altitude_agl_ft >= AP_ENGAGE_AGL_FT and not state.ap_master:
            self.adapter.engage_autopilot(state)
            self.adapter.select_heading_mode(state)
            self.adapter.set_altitude(self.plan.cruise_altitude_ft)
            self.adapter.select_altitude_mode(state)

    def _fly_lateral(self, state: SimState):
        approach_mode = self.phase in (Phase.APPROACH, Phase.LANDING)
        if self._autoland_active:
            # The aeroplane's own ILS guidance is flying; do not fight it.
            return self.lateral.update(state.position, state.tas_kt,
                                       state.wind_from_deg, state.wind_kt, approach_mode)
        command = self.lateral.update(
            state.position, state.tas_kt, state.wind_from_deg, state.wind_kt, approach_mode
        )
        if command.sequenced:
            leg = self.lateral.active_leg
            self._event(f"Now tracking {leg.ident}"
                        f"{f' ({leg.phase})' if leg.phase != 'enroute' else ''}")
        self.adapter.engage_autopilot(state)
        self.adapter.select_heading_mode(state)
        self.adapter.set_heading_true(command.heading_true_deg, state)
        return command

    def _fly_vertical(self, state: SimState, distance_to_go: float):
        if self._missed_approach_alt is not None:
            return self._fly_missed_approach(state, distance_to_go)
        command = self.vertical.update(
            self.phase, state.altitude_ft, max(0.0, distance_to_go),
            state.ground_speed_kt, self.lateral.active_index, state.altitude_agl_ft,
        )
        if self.phase is Phase.LANDING and not self._autoland_active \
                and not self._handed_over:
            # _fly_flare owns altitude and rate from here down; issuing an
            # altitude selection as well would level the aeroplane off just
            # above the runway, which is precisely what it must not do.
            self._command_speed(command)
            return command
        if self._autoland_active:
            # Glideslope is flying the vertical channel; only speed is ours.
            self._command_speed(command)
            return command

        self.adapter.set_altitude(command.altitude_ft)
        if command.vertical_speed_fpm is None:
            self.adapter.clear_vertical_speed()
            self.adapter.select_altitude_mode(state)
        else:
            self.adapter.set_vertical_speed(command.vertical_speed_fpm)
        self._command_speed(command)
        return command

    def _command_speed(self, command) -> None:
        if command.speed <= 0:
            return
        if command.speed_is_mach:
            self.adapter.set_mach(command.speed)
        else:
            self.adapter.set_speed_kt(command.speed)

    def _fly_flare(self, state: SimState) -> None:
        """The last hundred feet of a landing the AI Pilot is flying itself.

        On an ILS autoland the aeroplane's own logic does this and we stay out
        of the way. Otherwise the 3 degree path has to be broken manually: held
        all the way to the surface it arrives at some 700 fpm, which is an
        arrival rather than a landing. So below the flare height the vertical
        channel is commanded to a low fixed rate and the thrust is retarded,
        which is the same trade a pilot makes -- a slightly long touchdown in
        exchange for a soft one.
        """
        if self._autoland_active or self._handed_over:
            return
        runway = self.plan.arrival_runway
        field_elev = runway.elevation_ft if runway else self.plan.destination.elevation_ft

        # Put the altitude selector below the runway. An autopilot will level
        # off at whatever is selected, and on short final that is the one thing
        # it must never do -- the aeroplane would fly down the runway fifty feet
        # up until it ran out of fuel. Selecting a height below the surface
        # leaves vertical speed in sole command all the way to touchdown.
        self.adapter.set_altitude(field_elev - 500.0)

        if state.altitude_agl_ft <= RETARD_AGL_FT:
            self.adapter.idle_thrust()

        if state.altitude_agl_ft > FLARE_AGL_FT:
            # Hold the glidepath: the rate that arrives at the threshold at
            # fifty feet, bounded so a path correction never becomes a dive.
            target = -state.ground_speed_kt * 5.3 * (self.profile.descent_angle_deg / 3.0)
            self.adapter.set_vertical_speed(max(-1200.0, min(-200.0, target)))
            return

        if not self._flaring:
            self._flaring = True
            self._event("Flare")
        if runway is not None:
            # Kick off the drift so the aeroplane lands along the runway rather
            # than across it.
            self.adapter.set_heading_true(runway.heading_true_deg, state)
        # Descend at height over tau. The rate then decays with the height, so
        # the aeroplane settles onto the runway rather than flying into it: a
        # fixed flare rate, however small, is still being flown at the moment
        # the wheels arrive, and the autopilot's own rate limit means a step
        # command is never fully achieved before touchdown anyway.
        commanded = -state.altitude_agl_ft * 60.0 / FLARE_TAU_S
        commanded = max(FLARE_MAX_VS_FPM, min(FLARE_MIN_VS_FPM, commanded))
        self.adapter.clear_vertical_speed()
        self.adapter.set_vertical_speed(commanded)

    def _fly_rollout(self, state: SimState) -> None:
        self.adapter.idle_thrust()
        self.adapter.deploy_spoilers()
        if state.ap_master:
            self.adapter.disengage_autopilot(state)
        if state.ground_speed_kt > 30.0:
            self.adapter.apply_brakes()

    # -- Approach management -------------------------------------------------
    def _tune_arrival_ils(self, state: SimState) -> None:
        runway = self.plan.arrival_runway
        if self._nav_tuned or runway is None or not runway.has_ils:
            return
        self._nav_tuned = True
        self.adapter.tune_nav1(runway.ils_freq_mhz, runway.ils_course_true_deg, state)

    def _has_ils_autoland(self) -> bool:
        """Whether the aeroplane's own ILS guidance can fly this approach."""
        runway = self.plan.arrival_runway
        return bool(
            self.options.autoland in ("auto", "ils")
            and runway is not None and runway.has_ils
            and self.adapter.capabilities().autoland
        )

    def _will_land_itself(self) -> bool:
        """Whether the AI Pilot intends to complete the landing at all."""
        if self.options.autoland == "handover":
            return False
        if self.options.autoland == "ils":
            return self._has_ils_autoland()
        return True

    def _check_stabilisation(self, state: SimState) -> None:
        """The 500 ft gate: configured, on speed, on the centreline, or go around.

        Assessed once, on the way down through the gate height. Re-assessing
        below it would allow a go-around from fifty feet, which is a worse
        outcome than continuing with whatever the problem was; and assessing
        only on the phase change missed it entirely, because the phase changes
        at nine hundred feet and the gate is at five.
        """
        if state.altitude_agl_ft > STABILISATION_AGL_FT or self._gate_checked:
            return
        self._gate_checked = True
        vapp = self.profile.final_approach_speed_kt
        problems = []
        if state.ias_kt > vapp + 25.0:
            problems.append(f"{state.ias_kt - vapp:.0f} kt fast")
        if state.gear_down_pct < 95.0:
            problems.append("gear not down")
        if state.flaps_index < self.profile.landing_flaps_index - 1:
            problems.append("not configured for landing")
        if abs(self.status.cross_track_nm) > 0.5:
            problems.append(f"{abs(self.status.cross_track_nm):.1f} nm off the centreline")
        if state.vertical_speed_fpm < -1200.0:
            problems.append(f"{-state.vertical_speed_fpm:.0f} fpm descent")

        if not problems:
            return
        detail = ", ".join(problems)
        if self.options.go_around_if_unstable and self._go_arounds < self.options.max_go_arounds:
            self._go_around(state, detail)
        else:
            self._event(f"Approach is not stabilised ({detail}) -- continuing anyway",
                        "warning")

    def _go_around(self, state: SimState, reason: str) -> None:
        """Reject the approach, climb away, and set up to try again.

        The missed approach gets its own altitude target rather than reverting
        straight to the descent phase. Descent guidance is one-directional by
        construction -- it will never command a climb, since on a descent path
        that would be an error -- so dropping a go-around into it leaves the
        aeroplane stuck at five hundred feet with full thrust and nowhere to
        go, which is how the first version of this flew into New Jersey.
        """
        self._go_arounds += 1
        self._event(f"Going around: {reason}", "warning")
        self.adapter.clear_takeoff_thrust()
        self.adapter.takeoff_thrust()
        self._command_flaps(self.profile.takeoff_flaps_index, state)
        self.adapter.set_gear(False, state)
        self.adapter.retract_spoilers()
        field = (self.plan.arrival_runway.elevation_ft if self.plan.arrival_runway
                 else self.plan.destination.elevation_ft)
        self._missed_approach_alt = field + MISSED_APPROACH_HEIGHT_FT
        self.adapter.clear_vertical_speed()
        self.adapter.select_altitude_mode(state)
        self.adapter.set_altitude(self._missed_approach_alt)
        self.adapter.set_speed_kt(self.profile.initial_climb_speed_kt)
        self._autoland_active = False
        self._flaring = False
        self._handed_over = False
        self._handover_warned = False
        self._gate_checked = False
        self._nav_tuned = False
        self.adapter._approach_armed = False
        self.adapter._gear_down = None
        # Rejoin at the start of the approach and try again.
        intercept = self.plan.index_of_phase("approach")
        if intercept is not None:
            self.lateral.direct_to(intercept, state.position)
        self.phase = Phase.CLIMB
        self._event(
            f"Climbing to {self._missed_approach_alt:.0f} ft and repositioning "
            "for another approach"
        )

    def _fly_missed_approach(self, state: SimState, distance_to_go: float):
        """Vertical guidance while going around: climb to the missed altitude."""
        target = self._missed_approach_alt
        assert target is not None
        self.adapter.set_altitude(target)
        self.adapter.select_altitude_mode(state)
        self.adapter.set_speed_kt(self.profile.terminal_speed_kt)
        if abs(state.altitude_ft - target) <= 400.0:
            self._missed_approach_alt = None
            self.phase = Phase.DESCENT
            self._event("Level at the missed approach altitude, re-established "
                        "on the approach")
        from .vertical import VerticalCommand

        return VerticalCommand(altitude_ft=target, vertical_speed_fpm=None,
                               speed=self.profile.terminal_speed_kt,
                               speed_is_mach=False, reason="missed approach")

    # -- Configuration -------------------------------------------------------
    def _manage_configuration(self, state: SimState, distance_to_go: float) -> None:
        phase = self.phase
        if self._missed_approach_alt is not None:
            # Clean up on the go-around, then leave the configuration alone
            # until the approach is re-established.
            if state.altitude_agl_ft > GEAR_UP_AGL_FT:
                self.adapter.set_gear(False, state)
            self._retract_flaps(state)
            return

        if phase in (Phase.TAKEOFF, Phase.CLIMB):
            if state.altitude_agl_ft > GEAR_UP_AGL_FT and state.vertical_speed_fpm > 100:
                self.adapter.set_gear(False, state)
            if phase is Phase.CLIMB:
                self._retract_flaps(state)
            return

        if phase in (Phase.APPROACH, Phase.LANDING):
            self._configure_for_landing(state, distance_to_go)
            return

        if phase is Phase.ROLLOUT:
            self._command_flaps(0, state)

    def _retract_flaps(self, state: SimState) -> None:
        """Bring the flaps up one notch at a time as the aeroplane accelerates.

        The gate is the placard speed of the setting currently selected: flaps
        must come up *before* the aeroplane runs into their limit, so each notch
        is retracted as that limit is approached. The second condition simply
        cleans up the last notch once well established in the climb.
        """
        current = state.flaps_index
        if current <= 0:
            return
        setting = self.profile.flap(current)
        approaching_placard = setting is not None and state.ias_kt >= setting.max_speed_kt - 20.0
        well_established = state.ias_kt >= self.profile.initial_climb_speed_kt + 15.0
        if not (approaching_placard or well_established):
            return
        self._command_flaps(current - 1, state)

    def _command_flaps(self, target: int, state: SimState) -> None:
        """Select a flap setting, announcing it the first time it is asked for.

        The handle takes several seconds to reach a new detent and the adapter
        walks it there one notch per cycle, so the announcement has to hang off
        the *command* changing rather than off the handle arriving -- a
        retraction reaches its target and is then filtered out by the "already
        up" guard, and would never be announced at all.
        """
        if target != self._commanded_flaps:
            self._commanded_flaps = target
            setting = self.profile.flap(target)
            self._event(f"Flaps {setting}" if setting is not None else f"Flaps {target}")
        self.adapter.set_flaps(target, state)

    def _configure_for_landing(self, state: SimState, distance_to_go: float) -> None:
        runway = self.plan.arrival_runway
        target_flap = 0

        # Distance-based schedule, gated by the placard speed for each setting.
        if distance_to_go <= 6.0:
            target_flap = self.profile.landing_flaps_index
        elif distance_to_go <= 9.0:
            target_flap = max(1, self.profile.landing_flaps_index - 1)
        elif distance_to_go <= 13.0:
            target_flap = max(1, self.profile.landing_flaps_index - 2)
        elif distance_to_go <= 18.0:
            target_flap = 1

        allowed = self.profile.flap_for_speed(state.ias_kt)
        if allowed is not None:
            target_flap = min(target_flap, allowed.index)
        elif target_flap > 0:
            target_flap = 0            # too fast for any flap setting yet

        if target_flap > state.flaps_index or target_flap != self._commanded_flaps:
            self._command_flaps(max(target_flap, state.flaps_index), state)

        # Gear by ten miles or two thousand feet, whichever comes first.
        if (distance_to_go <= 10.0 or state.altitude_agl_ft <= 2000.0) and \
                state.ias_kt <= self.profile.gear_extend_speed_kt:
            self.adapter.set_gear(True, state)
            self.adapter.arm_spoilers()

        # Hand the approach to the aeroplane's own ILS guidance when we can.
        if runway is not None and runway.has_ils and distance_to_go <= 20.0 \
                and self.options.autoland in ("auto", "ils"):
            self.adapter.arm_approach(state)
            if not self._autoland_active and self.adapter.approach_is_captured(state):
                if self._has_ils_autoland():
                    self._autoland_active = True
                    self._event("Localizer and glideslope captured -- autoland engaged")

        if not self._will_land_itself():
            if not self._handover_warned and \
                    state.altitude_agl_ft <= HANDOVER_WARNING_AGL_FT:
                self._handover_warned = True
                self._event(
                    f"Stand by to take control: the landing is yours at "
                    f"{HANDOVER_AGL_FT:.0f} ft, about "
                    f"{HANDOVER_WARNING_AGL_FT / 60:.0f} seconds from now.",
                    "warning",
                )
            if not self._handed_over and state.altitude_agl_ft <= HANDOVER_AGL_FT:
                self._hand_over(state)

    def _hand_over(self, state: SimState) -> None:
        self._handed_over = True
        runway = self.plan.arrival_runway
        why = "no ILS at this runway" if (runway is None or not runway.has_ils) \
            else "you asked to land it yourself"
        self._event(
            f"YOUR CONTROLS -- {why}, so the landing is yours. The aeroplane is on "
            f"the centreline at {state.ias_kt:.0f} kt, {state.altitude_agl_ft:.0f} ft "
            "above the field, configured to land.",
            "warning",
        )
        self.adapter.disengage_autopilot(state)

    def _manage_lights(self, state: SimState) -> None:
        if self.phase is Phase.CLIMB and state.altitude_ft > 10000.0:
            self.adapter.set_landing_lights(False)
            self.adapter.set_taxi_lights(False)
        elif self.phase in (Phase.DESCENT, Phase.APPROACH) and state.altitude_ft < 10000.0:
            self.adapter.set_landing_lights(True)

    # -- Reporting -----------------------------------------------------------
    def _finish(self) -> None:
        self.engaged = False
        touchdown = ""
        if self._touchdown_vs is not None:
            touchdown = f" Touchdown at {abs(self._touchdown_vs):.0f} fpm."
        minutes = int(self.elapsed_s // 60)
        self._event(
            f"Flight complete at {self.plan.destination.icao} after "
            f"{minutes // 60}h {minutes % 60:02d}m.{touchdown}"
        )

    def _event(self, message: str, level: str = "info") -> FlightEvent:
        return self.log.add(self.elapsed_s, self.phase, message, level)

    def _fill_status(self, state: SimState, lateral_command, vertical_command,
                     distance_to_go: Optional[float] = None) -> None:
        status = self.status
        status.phase = self.phase
        status.engaged = self.engaged
        status.position = state.position
        status.altitude_ft = state.altitude_ft
        status.altitude_agl_ft = state.altitude_agl_ft
        status.ias_kt = state.ias_kt
        status.mach = state.mach
        status.ground_speed_kt = state.ground_speed_kt
        status.vertical_speed_fpm = state.vertical_speed_fpm
        status.heading_true_deg = state.heading_true_deg
        status.track_true_deg = state.track_true_deg
        status.flaps_index = state.flaps_index
        status.gear_down = state.gear_down_pct > 95.0
        status.time_enroute_s = self.elapsed_s
        status.autoland = self._autoland_active
        status.go_arounds = self._go_arounds
        status.active_index = self.lateral.active_index
        status.active_waypoint = self.lateral.active_leg.ident
        status.distance_to_destination_nm = (
            distance_to_go if distance_to_go is not None
            else self.lateral.distance_to_end_nm(state.position)
        )
        status.top_of_descent_nm = self.vertical_profile.top_of_descent_nm

        if lateral_command is not None:
            status.commanded_heading_deg = lateral_command.heading_true_deg
            status.cross_track_nm = lateral_command.cross_track_nm
            status.distance_to_waypoint_nm = lateral_command.distance_to_fix_nm
        if vertical_command is not None:
            status.target_altitude_ft = vertical_command.altitude_ft
            status.target_speed = vertical_command.speed
            status.target_speed_is_mach = vertical_command.speed_is_mach
            status.commanded_vs_fpm = vertical_command.vertical_speed_fpm
            status.path_deviation_ft = vertical_command.off_path_ft
            status.message = vertical_command.reason

        if state.ground_speed_kt > 40.0 and status.distance_to_destination_nm > 0:
            status.eta_s = status.distance_to_destination_nm / state.ground_speed_kt * 3600.0
        elif self.phase in (Phase.ROLLOUT, Phase.COMPLETE):
            status.eta_s = 0.0
