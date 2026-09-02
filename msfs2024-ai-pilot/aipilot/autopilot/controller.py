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
from ..geo import (
    LatLon,
    along_track_nm,
    cross_track_nm,
    destination_point,
    distance_nm,
    signed_diff_deg,
)
from ..perf.profiles import AircraftProfile
from ..route.plan import FlightPlan
from ..route.planner import rebuild_departure
from ..route.profile import build_vertical_profile
from ..sim.base import SimBackend, SimState
from ..units import mach_to_tas, tas_to_cas
from ..geo import initial_bearing_deg, normalize_deg
from ..route.taxi import runway_entry_point, simplify
from .ground import (
    PUSHBACK_MAX_NM,
    PUSHBACK_MIN_NM,
    PushbackGuidance,
    TaxiGuidance,
    pushback_needed,
)
from .lateral import LateralGuidance
from .phases import EventLog, FlightEvent, Phase, phase_rank
from .vertical import VerticalGuidance, should_start_descent

#: Fallback height at which the autopilot is engaged after takeoff, for a type
#: whose profile does not say. Below this the aeroplane is flown on runway
#: heading. Per-type values live in ``AircraftProfile.min_ap_engage_agl_ft``,
#: because some aeroplanes will not hold an autopilot engagement this low.
AP_ENGAGE_AGL_FT = 400.0

#: How many uncommanded autopilot disconnects before the AI Pilot stops simply
#: putting it back and explains what is probably causing them.
AP_DISCONNECT_DIAGNOSIS_AT = 3

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
#: A floor, not a fixed value -- see :meth:`AIPilot._flare_height`.
FLARE_AGL_FT = 80.0

#: Seconds of flare wanted before the wheels arrive.
FLARE_LEAD_S = 5.0

#: Minimum height above the terrain the vertical channel will command, as a
#: function of how far there still is to run. There is no terrain lookahead
#: available through SimConnect -- only the elevation directly underneath --
#: so this cannot see a mountain coming. What it can do is refuse to keep
#: descending into ground that is rising under the aeroplane, which is the
#: situation that actually kills a flight into somewhere like Burbank.
TERRAIN_FLOOR_FAR_FT = 1500.0
TERRAIN_FLOOR_NEAR_FT = 500.0
TERRAIN_FLOOR_FAR_NM = 15.0
TERRAIN_FLOOR_NEAR_NM = 5.0

#: Below this height, descending at more than this rate, outside the landing
#: phase, is a recovery rather than a correction.
PULL_UP_AGL_FT = 400.0
PULL_UP_VS_FPM = -1000.0

#: How far above the commanded speed counts as an overspeed worth acting on.
OVERSPEED_MARGIN_KT = 15.0

#: Ground speed at which the landing roll is over and the taxi in begins.
TAXI_IN_HANDOVER_KT = 25.0

#: How often to ask the tug to disconnect, and how long to wait before giving
#: up on the simulator ever reporting that it has. The interval matters: the
#: event is a toggle, and the state that says whether it worked arrives at
#: roughly the rate this loop runs at, so asking every cycle toggles the tug
#: back on as often as off.
TUG_RELEASE_INTERVAL_S = 1.5
TUG_RELEASE_TIMEOUT_S = 20.0

#: Walking pace, for straightening out on the runway.
LINEUP_SPEED_KT = 5.0

#: How closely lined up the aeroplane must be before the takeoff roll starts:
#: about sixty feet off the centreline and six degrees of heading.
LINEUP_TOLERANCE_NM = 0.010
LINEUP_HEADING_DEG = 6.0

#: How long the autothrottle may be off the commanded speed before the AI
#: Pilot concludes it is not actually flying it and takes the levers.
AUTOTHROTTLE_DOUBT_S = 45.0

#: How fast the thrust levers may move, as a percentage of travel per second.
LEVER_RATE_PERCENT_PER_S = 12.0

#: Flare time constant, in seconds. The flare law is the standard exponential
#: one -- descend at height divided by tau -- which is what makes a landing
#: soft: the rate goes to zero as the height does, instead of the aeroplane
#: arriving at whatever fixed rate it was told to hold. Eight seconds gives
#: about 600 fpm at eighty feet and 75 fpm at ten.
FLARE_TAU_S = 8.0

#: Rate commanded once past the threshold, to settle the aeroplane on rather
#: than float it down the runway.
TOUCHDOWN_VS_FPM = -180.0

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
    #: Refuse to open the thrust levers until the aeroplane is on a runway.
    require_runway: bool = True
    #: Fly the thrust levers directly when the autothrottle is not holding the
    #: commanded speed, and protect against overspeed.
    manage_thrust: bool = True
    #: Refuse to descend below a minimum height above the terrain underneath.
    terrain_protection: bool = True
    #: Push back and taxi to the runway, where taxiway data is available.
    #: Without that data nothing moves on the ground, by design: guessing a
    #: path across an apron is how an aeroplane ends up in a building.
    taxi: bool = True


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
    ap_disconnects: int = 0

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
        ground: Optional[object] = None,
        arrival_ground: Optional[object] = None,
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
        self.ground_network = ground
        self.arrival_ground = arrival_ground
        self.arrival_stand = None
        self.taxi: Optional[TaxiGuidance] = None
        self.pushback: Optional[PushbackGuidance] = None
        self.lateral = LateralGuidance(plan, profile.max_bank_deg)
        self.vertical = VerticalGuidance(plan, profile, self.vertical_profile)

        self.phase = Phase.PREFLIGHT
        self.engaged = False
        self.status = PilotStatus(top_of_descent_nm=self.vertical_profile.top_of_descent_nm)
        self.elapsed_s = 0.0
        self._go_arounds = 0
        self._ap_disconnects = 0
        self._runway_wait_reported: Optional[float] = None
        self._runway_check_warned = False
        self._ground_started = False
        #: When the tug was first asked to leave, and when we last asked.
        self._tug_release_started: Optional[float] = None
        self._tug_release_sent = -TUG_RELEASE_INTERVAL_S
        self._overspeed = False
        self._speedbrake_out = False
        self._lever_percent = 60.0
        self._autothrottle_working: Optional[bool] = None
        self._autothrottle_doubt_s = 0.0
        self._terrain_limited = False
        self._last_dt = 1.0
        self._ap_ever_engaged = False
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
        self._last_dt = max(0.05, dt)
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
        self._watch_autopilot(state)

        lateral_command = None
        vertical_command = None
        if self.phase in (Phase.CLIMB, Phase.CRUISE, Phase.DESCENT,
                          Phase.APPROACH, Phase.LANDING):
            lateral_command = self._fly_lateral(state)
            vertical_command = self._fly_vertical(state, distance_to_go)
            if self.phase is Phase.LANDING:
                self._fly_flare(state, distance_to_go)
        elif self.phase is Phase.TAKEOFF:
            self._fly_takeoff(state)
        elif self.phase is Phase.PUSHBACK:
            self._fly_pushback(state)
        elif self.phase in (Phase.TAXI, Phase.TAXI_IN):
            self._fly_taxi(state)
        elif self.phase is Phase.ROLLOUT:
            self._fly_rollout(state)

        if self.options.manage_thrust and vertical_command is not None:
            self._manage_thrust(state, vertical_command)
        if self.options.manage_configuration:
            self._manage_configuration(state, distance_to_go)
        if self.options.manage_lights:
            self._manage_lights(state)

        self._fill_status(state, lateral_command, vertical_command, distance_to_go)
        return self.status

    # -- Phase machine -------------------------------------------------------
    def _enter_phase(self, phase: Phase, reason: str = "",
                     force: bool = False) -> None:
        if phase_rank(phase) < phase_rank(self.phase) \
                and phase is not Phase.APPROACH and not force:
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
            if not state.on_ground:
                self._enter_phase(Phase.CLIMB, "already airborne")
                self._establish_airborne(state)
                return
            if self._ready_for_takeoff(state):
                self._enter_phase(Phase.TAKEOFF, "lined up, cleared for takeoff")
            return

        if phase is Phase.PUSHBACK:
            if self.pushback is not None and self.pushback.finished(state.position):
                self._release_tug(state)
            return

        if phase is Phase.TAXI:
            # Properly lined up, not merely somewhere on the runway. The looser
            # "which runway is this" test tolerates being most of a runway
            # width off centre, which is fine for identifying a runway and no
            # good at all for starting a takeoff roll down one.
            if self._lined_up_on_departure_runway(state):
                self._ready_for_takeoff(state)
                self._enter_phase(Phase.TAKEOFF, "lined up, cleared for takeoff")
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
            if state.ground_speed_kt <= TAXI_IN_HANDOVER_KT:
                if self._start_taxi_in(state):
                    self._enter_phase(Phase.TAXI_IN, "vacating and taxiing in")
                else:
                    self._enter_phase(Phase.COMPLETE, "clear of the runway speed")
                    self._finish()
            return

        if phase is Phase.TAXI_IN:
            if self.taxi is not None and self.taxi.finished and \
                    state.ground_speed_kt < 1.0:
                self.adapter.set_parking_brake(True, state)
                self._enter_phase(Phase.COMPLETE, "on stand")
                self._finish()
            return

    # -- Phase behaviour -----------------------------------------------------
    def _do_preflight(self, state: SimState) -> None:
        if self._preflight_done:
            return
        self._preflight_done = True
        runway = self.plan.departure_runway
        self._command_flaps(self.profile.takeoff_flaps_index, state)
        self.adapter.set_altitude(self.plan.cruise_altitude_ft)
        self.adapter.set_speed_kt(self.profile.v2_kt)
        if runway is not None:
            self.adapter.set_heading_true(runway.heading_true_deg, state)
        self.adapter.set_autobrake(2)
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
        self.adapter.set_parking_brake(False, state)
        self.adapter.set_wheel_brakes(0.0)
        self.adapter.set_autothrottle(True)
        self.adapter.takeoff_thrust()
        if runway is not None and state.on_ground:
            # Keep straight on the runway with the nosewheel until flying.
            # Cross-track as well as heading: holding the runway heading while
            # displaced simply runs parallel to the centreline off the edge.
            length_nm = runway.length_ft / 6076.11548556
            far_end = destination_point(runway.threshold,
                                        runway.heading_true_deg, length_nm)
            offset = cross_track_nm(state.position, runway.threshold, far_end)
            error = signed_diff_deg(runway.heading_true_deg, state.heading_true_deg)
            command = error * 0.08 - offset * 25.0
            self.adapter.set_steering(max(-0.6, min(0.6, command)))
        elif not state.on_ground:
            self.adapter.set_steering(0.0)
        self.adapter.set_flaps(self._commanded_flaps, state)
        if runway is not None:
            self.adapter.set_heading_true(runway.heading_true_deg, state)
        self.adapter.set_speed_kt(self.profile.initial_climb_speed_kt)
        if state.altitude_agl_ft >= self._ap_engage_agl and not state.ap_master:
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
        command = self._apply_terrain_floor(command, state, max(0.0, distance_to_go))
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
        """Send the commanded speed, clamped to the envelope.

        A last line of defence rather than the main one. Nothing upstream
        should ever ask for more than Vmo, but a speed target is the one thing
        here that can break an aeroplane, and the cost of checking is nothing.
        """
        if command.speed <= 0:
            return
        if command.speed_is_mach:
            self.adapter.set_mach(min(command.speed, self.profile.max_mach - 0.01))
        else:
            self.adapter.set_speed_kt(min(command.speed, self.profile.vmo_kt - 10.0))

    def _fly_flare(self, state: SimState, distance_to_go_nm: float = 1.0) -> None:
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

        # Put the altitude selector below the ground. An autopilot will level
        # off at whatever is selected, and on short final that is the one thing
        # it must never do -- the aeroplane would fly down the runway fifty feet
        # up until it ran out of fuel. Selecting a height below the surface
        # leaves vertical speed in sole command all the way to touchdown.
        #
        # The reference is the terrain the simulator reports underneath, not
        # the field elevation from the navigation data. Where the two disagree
        # -- different scenery, an airport the data has at the wrong height --
        # the nav data figure can be above the real ground, and the aeroplane
        # levels off short of the runway and hovers.
        surface = min(field_elev, state.ground_elevation_ft or field_elev)
        self.adapter.set_altitude(surface - 500.0)

        if state.altitude_agl_ft <= RETARD_AGL_FT:
            self.adapter.idle_thrust()

        if state.altitude_agl_ft > self._flare_height(state):
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
        if distance_to_go_nm <= 0.0:
            # Past the threshold. The flare has done its job and the aeroplane
            # should now be on the runway, so it is planted rather than floated
            # -- an exponential flare over ground that keeps falling away will
            # hold an aeroplane a few feet up indefinitely.
            commanded = min(commanded, TOUCHDOWN_VS_FPM)
        commanded = max(FLARE_MAX_VS_FPM, min(FLARE_MIN_VS_FPM, commanded))
        self.adapter.clear_vertical_speed()
        self.adapter.set_vertical_speed(commanded)

    def _flare_height(self, state: SimState) -> float:
        """Where to start rounding out, given the rate and the control interval.

        A fixed height assumes the loop runs often enough to act on it. At a
        coarse control rate the aeroplane can fall through the whole flare
        window between two cycles and arrive at the approach rate, so the
        height is derived from how long the flare needs instead.
        """
        descent_ft_per_s = max(0.0, -state.vertical_speed_fpm) / 60.0
        needed = descent_ft_per_s * (FLARE_LEAD_S + 2.0 * self._last_dt)
        return max(FLARE_AGL_FT, needed)

    def _fly_rollout(self, state: SimState) -> None:
        self.adapter.idle_thrust()
        self.adapter.deploy_spoilers()
        if state.ap_master:
            self.adapter.disengage_autopilot(state)
        if state.ground_speed_kt > 30.0:
            self.adapter.apply_brakes()

    # -- Pushback and taxi ---------------------------------------------------
    def _begin_ground_movement(self, state: SimState) -> bool:
        """Start a pushback or a taxi, if there is anything to work with."""
        if not self.options.taxi or self.ground_network is None:
            return False
        if self._ground_started:
            return False
        self._ground_started = True

        runway = self.plan.departure_runway
        first_leg = None
        if runway is not None and self.ground_network is not None:
            route = self.ground_network.route(
                state.position, runway_entry_point(runway, self.ground_network))
            if len(route) > 1:
                first_leg = route[1]
        needed, distance = pushback_needed(state.position, self.ground_network,
                                           state.heading_true_deg, first_leg)
        if needed:
            # Leave the aeroplane facing the first leg it actually has to
            # drive, judged from where the pushback will have put it -- not the
            # direction of the taxiway it eventually joins. Facing the eventual
            # taxiway means coming off the stand pointing along a taxiway it
            # has not reached yet, and then swinging ninety degrees the wrong
            # way to get to it.
            facing = None
            if runway is not None:
                push = max(PUSHBACK_MIN_NM, min(PUSHBACK_MAX_NM, distance))
                end = destination_point(state.position,
                                        normalize_deg(state.heading_true_deg + 180.0),
                                        push)
                onward = self.ground_network.route(
                    end, runway_entry_point(runway, self.ground_network))
                if onward:
                    facing = initial_bearing_deg(end, onward[0])
                    if len(onward) > 1 and \
                            distance_nm(end, onward[0]) < PUSHBACK_MIN_NM:
                        facing = initial_bearing_deg(onward[0], onward[1])
            self.pushback = PushbackGuidance(state.position,
                                             state.heading_true_deg, distance,
                                             facing)
            self.adapter.set_wheel_brakes(0.0)
            self.adapter.set_parking_brake(False, state)
            self.adapter.set_pushback(True, state)
            self.adapter.set_tug_heading(self.pushback.final_heading)
            self._event(
                f"Pushing back {self.pushback.target_distance_nm * 6076:.0f} ft, "
                f"turning onto {self.pushback.final_heading:.0f} degrees.")
            self._enter_phase(Phase.PUSHBACK, "pushback")
            return True

        if self._start_taxi(state):
            self._enter_phase(Phase.TAXI, "taxiing to the runway")
            return True
        return False

    def _start_taxi(self, state: SimState) -> bool:
        """Work out a route across the taxiways to the departure runway."""
        runway = self.plan.departure_runway
        if self.ground_network is None or runway is None:
            return False
        entry = runway_entry_point(runway, self.ground_network)
        route = self.ground_network.route(state.position, entry)
        if not route:
            self._event(
                "Could not find a way across the taxiways to "
                f"{runway.ident}. Taxi out by hand and this will take over once "
                "you are lined up.", "warning")
            return False
        route = simplify(route)
        # Drop any leading points that are behind the aeroplane. The route
        # starts at the nearest point of the network, which after a pushback
        # can easily be a few yards *behind* -- and an aeroplane sent to a
        # point behind it turns right round on the spot to get there, which on
        # an apron is both wrong and slow.
        while len(route) > 1:
            bearing = initial_bearing_deg(state.position, route[0])
            if abs(signed_diff_deg(bearing, state.heading_true_deg)) < 100.0:
                break
            route.pop(0)
        # Then onto the runway itself and line up, so the takeoff check passes.
        route.append(runway.threshold)
        # Well down the runway, so the final turn has room to straighten.
        route.append(destination_point(runway.threshold,
                                       runway.heading_true_deg, 0.30))
        self.taxi = TaxiGuidance(route)
        distance = self.taxi.distance_remaining_nm(state.position)
        self._event(f"Taxiing to {runway.ident}: {distance:.2f} nm, "
                    f"{len(route)} turns.")
        self.adapter.set_parking_brake(False, state)
        return True

    def _start_taxi_in(self, state: SimState) -> bool:
        """Route from where the aeroplane stopped to a parking stand.

        Arriving is the other half of the job. Without this the AI Pilot brings
        an aeroplane down the approach, lands it, and abandons it on the runway
        -- which is worse than useless at a busy airport.
        """
        if not self.options.taxi or self.arrival_ground is None:
            return False
        layout = getattr(self.arrival_ground, "layout", None)
        stands = list(getattr(layout, "parking", ()) or ())
        if not stands:
            self._event("No stand data for this airport, so the taxi in is "
                        "yours. Vacate when ready.", "warning")
            return False

        # The nearest stand the network can actually reach. Nearest by straight
        # line is not the same thing: the closest one may be on the other side
        # of the runway with no route to it.
        stands.sort(key=lambda stand: distance_nm(state.position, stand.position))
        for stand in stands[:12]:
            route = self.arrival_ground.route(state.position, stand.position)
            if not route:
                continue
            route = simplify(route)
            route.append(stand.position)
            self.taxi = TaxiGuidance(route)
            self.arrival_stand = stand
            distance = self.taxi.distance_remaining_nm(state.position)
            self._event(f"Vacating and taxiing to {stand.name}: "
                        f"{distance:.2f} nm.")
            return True

        self._event("Could not find a way across the taxiways to a stand, so "
                    "the taxi in is yours.", "warning")
        return False

    def _fly_pushback(self, state: SimState) -> None:
        if self.pushback is None or self.pushback.done:
            # Nothing once the push is over. Sending a tug heading is how a
            # pushback is *started*, not merely how a running one is steered,
            # so a tug heading sent after asking the tug to leave summons it
            # straight back. The aeroplane then sits on the stand with its
            # nosewheel swinging, held by a tug that is re-attached as fast as
            # it is released, and no amount of thrust will move it.
            return
        # The tug event takes the heading the aeroplane should end up on.
        self.adapter.set_tug_heading(self.pushback.final_heading)
        self.adapter.set_wheel_brakes(0.0)

    def _release_tug(self, state: SimState) -> None:
        """Stop, let the tug go, and start taxiing.

        The tug is asked to leave on a cooldown rather than every cycle. The
        event is a toggle and the state that says whether it worked arrives at
        about the rate this loop runs at, so sending it every cycle toggles the
        tug back on as often as off.
        """
        self.adapter.set_throttle_percent(0.0)
        self.adapter.set_wheel_brakes(1.0)

        if self._tug_release_started is None:
            self._tug_release_started = self.elapsed_s

        waited = self.elapsed_s - self._tug_release_started
        if state.pushback_attached and \
                self.elapsed_s - self._tug_release_sent >= TUG_RELEASE_INTERVAL_S:
            self._tug_release_sent = self.elapsed_s
            self.adapter.set_pushback(False, state)

        stopped = state.ground_speed_kt < 1.0 and not state.pushback_attached
        if not stopped:
            if waited < TUG_RELEASE_TIMEOUT_S:
                self.status.message = "waiting for the tug to disconnect"
                return
            # Some aircraft never report the tug leaving. Rather than sit on
            # the stand for ever, say so and get on with the taxi.
            self._event(
                "The simulator still reports a tug attached "
                f"{TUG_RELEASE_TIMEOUT_S:.0f} seconds after asking it to "
                "disconnect. Carrying on with the taxi. If the aeroplane does "
                "not move, press the pushback key yourself to release it.",
                "warning")

        self.pushback = None
        self._tug_release_started = None
        if self._start_taxi(state):
            self.adapter.set_wheel_brakes(0.0)
            self._enter_phase(Phase.TAXI, "taxiing to the runway")
            return
        # No route off the stand. Going to TAXI here would be a trap: the taxi
        # phase with nothing to follow steers nothing and commands nothing, so
        # the aeroplane stands on the apron for ever and the phase machine will
        # not run backwards to let it recover. Waiting is the honest state --
        # it is on the ground, going nowhere, and needs the pilot -- and from
        # there it still takes over by itself the moment it is lined up.
        self.adapter.set_wheel_brakes(1.0)
        self.adapter.set_parking_brake(True, state)
        self._runway_wait_reported = None
        self._enter_phase(Phase.PREFLIGHT, "pushback complete, waiting",
                          force=True)

    def _fly_taxi(self, state: SimState) -> None:
        """Steer and control speed along the taxi route."""
        if self.taxi is None:
            return
        command = self.taxi.update(state)
        if command.finished:
            if self.phase is Phase.TAXI_IN:
                self.adapter.set_steering(0.0)
                self.adapter.set_throttle_percent(0.0)
                self.adapter.set_wheel_brakes(1.0)
                self.status.message = "on stand"
            else:
                self._line_up(state)
            return
        self.status.message = command.reason
        self.adapter.set_steering(command.steering)
        excess = state.ground_speed_kt - command.target_speed_kt
        if excess > 1.5:
            self.adapter.set_throttle_percent(0.0)
            self.adapter.set_wheel_brakes(min(1.0, excess / 6.0))
        else:
            self.adapter.set_wheel_brakes(0.0)
            self.adapter.set_throttle_percent(max(6.0, 22.0 - excess * 6.0))

    def _line_up(self, state: SimState) -> None:
        """Creep forward on the centreline until properly aligned.

        The taxi route ends on the runway, but arriving on a runway and being
        lined up along it are different things: the last turn is a ninety
        degree one taken at walking pace, and an aeroplane comes out of it
        pointing some way off. Rather than stopping there and calling it lined
        up, it straightens out first -- which is what a crew does, and what the
        takeoff roll needs, since a roll that starts fifteen degrees off runs
        out of runway sideways.
        """
        runway = self.plan.departure_runway
        if runway is None:
            self.adapter.set_throttle_percent(0.0)
            self.adapter.set_wheel_brakes(1.0)
            return
        length_nm = runway.length_ft / 6076.11548556
        far_end = destination_point(runway.threshold, runway.heading_true_deg,
                                    length_nm)
        offset = cross_track_nm(state.position, runway.threshold, far_end)
        error = signed_diff_deg(runway.heading_true_deg, state.heading_true_deg)
        self.adapter.set_steering(max(-0.8, min(0.8, error * 0.06 - offset * 30.0)))
        self.status.message = (f"lining up on {runway.ident}: "
                               f"{abs(error):.0f} degrees, "
                               f"{abs(offset) * 6076:.0f} ft off the centreline")
        if state.ground_speed_kt > LINEUP_SPEED_KT + 1.5:
            self.adapter.set_throttle_percent(0.0)
            self.adapter.set_wheel_brakes(0.4)
        else:
            self.adapter.set_wheel_brakes(0.0)
            self.adapter.set_throttle_percent(16.0)

    # -- Thrust, and not letting the speed run away --------------------------
    def _target_speed_kt(self, command, altitude_ft: float) -> float:
        """The commanded speed as an indicated airspeed, whatever it was set in.

        The altitude comes from the live state, not from the status snapshot:
        the snapshot is filled at the end of the cycle, so on the first pass it
        still reads zero -- and Mach 0.80 converted at sea level is 529 knots,
        which the thrust controller then dutifully chases.
        """
        if not command.speed_is_mach:
            return command.speed
        altitude = max(altitude_ft, 0.0)
        return tas_to_cas(mach_to_tas(command.speed, altitude), altitude)

    def _manage_thrust(self, state: SimState, command) -> None:
        """Keep the speed under control, whether or not the autothrottle helps.

        An armed autothrottle is not the same as an autothrottle that is flying
        the aeroplane, and on some aircraft it quietly does not take the levers
        at all. Whatever position they were last commanded to then stays --
        and after takeoff that is full power. The aeroplane holds its vertical
        speed by pitching, the thrust never comes back, and it arrives in the
        descent at four hundred and fifty knots.

        So the speed is checked against what was asked for. If the autothrottle
        is doing its job, nothing happens here. If it is not, the levers are
        flown directly.
        """
        if self.phase in (Phase.PREFLIGHT, Phase.TAKEOFF, Phase.ROLLOUT,
                          Phase.COMPLETE):
            return
        target = self._target_speed_kt(command, state.altitude_ft)
        if target <= 0:
            return
        excess = state.ias_kt - target
        limit = min(self.profile.vmo_kt, target + OVERSPEED_MARGIN_KT)

        if state.ias_kt > self.profile.vmo_kt:
            if not self._overspeed:
                self._overspeed = True
                self._event(
                    f"Overspeed: {state.ias_kt:.0f} kt against a limit of "
                    f"{self.profile.vmo_kt:.0f}. Closing the thrust levers and "
                    "using the speedbrake.", "warning")
            self._lever_percent = 0.0
            self.adapter.set_throttle_percent(0.0)
            self.adapter.set_speedbrake_percent(100.0)
            return
        if self._overspeed and state.ias_kt < target + 5.0:
            self._overspeed = False
            self.adapter.set_speedbrake_percent(0.0)
            self._event("Back within limits.")

        if state.ap_autothrottle and abs(excess) < OVERSPEED_MARGIN_KT:
            self._autothrottle_doubt_s = 0.0
            self._autothrottle_working = True
            return                     # the aeroplane is flying it; leave it be

        if state.ap_autothrottle and self._autothrottle_working is not False:
            # Give it time before concluding anything. Speed lags the target
            # for a while after every thrust change -- accelerating through
            # the climb, decelerating on the approach -- and judging on a
            # single cycle declares a perfectly good autothrottle broken about
            # a minute into every flight.
            self._autothrottle_doubt_s += self._last_dt
            if self._autothrottle_doubt_s < AUTOTHROTTLE_DOUBT_S:
                return
            self._autothrottle_working = False
            self._event(
                "The autothrottle is armed but has not held the speed for "
                f"{AUTOTHROTTLE_DOUBT_S:.0f} seconds, so the thrust levers are "
                "being flown directly instead.", "warning")

        if abs(excess) < 6.0:
            return

        # A proportional lever position around a per-phase trim setting.
        base = {Phase.CLIMB: 88.0, Phase.CRUISE: 68.0, Phase.DESCENT: 22.0,
                Phase.APPROACH: 40.0, Phase.LANDING: 35.0}.get(self.phase, 55.0)
        self._move_levers(base - excess * 3.0)
        if excess > OVERSPEED_MARGIN_KT and self.phase in (Phase.DESCENT,
                                                           Phase.APPROACH):
            self.adapter.set_speedbrake_percent(min(100.0, excess * 4.0))
        elif excess < 0 and self._speedbrake_out:
            self.adapter.set_speedbrake_percent(0.0)
        self._speedbrake_out = excess > OVERSPEED_MARGIN_KT

    def _move_levers(self, wanted_percent: float) -> None:
        """Move the thrust levers towards a position, at a believable rate.

        Commanding the position outright makes the levers slam between idle and
        full as the speed crosses the target, which no autothrottle does and no
        engine would enjoy. Real thrust levers take a few seconds end to end.
        """
        wanted = max(0.0, min(100.0, wanted_percent))
        current = self._lever_percent
        step = LEVER_RATE_PERCENT_PER_S * self._last_dt
        if abs(wanted - current) <= step:
            self._lever_percent = wanted
        else:
            self._lever_percent = current + (step if wanted > current else -step)
        self.adapter.set_throttle_percent(self._lever_percent)

    # -- Terrain -------------------------------------------------------------
    def _terrain_floor_agl_ft(self, distance_to_go_nm: float) -> float:
        """Minimum height above the ground underneath, by distance to run."""
        if distance_to_go_nm >= TERRAIN_FLOOR_FAR_NM:
            return TERRAIN_FLOOR_FAR_FT
        if distance_to_go_nm <= TERRAIN_FLOOR_NEAR_NM:
            return 0.0            # on final: the glidepath is the floor
        span = TERRAIN_FLOOR_FAR_NM - TERRAIN_FLOOR_NEAR_NM
        fraction = (distance_to_go_nm - TERRAIN_FLOOR_NEAR_NM) / span
        return TERRAIN_FLOOR_NEAR_FT + fraction * (TERRAIN_FLOOR_FAR_FT
                                                   - TERRAIN_FLOOR_NEAR_FT)

    def _apply_terrain_floor(self, command, state: SimState,
                             distance_to_go_nm: float):
        """Refuse to fly the descent path into the ground.

        SimConnect reports the terrain elevation under the aeroplane but has no
        way to ask about terrain ahead, so this cannot see a ridge coming. What
        it does do is notice the ground rising underneath and stop descending
        into it, which is the case that turns a short sector into somewhere
        like Burbank into a hillside.
        """
        if not self.options.terrain_protection:
            return command
        if self.phase in (Phase.PREFLIGHT, Phase.PUSHBACK, Phase.TAXI,
                          Phase.TAKEOFF, Phase.CLIMB, Phase.ROLLOUT,
                          Phase.COMPLETE):
            return command
        if self._autoland_active or self._handed_over:
            return command

        floor_agl = self._terrain_floor_agl_ft(distance_to_go_nm)
        if floor_agl <= 0.0:
            return command
        floor_alt = state.ground_elevation_ft + floor_agl

        # The floor may only ever arrest a descent. An earlier version applied
        # it to whatever the vertical channel had asked for, which turned a
        # climb through the floor into a climb *to* the floor: the aeroplane
        # levelled at fifteen hundred feet on departure and flew the entire
        # sector there, because the commanded rate shrank to nothing as it
        # approached the very height it was trying to climb away from.
        emergency = (state.altitude_agl_ft < PULL_UP_AGL_FT
                     and state.vertical_speed_fpm < PULL_UP_VS_FPM
                     and self.phase is not Phase.LANDING)
        # Keyed on where the aeroplane *is*, not on what is in the altitude
        # selector. On final the selector is deliberately at or below the
        # runway -- that is what landing is -- and treating that as flying into
        # the ground produces a terrain warning on every approach, which is the
        # fastest way to teach someone to ignore terrain warnings.
        commanded_down = (command.vertical_speed_fpm is not None
                          and command.vertical_speed_fpm < 0.0)
        going_down = state.vertical_speed_fpm < -100.0
        losing_ground = (commanded_down or going_down) and \
            state.altitude_ft <= floor_alt
        if not emergency and not losing_ground:
            self._terrain_limited = False
            return command

        from dataclasses import replace as _replace

        if not self._terrain_limited:
            self._terrain_limited = True
            self._event(
                f"Terrain: only {state.altitude_agl_ft:.0f} ft above the ground "
                f"with {distance_to_go_nm:.0f} nm to run. Levelling off at "
                f"{floor_alt:.0f} ft rather than continuing down.", "warning")
        climb = 1200.0 if emergency else max(0.0, (floor_alt - state.altitude_ft) * 2.0)
        return _replace(command, altitude_ft=max(command.altitude_ft, floor_alt),
                        vertical_speed_fpm=min(1800.0, climb),
                        reason="held off by terrain")

    # -- Not taking off from the apron ---------------------------------------
    def _runway_under_aircraft(self, state: SimState) -> Optional[object]:
        """The departure runway the aeroplane is actually lined up on, if any."""
        for runway in self.plan.origin.runways:
            length_nm = runway.length_ft / 6076.11548556
            if length_nm < 0.05:
                continue
            far_end = destination_point(runway.threshold,
                                        runway.heading_true_deg, length_nm)
            along = along_track_nm(state.position, runway.threshold, far_end)
            if not -0.15 <= along <= length_nm + 0.05:
                continue
            # Half the runway width plus a margin: enough to be lined up
            # slightly off centre, not enough to be on a parallel taxiway.
            tolerance = (runway.width_ft / 2.0 + 120.0) / 6076.11548556
            if abs(cross_track_nm(state.position, runway.threshold, far_end)) > tolerance:
                continue
            if abs(signed_diff_deg(state.heading_true_deg,
                                   runway.heading_true_deg)) > 30.0:
                continue
            return runway
        return None

    def _lined_up_on_departure_runway(self, state: SimState) -> bool:
        """Tight alignment test, for handing over from taxi to takeoff."""
        runway = self.plan.departure_runway
        if runway is None or not state.on_ground:
            return False
        length_nm = runway.length_ft / 6076.11548556
        far_end = destination_point(runway.threshold, runway.heading_true_deg,
                                    length_nm)
        along = along_track_nm(state.position, runway.threshold, far_end)
        if not -0.05 <= along <= length_nm * 0.6:
            return False
        if abs(cross_track_nm(state.position, runway.threshold, far_end)) > \
                LINEUP_TOLERANCE_NM:
            return False
        return abs(signed_diff_deg(state.heading_true_deg,
                                   runway.heading_true_deg)) <= LINEUP_HEADING_DEG

    def _ready_for_takeoff(self, state: SimState) -> bool:
        """Whether it is safe to open the thrust levers.

        The AI Pilot does not taxi, and an earlier version simply assumed the
        aeroplane was lined up. Run from a gate, it applied takeoff thrust on
        the apron and drove into a terminal building. So: check, and wait.
        """
        if not self.options.require_runway:
            return True
        if not self.plan.origin.runways:
            if not self._runway_check_warned:
                self._runway_check_warned = True
                self._event(
                    f"No runway data for {self.plan.origin.icao}, so there is no "
                    "way to check the aeroplane is on one. Make sure it is lined "
                    "up before this rolls.", "warning")
            return True

        runway = self._runway_under_aircraft(state)
        if runway is not None:
            if self.plan.departure_runway is None or \
                    runway.ident != self.plan.departure_runway.ident:
                rebuild_departure(self.plan, runway, self.profile)
                self.lateral = LateralGuidance(self.plan, self.profile.max_bank_deg)
                self._event(f"Lined up on {runway.ident} rather than the planned "
                            f"runway -- using {runway.ident}.")
            return True

        if self._begin_ground_movement(state):
            return False
        self._report_waiting_for_runway(state)
        return False

    def _report_waiting_for_runway(self, state: SimState) -> None:
        """Say, once and then occasionally, that it is waiting to be lined up."""
        now = self.elapsed_s
        if self._runway_wait_reported and now - self._runway_wait_reported < 30.0:
            return
        first = self._runway_wait_reported is None
        self._runway_wait_reported = now

        nearest, distance = None, float("inf")
        for runway in self.plan.origin.runways:
            d = distance_nm(state.position, runway.threshold)
            if d < distance:
                nearest, distance = runway, d
        where = (f" Nearest runway is {nearest.ident}, {distance:.1f} nm away."
                 if nearest is not None else "")
        if first:
            self._event(
                "Waiting: the aeroplane is not lined up on a runway, and the AI "
                "Pilot does not taxi. Taxi out and line up, and it will take "
                "over by itself as soon as you do." + where, "warning")
        else:
            self._event(f"Still waiting to be lined up on a runway.{where}")

    # -- Keeping hold of the autopilot ---------------------------------------
    @property
    def _ap_engage_agl(self) -> float:
        return self.profile.min_ap_engage_agl_ft or AP_ENGAGE_AGL_FT

    def _watch_autopilot(self, state: SimState) -> None:
        """Notice when the aeroplane drops the autopilot, and put it back.

        The commonest complaint about the simulator's own AI pilot is that it
        engages and then quietly stops flying, leaving the aeroplane to drift
        off while everything still looks normal. The cause is usually not in
        the aeroplane at all: a joystick or rudder axis with a little jitter on
        it reads as a control input and disconnects the autopilot, and nothing
        announces that it has happened.

        So rather than assume the engagement holds, this checks every cycle. A
        one-off is put back silently enough not to be noise. A pattern of them
        is a configuration problem the user has to fix, and no amount of
        re-engaging will help, so at that point it says what to go and look at
        instead of quietly fighting it for the rest of the flight.
        """
        if not self.phase.airborne or self._handed_over:
            return
        if state.altitude_agl_ft < self._ap_engage_agl:
            return
        if state.ap_master:
            self._ap_ever_engaged = True
            return
        if not self._ap_ever_engaged:
            return            # not yet engaged for the first time; takeoff owns that

        self._ap_disconnects += 1
        self.adapter.engage_autopilot(state)
        # A disconnect drops the lateral and vertical modes with it.
        self.adapter.select_heading_mode(state)

        if self._ap_disconnects == 1:
            self._event("The aeroplane dropped the autopilot on its own -- "
                        "re-engaging.", "warning")
        elif self._ap_disconnects == AP_DISCONNECT_DIAGNOSIS_AT:
            self._event(
                f"The autopilot has now disconnected by itself "
                f"{self._ap_disconnects} times. That is almost always "
                "something outside the aeroplane. In order of likelihood: a "
                "joystick, rudder or trim axis with a little jitter on it, "
                "which reads as a control input (give it a small dead zone); a "
                "control bound twice, or an autopilot toggle bound to a switch "
                "that is being held; or the simulator's own AI piloting "
                "assistance switched on and fighting for the aeroplane. "
                "See docs/MSFS2020.md.",
                "warning",
            )

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
        """Lights and cabin signs, to the schedule an airline crew works to.

        Nav lights and the beacon stay on whenever the engines are turning. The
        strobes and landing lights belong to the runway: on when lining up, off
        once clear of it the other end. The taxi light is for taxiing and comes
        off entering the runway. Ten thousand feet is the dividing line for the
        landing lights and the seatbelt sign on the way up, and the start of
        the descent brings both back.
        """
        self.adapter.tick_switches()
        phase = self.phase
        on_runway = phase in (Phase.TAKEOFF, Phase.LANDING, Phase.ROLLOUT)
        airborne = phase.airborne
        below_ten = state.altitude_ft < 10000.0

        self.adapter.set_nav_lights(True, state)
        self.adapter.set_beacon(state.engines_running or phase is not Phase.COMPLETE,
                                state)
        self.adapter.set_no_smoking_sign(True, state)

        # The strobes mark occupying a runway.
        self.adapter.set_strobes(on_runway or airborne, state)

        # Taxi light for taxiing and pushback; off once lined up.
        self.adapter.set_taxi_lights(
            phase in (Phase.PREFLIGHT, Phase.PUSHBACK, Phase.TAXI, Phase.TAXI_IN)
            or (phase is Phase.ROLLOUT and state.ground_speed_kt < 40.0), state)

        # Landing lights from lining up until ten thousand feet, and again from
        # ten thousand on the way down until clear of the runway.
        self.adapter.set_landing_lights(
            on_runway or (airborne and below_ten), state)

        # Wing and logo lights are ground and low-level items.
        self.adapter.set_wing_lights(not airborne or below_ten, state)
        self.adapter.set_logo_lights(not airborne or below_ten, state)

        # Seatbelts on from pushback to ten thousand feet, and from top of
        # descent to the gate.
        self.adapter.set_seatbelt_sign(
            phase is not Phase.COMPLETE
            and (not airborne or below_ten
                 or phase in (Phase.DESCENT, Phase.APPROACH, Phase.LANDING)),
            state)

    # -- Reporting -----------------------------------------------------------
    def _finish(self) -> None:
        self.engaged = False
        touchdown = ""
        if self._touchdown_vs is not None:
            touchdown = f" Touchdown at {abs(self._touchdown_vs):.0f} fpm."
        if self.arrival_stand is not None:
            touchdown += f" Parked at {self.arrival_stand.name}."
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
        status.ap_disconnects = self._ap_disconnects
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
