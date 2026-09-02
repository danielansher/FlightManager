'use strict';

const $ = (id) => document.getElementById(id);
const state = { plan: null, eventCount: 0, poll: null, trail: [] };

// --- Setup -------------------------------------------------------------------
async function loadAircraft() {
  const list = await (await fetch('/api/aircraft')).json();
  $('aircraft').innerHTML = list
    .filter((a) => a.key !== 'generic')
    .map((a) => `<option value="${a.key}">${a.name}</option>`)
    .join('');
}

$('sim').addEventListener('change', () => {
  $('speed-field').hidden = $('sim').value !== 'mock';
});

function request() {
  return {
    origin: $('origin').value.trim().toUpperCase(),
    destination: $('destination').value.trim().toUpperCase(),
    aircraft: $('aircraft').value,
    cruise: $('cruise').value.trim(),
    departure_runway: $('departure_runway').value.trim(),
    arrival_runway: $('arrival_runway').value.trim(),
    wind_from: $('wind_from').value.trim(),
    wind_kt: $('wind_kt').value.trim(),
    simbrief: $('simbrief').value.trim(),
    route: $('route').value.trim(),
    sim: $('sim').value,
    speed: parseFloat($('speed').value) || 1,
    autoland: $('autoland').value,
    airborne: $('airborne').checked,
    debug: $('debug').checked,
  };
}

function showMessages(warnings, error, notes) {
  const box = $('warnings');
  box.innerHTML = '';
  (notes || []).forEach((text) => {
    const el = document.createElement('div');
    el.className = 'note';
    el.textContent = text;
    box.appendChild(el);
  });
  if (error) {
    const el = document.createElement('div');
    el.className = 'error-box';
    el.textContent = error;
    box.appendChild(el);
  }
  (warnings || []).forEach((text) => {
    const el = document.createElement('div');
    el.className = 'warning';
    el.textContent = text;
    box.appendChild(el);
  });
}

$('plan-btn').addEventListener('click', async () => {
  $('plan-btn').disabled = true;
  try {
    const reply = await (await fetch('/api/plan', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(request()),
    })).json();
    if (!reply.ok) { showMessages([], reply.error); $('engage-btn').disabled = true; return; }
    state.plan = reply;
    state.trail = [];
    if (reply.simbrief) {
      $('origin').value = reply.origin.icao;
      $('destination').value = reply.destination.icao;
    }
    showMessages(reply.warnings, null, reply.runway_notes);
    $('engage-btn').disabled = false;
    $('map-panel').hidden = false;
    drawMap();
    $('connection').textContent =
      `${reply.origin.icao}/${reply.origin.runway} → ${reply.destination.icao}/${reply.destination.runway}`
      + ` · ${Math.round(reply.distance_nm)} nm · FL${Math.round(reply.cruise_ft / 100)}`;
    $('connection').className = 'pill idle';
  } catch (err) {
    showMessages([], String(err));
  } finally {
    $('plan-btn').disabled = false;
  }
});

$('engage-btn').addEventListener('click', async () => {
  $('engage-btn').disabled = true;
  try {
    const reply = await (await fetch('/api/engage', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(request()),
    })).json();
    if (!reply.ok) { showMessages([], reply.error); $('engage-btn').disabled = false; return; }
    state.eventCount = 0;
    $('log').innerHTML = '';
    if (reply.trace) {
      showMessages([], null, [
        `Recording a debug trace to ${reply.trace}. When the flight ends, `
        + `summarise it with:  python -m aipilot debug-report ${reply.trace}`,
      ]);
    }
    ['readout', 'log-panel'].forEach((id) => { $(id).hidden = false; });
    $('stop-btn').hidden = false;
    startPolling();
  } catch (err) {
    showMessages([], String(err));
    $('engage-btn').disabled = false;
  }
});

$('stop-btn').addEventListener('click', async () => {
  await fetch('/api/disengage', { method: 'POST' });
});

// --- Live state --------------------------------------------------------------
function startPolling() {
  if (state.poll) clearInterval(state.poll);
  state.poll = setInterval(tick, 500);
  tick();
}

async function tick() {
  let data;
  try {
    data = await (await fetch(`/api/state?since=${state.eventCount}`)).json();
  } catch (err) { return; }
  if (data.error) {
    $('connection').textContent = data.error;
    $('connection').className = 'pill error';
  }
  if (data.event_count !== undefined) state.eventCount = data.event_count;
  (data.events || []).forEach(appendEvent);
  if (data.phase === undefined) return;

  $('phase').textContent = data.phase_label;
  $('message').textContent = data.message || '';
  $('alt').textContent = Math.round(data.altitude_ft).toLocaleString();
  $('ias').textContent = Math.round(data.ias_kt);
  $('vs').textContent = (data.vertical_speed_fpm >= 0 ? '+' : '') + Math.round(data.vertical_speed_fpm);
  $('hdg').textContent = String(Math.round(data.heading_true_deg) % 360).padStart(3, '0');
  $('wpt').textContent = data.active_waypoint || '—';
  $('wptdist').textContent = data.distance_to_waypoint_nm
    ? `${data.distance_to_waypoint_nm.toFixed(1)} nm` : '';
  $('togo').textContent = Math.round(data.distance_to_destination_nm);
  $('eta').textContent = data.eta;
  $('target').textContent = data.target_speed_is_mach
    ? `M${data.target_speed.toFixed(2)}` : `${Math.round(data.target_speed)} kt`;
  $('targetalt').textContent = data.target_altitude_ft
    ? `to ${Math.round(data.target_altitude_ft).toLocaleString()} ft` : '';

  setChip('chip-gear', data.gear_down, data.gear_down ? 'gear down' : 'gear up');
  setChip('chip-flaps', data.flaps_index > 0, `flaps ${data.flaps_index}`);
  const xtk = Math.abs(data.cross_track_nm || 0);
  const xtkEl = $('chip-xtk');
  xtkEl.textContent = xtk < 0.15 ? 'on track' : `${xtk.toFixed(1)} nm off track`;
  xtkEl.className = 'chip' + (xtk > 1.5 ? ' alert' : '');
  const dev = data.path_deviation_ft || 0;
  const pathEl = $('chip-path');
  pathEl.textContent = Math.abs(dev) < 250 ? 'on path'
    : `${Math.abs(Math.round(dev))} ft ${dev > 0 ? 'high' : 'low'}`;
  pathEl.className = 'chip' + (Math.abs(dev) > 1200 ? ' alert' : '');
  $('chip-autoland').hidden = !data.autoland;
  $('chip-autoland').className = 'chip on';

  if (data.lat || data.lon) {
    const last = state.trail[state.trail.length - 1];
    if (!last || Math.hypot(last[0] - data.lat, last[1] - data.lon) > 0.02) {
      state.trail.push([data.lat, data.lon]);
      if (state.trail.length > 4000) state.trail.shift();
    }
    drawMap([data.lat, data.lon], data.track_true_deg);
  }

  $('connection').className = 'pill ' + (data.running ? 'live' : 'idle');
  if (!data.running) {
    $('connection').textContent = data.phase === 'complete' ? 'flight complete' : 'not running';
    $('stop-btn').hidden = true;
    $('engage-btn').disabled = false;
    if (state.poll) { clearInterval(state.poll); state.poll = null; }
  } else {
    $('connection').textContent = `flying · ${data.phase_label.toLowerCase()}`;
  }
}

function setChip(id, on, text) {
  const el = $(id);
  el.textContent = text;
  el.className = 'chip' + (on ? ' on' : '');
}

function appendEvent(event) {
  const el = document.createElement('div');
  const minutes = String(Math.floor(event.time_s / 60)).padStart(2, '0');
  const seconds = String(Math.floor(event.time_s % 60)).padStart(2, '0');
  if (event.level === 'warning') el.className = 'warning-line';
  if (event.level === 'error') el.className = 'error-line';
  el.innerHTML = `<span class="t">${minutes}:${seconds}</span> `
    + `<span class="p">${event.phase.padEnd(9)}</span> `;
  el.appendChild(document.createTextNode(event.message));
  const log = $('log');
  log.appendChild(el);
  log.scrollTop = log.scrollHeight;
}

// --- Map ---------------------------------------------------------------------
// An equirectangular plot with the longitude scale corrected for latitude. Not
// a projection anyone would navigate on, but over one flight it shows the route
// shape, the aeroplane's position on it, and where it has actually been -- which
// is what the panel is for.
function drawMap(aircraft, track) {
  if (!state.plan) return;
  const canvas = $('map');
  const ctx = canvas.getContext('2d');
  const W = canvas.width, H = canvas.height, pad = 34;
  ctx.clearRect(0, 0, W, H);

  const points = state.plan.legs.map((l) => [l.lat, l.lon]);
  if (aircraft) points.push(aircraft);
  const lons = points.map((p) => unwrap(p[1], points[0][1]));
  const lats = points.map((p) => p[0]);
  const meanLat = (Math.min(...lats) + Math.max(...lats)) / 2;
  const k = Math.max(0.15, Math.cos(meanLat * Math.PI / 180));

  let minX = Math.min(...lons) * k, maxX = Math.max(...lons) * k;
  let minY = Math.min(...lats), maxY = Math.max(...lats);
  const spanX = Math.max(maxX - minX, 0.02), spanY = Math.max(maxY - minY, 0.02);
  const scale = Math.min((W - 2 * pad) / spanX, (H - 2 * pad) / spanY);
  const offX = (W - spanX * scale) / 2, offY = (H - spanY * scale) / 2;
  const project = (lat, lon) => [
    offX + (unwrap(lon, points[0][1]) * k - minX) * scale,
    H - offY - (lat - minY) * scale,
  ];

  // Flown track.
  if (state.trail.length > 1) {
    ctx.strokeStyle = 'rgba(77,163,255,.30)';
    ctx.lineWidth = 4;
    ctx.beginPath();
    state.trail.forEach(([lat, lon], i) => {
      const [x, y] = project(lat, lon);
      i ? ctx.lineTo(x, y) : ctx.moveTo(x, y);
    });
    ctx.stroke();
  }

  // Planned route.
  ctx.strokeStyle = '#3d4657';
  ctx.lineWidth = 1.5;
  ctx.setLineDash([5, 4]);
  ctx.beginPath();
  state.plan.legs.forEach((leg, i) => {
    const [x, y] = project(leg.lat, leg.lon);
    i ? ctx.lineTo(x, y) : ctx.moveTo(x, y);
  });
  ctx.stroke();
  ctx.setLineDash([]);

  // Fixes. Only the enroute ones get labels, spaced out: the departure and
  // arrival fixes are all within a few miles of their airport and at this
  // scale their labels land on top of one another and become unreadable.
  const enroute = state.plan.legs.filter((l) => l.phase === 'enroute');
  const step = Math.max(1, Math.ceil(enroute.length / 10));
  let seen = 0;
  state.plan.legs.forEach((leg) => {
    const [x, y] = project(leg.lat, leg.lon);
    const terminal = leg.phase === 'takeoff' || leg.phase === 'landing';
    ctx.fillStyle = terminal ? '#3fb950' : '#4d5a70';
    ctx.beginPath();
    ctx.arc(x, y, terminal ? 4.5 : 2.5, 0, Math.PI * 2);
    ctx.fill();
    if (leg.phase === 'enroute' && seen++ % step === 0) {
      ctx.fillStyle = '#6b7787';
      ctx.font = '10px ui-monospace, monospace';
      ctx.fillText(leg.ident, x + 7, y - 5);
    }
  });

  // The airports, named once each. Each label goes on whichever side of its
  // dot has room, so neither is clipped by the edge of the canvas.
  ctx.font = '600 12px ui-monospace, monospace';
  ctx.fillStyle = '#3fb950';
  const label = (leg, icao) => {
    const [x, y] = project(leg.lat, leg.lon);
    const width = ctx.measureText(icao).width;
    const left = x + 9 + width > W - 6;
    ctx.fillText(icao, left ? x - 9 - width : x + 9, Math.min(H - 6, Math.max(12, y + 4)));
  };
  label(state.plan.legs[0], state.plan.origin.icao);
  label(state.plan.legs[state.plan.legs.length - 1], state.plan.destination.icao);

  // The aeroplane.
  if (aircraft) {
    const [x, y] = project(aircraft[0], aircraft[1]);
    ctx.save();
    ctx.translate(x, y);
    ctx.rotate(((track || 0) * Math.PI) / 180);
    ctx.fillStyle = '#4da3ff';
    ctx.beginPath();
    ctx.moveTo(0, -9); ctx.lineTo(6.5, 8); ctx.lineTo(0, 4.5); ctx.lineTo(-6.5, 8);
    ctx.closePath();
    ctx.fill();
    ctx.restore();
  }
}

// Keep longitudes continuous across the date line.
function unwrap(lon, reference) {
  let value = lon;
  while (value - reference > 180) value -= 360;
  while (value - reference < -180) value += 360;
  return value;
}

loadAircraft();
