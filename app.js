'use strict';
/* ============================================================
   GROUNDSTATION — drone flight analysis for pilot training
   Matches DJI SRT telemetry to the video, frame by frame.
   ============================================================ */

// ---------- helpers ----------
const $ = id => document.getElementById(id);
const clamp = (v, a, b) => Math.min(b, Math.max(a, v));

function fmtT(s) {                       // 00:00.00
  if (!isFinite(s)) s = 0;
  const m = Math.floor(s / 60), sec = s - m * 60;
  return String(m).padStart(2, '0') + ':' + sec.toFixed(2).padStart(5, '0');
}
function fmtTc(s) {                      // 00:00:00 = min:sec:frame
  if (!isFinite(s)) s = 0;
  const fps = S.fps || 50;
  const tot = Math.round(s * fps);
  const ff = tot % fps;
  const secs = Math.floor(tot / fps);
  const m = Math.floor(secs / 60), sec = secs % 60;
  return String(m).padStart(2, '0') + ':' + String(sec).padStart(2, '0') + ':' + String(ff).padStart(2, '0');
}
function fmtTs(s) {                      // 00:00 (axis labels)
  const m = Math.floor(s / 60), sec = Math.round(s - m * 60);
  return String(m).padStart(2, '0') + ':' + String(sec).padStart(2, '0');
}
function fmtClock(ms) {
  if (!ms) return '--:--:--.-';
  const d = new Date(ms);
  return String(d.getHours()).padStart(2, '0') + ':' +
         String(d.getMinutes()).padStart(2, '0') + ':' +
         String(d.getSeconds()).padStart(2, '0') + '.' +
         Math.floor(d.getMilliseconds() / 100);
}
function fmtBytes(b) {
  if (b > 1e9) return (b / 1e9).toFixed(1) + ' GB';
  if (b > 1e6) return Math.round(b / 1e6) + ' MB';
  return Math.round(b / 1e3) + ' kB';
}

const EARTH_R = 6371000;
function hav(lat1, lon1, lat2, lon2) {  // distance in meters
  const toR = Math.PI / 180;
  const dLat = (lat2 - lat1) * toR, dLon = (lon2 - lon1) * toR;
  const a = Math.sin(dLat / 2) ** 2 +
    Math.cos(lat1 * toR) * Math.cos(lat2 * toR) * Math.sin(dLon / 2) ** 2;
  return 2 * EARTH_R * Math.asin(Math.sqrt(a));
}
function bearing(lat1, lon1, lat2, lon2) {
  const toR = Math.PI / 180;
  const y = Math.sin((lon2 - lon1) * toR) * Math.cos(lat2 * toR);
  const x = Math.cos(lat1 * toR) * Math.sin(lat2 * toR) -
            Math.sin(lat1 * toR) * Math.cos(lat2 * toR) * Math.cos((lon2 - lon1) * toR);
  return (Math.atan2(y, x) * 180 / Math.PI + 360) % 360;
}
const COMPASS = ['N','NNE','NE','ENE','E','ESE','SE','SSE','S','SSW','SW','WSW','W','WNW','NW','NNW'];
const compass = deg => COMPASS[Math.round(deg / 22.5) % 16];

// binary search: largest index with arr[i] <= v
function bisect(arr, v) {
  let lo = 0, hi = arr.length - 1;
  if (v <= arr[0]) return 0;
  if (v >= arr[hi]) return hi;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (arr[mid] <= v) lo = mid; else hi = mid;
  }
  return lo;
}

let toastTimer = null;
function toast(msg) {
  const el = $('toast');
  el.textContent = msg;
  el.classList.add('show');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => el.classList.remove('show'), 2600);
}

// ---------- state ----------
const S = {
  clips: [], clipIdx: -1, clip: null,
  tele: null,          // full per-frame telemetry
  smp: null,           // downsampled series for charts/map
  events: [],          // coaching cues
  markers: [],
  fps: 50,
  duration: 0,
  quality: 'proxy',
  follow: true,
  sat: false,
  estimate: false,
};

const video = $('video');
const CAT = {
  observation: { label: 'OBSERVATION', color: '#3987e5' },
  risk:        { label: 'RISK',        color: '#d03b3b' },
  camera:      { label: 'CAMERA',      color: '#c98500' },
  navigation:  { label: 'NAVIGATION',  color: '#9085e9' },
  poi:         { label: 'POI',         color: '#0ca30c' },
};
const SEV = {
  good:     { ic: '◆', cls: 'sev-good',     label: 'GOOD' },
  warn:     { ic: '▲', cls: 'sev-warn',     label: 'WATCH' },
  serious:  { ic: '▲', cls: 'sev-serious',  label: 'RISK' },
  critical: { ic: '⬤', cls: 'sev-critical', label: 'CRITICAL' },
};

// ============================================================
// SRT PARSER
// ============================================================
function parseSrt(text) {
  const blocks = text.split(/\r?\n\r?\n/);
  const n = blocks.length;
  const T = {
    t: new Float64Array(n), clock: new Float64Array(n),
    lat: new Float64Array(n), lon: new Float64Array(n),
    relAlt: new Float64Array(n), absAlt: new Float64Array(n),
    iso: new Float32Array(n), shutDen: new Float32Array(n),
    fnum: new Float32Array(n), ev: new Float32Array(n),
    focal: new Float32Array(n), ct: new Float32Array(n), tint: new Float32Array(n),
    colorMd: '', count: 0,
  };
  const reTime = /(\d{2}):(\d{2}):(\d{2}),(\d{3})\s*-->/;
  const reClock = /(\d{4})-(\d{2})-(\d{2})\s+(\d{2}):(\d{2}):(\d{2})[.,](\d+)/;
  const reKV = {
    iso: /\[iso\s*:\s*([\d.]+)/, shutter: /\[shutter\s*:\s*([\d/.]+)/,
    fnum: /\[fnum\s*:\s*([\d.]+)/, ev: /\[ev\s*:\s*([-\d.]+)/,
    focal: /\[focal_len\s*:\s*([\d.]+)/,
    lat: /\[latitude\s*:\s*([-\d.]+)/, lon: /\[longitude\s*:\s*([-\d.]+)/,
    alt: /\[rel_alt\s*:\s*([-\d.]+)\s+abs_alt\s*:\s*([-\d.]+)/,
    ct: /\[ct\s*:\s*(\d+)/, tint: /tint\s*:\s*([-\d]+)/,
    colorMd: /\[color_md\s*:\s*(\w+)/,
  };
  let k = 0;
  for (let i = 0; i < n; i++) {
    const b = blocks[i];
    const mt = reTime.exec(b);
    if (!mt) continue;
    T.t[k] = (+mt[1]) * 3600 + (+mt[2]) * 60 + (+mt[3]) + (+mt[4]) / 1000;
    const mc = reClock.exec(b);
    if (mc) {
      T.clock[k] = new Date(+mc[1], mc[2] - 1, +mc[3], +mc[4], +mc[5], +mc[6],
        Math.round(+('0.' + mc[7]) * 1000)).getTime();
    }
    let m;
    T.iso[k]  = (m = reKV.iso.exec(b))  ? +m[1] : NaN;
    if ((m = reKV.shutter.exec(b))) {
      const sh = m[1];
      T.shutDen[k] = sh.startsWith('1/') ? +sh.slice(2) : (+sh > 0 ? 1 / +sh : NaN);
    } else T.shutDen[k] = NaN;
    T.fnum[k]  = (m = reKV.fnum.exec(b))  ? +m[1] : NaN;
    T.ev[k]    = (m = reKV.ev.exec(b))    ? +m[1] : NaN;
    T.focal[k] = (m = reKV.focal.exec(b)) ? +m[1] : NaN;
    T.lat[k]   = (m = reKV.lat.exec(b))   ? +m[1] : NaN;
    T.lon[k]   = (m = reKV.lon.exec(b))   ? +m[1] : NaN;
    if ((m = reKV.alt.exec(b))) { T.relAlt[k] = +m[1]; T.absAlt[k] = +m[2]; }
    else { T.relAlt[k] = NaN; T.absAlt[k] = NaN; }
    T.ct[k]   = (m = reKV.ct.exec(b))   ? +m[1] : NaN;
    T.tint[k] = (m = reKV.tint.exec(b)) ? +m[1] : NaN;
    if (!T.colorMd && (m = reKV.colorMd.exec(b))) T.colorMd = m[1];
    k++;
  }
  T.count = k;
  // trim arrays to the number of valid blocks (the tail would be 0/NaN otherwise)
  for (const key of ['t', 'clock', 'lat', 'lon', 'relAlt', 'absAlt', 'iso',
                     'shutDen', 'fnum', 'ev', 'focal', 'ct', 'tint']) {
    T[key] = T[key].subarray(0, k);
  }
  return T;
}

// ============================================================
// DERIVED DATA — speed, heading, distance, samples
// ============================================================
function derive(T) {
  const n = T.count;
  const dur = n > 1 ? T.t[n - 1] : 0;
  const fps = n > 1 ? Math.max(1, Math.round((n - 1) / dur)) : 50;
  const w = Math.max(1, Math.round(fps / 2));      // ±0.5 s window

  T.speed = new Float32Array(n);    // m/s horizontal
  T.vspeed = new Float32Array(n);   // m/s vertical
  T.heading = new Float32Array(n);
  let lastHdg = 0;
  for (let i = 0; i < n; i++) {
    const a = Math.max(0, i - w), b = Math.min(n - 1, i + w);
    const dt = T.t[b] - T.t[a] || 1;
    const d = hav(T.lat[a], T.lon[a], T.lat[b], T.lon[b]);
    T.speed[i] = d / dt;
    T.vspeed[i] = (T.relAlt[b] - T.relAlt[a]) / dt;
    if (d > 0.5) lastHdg = bearing(T.lat[a], T.lon[a], T.lat[b], T.lon[b]);
    T.heading[i] = lastHdg;
  }

  // downsample for charts/map (~1400 points) + cumulative distance
  const stride = Math.max(1, Math.floor(n / 1400));
  const m = Math.ceil(n / stride);
  const smp = {
    stride, n: m,
    t: new Float64Array(m), alt: new Float32Array(m), absAlt: new Float32Array(m),
    kmh: new Float32Array(m), iso: new Float32Array(m), vs: new Float32Array(m),
    lat: new Float64Array(m), lon: new Float64Array(m), dist: new Float32Array(m),
    shutDen: new Float32Array(m),
  };
  let cum = 0;
  for (let j = 0; j < m; j++) {
    const i = Math.min(n - 1, j * stride);
    smp.t[j] = T.t[i];
    smp.alt[j] = T.relAlt[i];
    smp.absAlt[j] = T.absAlt[i];
    smp.kmh[j] = T.speed[i] * 3.6;
    smp.iso[j] = T.iso[i];
    smp.vs[j] = T.vspeed[i];
    smp.lat[j] = T.lat[i];
    smp.lon[j] = T.lon[i];
    smp.shutDen[j] = T.shutDen[i];
    if (j > 0) {
      const d = hav(smp.lat[j - 1], smp.lon[j - 1], smp.lat[j], smp.lon[j]);
      if (d > 0.15) cum += d;      // ignore GPS jitter
    }
    smp.dist[j] = cum;
  }

  // statistics
  let maxAlt = -1e9, maxKmh = 0, sumKmh = 0, maxIso = 0;
  for (let j = 0; j < m; j++) {
    if (smp.alt[j] > maxAlt) maxAlt = smp.alt[j];
    if (smp.kmh[j] > maxKmh) maxKmh = smp.kmh[j];
    if (smp.iso[j] > maxIso) maxIso = smp.iso[j];
    sumKmh += smp.kmh[j];
  }
  const stats = {
    dur, fps, maxAlt, maxKmh, avgKmh: sumKmh / m, dist: cum, maxIso,
    startClock: T.clock[0],
  };
  return { smp, stats, fps, dur };
}

// ============================================================
// COACHING CUES
// ============================================================
function detectEvents(smp) {
  const evts = [];
  // helper: contiguous segments where a predicate holds
  function segs(pred, minDur, gap = 1.5) {
    const out = [];
    let t0 = null, tLast = null;
    for (let j = 0; j < smp.n; j++) {
      if (pred(j)) {
        if (t0 === null) t0 = smp.t[j];
        tLast = smp.t[j];
      } else if (t0 !== null && smp.t[j] - tLast > gap) {
        if (tLast - t0 >= minDur) out.push([t0, tLast]);
        t0 = null;
      }
    }
    if (t0 !== null && tLast - t0 >= minDur) out.push([t0, tLast]);
    return out;
  }
  const idxAt = t => bisect(smp.t, t);
  const peak = (arr, a, b) => {
    let v = -Infinity;
    for (let j = idxAt(a); j <= idxAt(b); j++) if (arr[j] > v) v = arr[j];
    return v;
  };
  const push = (list, sev, title, descFn) => {
    for (const [a, b] of list) {
      evts.push({ t0: a, t1: b, sev, title, desc: descFn(a, b) });
    }
  };

  push(segs(j => smp.kmh[j] < 1.8 && smp.alt[j] > 3, 6), 'good',
    'Stable hover',
    (a, b) => `${Math.round(b - a)} s holding position at ${smp.alt[idxAt(a)].toFixed(0)} m — a usable inspection window.`);

  push(segs(j => smp.kmh[j] > 28 && smp.alt[j] < 15, 2), 'serious',
    'High speed close to the ground',
    (a, b) => `Up to ${Math.round(peak(smp.kmh, a, b))} km/h below 15 m — little time to react to obstacles.`);

  push(segs(j => Math.abs(smp.vs[j]) > 3, 2), 'warn',
    'Rapid climb or descent',
    (a, b) => `Vertical speed above 3 m/s for ${Math.round(b - a)} s — watch for vortex ring state when descending.`);

  push(segs(j => smp.iso[j] >= 800, 1.5), 'warn',
    'High ISO',
    () => `ISO 800 or above — expect visible noise. Consider a slower shutter or a different time of day.`);

  push(segs(j => smp.shutDen[j] >= 200 && smp.kmh[j] > 7, 4), 'warn',
    'Shutter speed vs. the 180° rule',
    () => `Shutter faster than about 1/100 at 50 fps while moving — motion may look stuttery for cinematic work.`);

  push(segs(j => smp.alt[j] > 118, 1), 'critical',
    'Approaching the 120 m limit',
    () => `Altitude is approaching the 120 m legal ceiling (EU open category).`);

  evts.sort((x, y) => x.t0 - y.t0);
  return evts;
}

// ============================================================
// CHARTS
// ============================================================
const tipEl = document.createElement('div');
tipEl.className = 'viz-tip hidden';
document.body.appendChild(tipEl);

class Chart {
  constructor(canvas, cfg) {
    this.cv = canvas;
    this.cfg = cfg;              // {times, values, color, fill, unit, fmt}
    this.hoverT = null;
    this.static = null;
    this.pad = { l: 44, r: 10, t: 8, b: 18 };
    this.bindEvents();
    this.layout();
  }
  layout() {
    const dpr = window.devicePixelRatio || 1;
    const rect = this.cv.getBoundingClientRect();
    this.W = Math.max(80, rect.width); this.H = Math.max(60, rect.height);
    this.cv.width = this.W * dpr; this.cv.height = this.H * dpr;
    this.dpr = dpr;
    const { times, values } = this.cfg;
    this.t0 = times[0]; this.t1 = times[times.length - 1] || 1;
    let vmin = Infinity, vmax = -Infinity;
    for (let i = 0; i < values.length; i++) {
      const v = values[i];
      if (isFinite(v)) { if (v < vmin) vmin = v; if (v > vmax) vmax = v; }
    }
    if (!isFinite(vmin)) { vmin = 0; vmax = 1; }
    if (vmax - vmin < 1e-6) { vmax += 1; vmin -= 1; }
    const span = vmax - vmin;
    this.vmin = this.cfg.zeroBase ? Math.min(0, vmin) : vmin - span * 0.08;
    this.vmax = vmax + span * 0.12;
    this.renderStatic();
  }
  x(t) { return this.pad.l + (t - this.t0) / (this.t1 - this.t0) * (this.W - this.pad.l - this.pad.r); }
  y(v) { return this.pad.t + (1 - (v - this.vmin) / (this.vmax - this.vmin)) * (this.H - this.pad.t - this.pad.b); }
  xToT(px) { return clamp(this.t0 + (px - this.pad.l) / (this.W - this.pad.l - this.pad.r) * (this.t1 - this.t0), this.t0, this.t1); }
  valueAt(t) { return this.cfg.values[bisect(this.cfg.times, t)]; }

  renderStatic() {
    const off = document.createElement('canvas');
    off.width = this.cv.width; off.height = this.cv.height;
    const c = off.getContext('2d');
    c.scale(this.dpr, this.dpr);
    const css = getComputedStyle(document.documentElement);
    const grid = css.getPropertyValue('--grid').trim() || '#262b26';
    const muted = css.getPropertyValue('--muted').trim() || '#86897f';

    c.font = '10px "JetBrains Mono", monospace';
    // y gridlines + labels
    const ticks = 3;
    c.textAlign = 'right'; c.textBaseline = 'middle';
    for (let i = 0; i <= ticks; i++) {
      const v = this.vmin + (this.vmax - this.vmin) * i / ticks;
      const yy = this.y(v);
      c.strokeStyle = grid; c.lineWidth = 1;
      c.beginPath(); c.moveTo(this.pad.l, yy); c.lineTo(this.W - this.pad.r, yy); c.stroke();
      c.fillStyle = muted;
      c.fillText(this.cfg.fmt(v), this.pad.l - 6, yy);
    }
    // x labels
    c.textAlign = 'center'; c.textBaseline = 'top';
    const span = this.t1 - this.t0;
    const step = span > 900 ? 240 : span > 420 ? 120 : span > 150 ? 60 : span > 60 ? 30 : span > 20 ? 10 : 5;
    for (let tt = Math.ceil(this.t0 / step) * step; tt <= this.t1; tt += step) {
      c.fillStyle = muted;
      c.fillText(fmtTs(tt), this.x(tt), this.H - this.pad.b + 4);
    }
    // series
    const { times, values, color } = this.cfg;
    c.beginPath();
    let started = false;
    for (let i = 0; i < times.length; i++) {
      const v = values[i];
      if (!isFinite(v)) continue;
      const xx = this.x(times[i]), yy = this.y(v);
      if (!started) { c.moveTo(xx, yy); started = true; } else c.lineTo(xx, yy);
    }
    if (this.cfg.fill && started) {
      const base = this.y(Math.max(this.vmin, 0));
      c.save();
      c.lineTo(this.x(times[times.length - 1]), base);
      c.lineTo(this.x(times[0]), base);
      c.closePath();
      const g = c.createLinearGradient(0, this.pad.t, 0, base);
      g.addColorStop(0, color + '55'); g.addColorStop(1, color + '08');
      c.fillStyle = g; c.fill();
      c.restore();
      // redraw the line (filling closed the path)
      c.beginPath(); started = false;
      for (let i = 0; i < times.length; i++) {
        const v = values[i];
        if (!isFinite(v)) continue;
        const xx = this.x(times[i]), yy = this.y(v);
        if (!started) { c.moveTo(xx, yy); started = true; } else c.lineTo(xx, yy);
      }
    }
    c.strokeStyle = color; c.lineWidth = 2; c.lineJoin = 'round'; c.stroke();
    this.static = off;
  }

  draw(tNow) {
    const c = this.cv.getContext('2d');
    c.setTransform(1, 0, 0, 1, 0, 0);
    c.clearRect(0, 0, this.cv.width, this.cv.height);
    if (this.static) c.drawImage(this.static, 0, 0);
    c.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);
    // playhead
    const px = this.x(clamp(tNow, this.t0, this.t1));
    c.strokeStyle = '#ffb000'; c.lineWidth = 1.5;
    c.beginPath(); c.moveTo(px, this.pad.t); c.lineTo(px, this.H - this.pad.b); c.stroke();
    c.fillStyle = '#ffb000';
    c.beginPath(); c.arc(px, this.y(this.valueAt(tNow)), 3.5, 0, 7); c.fill();
    // hover
    if (this.hoverT !== null) {
      const hx = this.x(this.hoverT);
      c.strokeStyle = 'rgba(255,255,255,.25)'; c.lineWidth = 1;
      c.setLineDash([3, 3]);
      c.beginPath(); c.moveTo(hx, this.pad.t); c.lineTo(hx, this.H - this.pad.b); c.stroke();
      c.setLineDash([]);
    }
  }

  bindEvents() {
    const cv = this.cv;
    let down = false;
    const toT = e => this.xToT(e.clientX - cv.getBoundingClientRect().left);
    cv.addEventListener('pointerdown', e => {
      down = true; cv.setPointerCapture(e.pointerId);
      seekTo(toT(e));
    });
    cv.addEventListener('pointermove', e => {
      const t = toT(e);
      this.hoverT = t;
      if (down) seekTo(t);
      tipEl.classList.remove('hidden');
      tipEl.innerHTML = `<span class="tt-label">${fmtT(t)}</span> &nbsp;${this.cfg.fmt(this.valueAt(t))} ${this.cfg.unit}`;
      tipEl.style.left = (e.clientX + 14) + 'px';
      tipEl.style.top = (e.clientY - 14) + 'px';
    });
    cv.addEventListener('pointerup', () => { down = false; });
    cv.addEventListener('pointerleave', () => {
      down = false; this.hoverT = null; tipEl.classList.add('hidden');
    });
  }
}

let charts = [];
function buildCharts() {
  const { smp } = S;
  const mk = (id, values, color, unit, fmt, fill, zeroBase) =>
    new Chart($(id), { times: smp.t, values, color, unit, fmt, fill, zeroBase });
  charts = [
    mk('chAlt',   smp.alt, '#3987e5', 'm',    v => v.toFixed(0), true,  true),
    mk('chSpeed', smp.kmh, '#d95926', 'km/h', v => v.toFixed(0), true,  true),
    mk('chIso',   smp.iso, '#c98500', 'ISO',  v => v.toFixed(0), false, false),
  ];
}

// ============================================================
// TIMELINE
// ============================================================
const tl = $('timeline');
let tlStatic = null;

function renderTimelineStatic() {
  if (!S.smp) return;
  const dpr = window.devicePixelRatio || 1;
  const rect = tl.getBoundingClientRect();
  tl.width = rect.width * dpr; tl.height = 72 * dpr;
  const off = document.createElement('canvas');
  off.width = tl.width; off.height = tl.height;
  const c = off.getContext('2d');
  c.scale(dpr, dpr);
  const W = rect.width, H = 72;
  const { smp } = S;
  const x = t => t / S.duration * W;

  const drawRuler = () => {
    c.font = '9px "JetBrains Mono", monospace';
    c.fillStyle = '#86897f'; c.textAlign = 'left';
    const step = S.duration > 900 ? 120 : S.duration > 300 ? 60 : S.duration > 100 ? 30 : 10;
    for (let tt = 0; tt <= S.duration; tt += step) {
      c.fillRect(x(tt), H - 10, 1, 4);
      c.fillText(fmtTs(tt), x(tt) + 3, H - 2);
    }
  };

  if (S.estimate) {
    // estimate mode: ruler and marker ticks only — the profile would give the answer away
    drawRuler();
    for (const mk of S.markers) {
      const mx = x(mk.t);
      c.strokeStyle = CAT[mk.cat]?.color || '#ffb000';
      c.lineWidth = 1.5;
      c.beginPath(); c.moveTo(mx, 4); c.lineTo(mx, H - 20); c.stroke();
    }
    tlStatic = off;
    return;
  }

  // altitude sparkline
  let maxAlt = 1;
  for (let j = 0; j < smp.n; j++) if (smp.alt[j] > maxAlt) maxAlt = smp.alt[j];
  const yAlt = v => 8 + (1 - v / maxAlt) * (H - 30);
  c.beginPath();
  c.moveTo(x(smp.t[0]), yAlt(Math.max(0, smp.alt[0])));
  for (let j = 1; j < smp.n; j++) c.lineTo(x(smp.t[j]), yAlt(Math.max(0, smp.alt[j])));
  c.lineTo(x(smp.t[smp.n - 1]), H - 22); c.lineTo(x(smp.t[0]), H - 22); c.closePath();
  c.fillStyle = 'rgba(57,135,229,.20)'; c.fill();
  c.strokeStyle = 'rgba(57,135,229,.75)'; c.lineWidth = 1.5;
  c.beginPath();
  c.moveTo(x(smp.t[0]), yAlt(Math.max(0, smp.alt[0])));
  for (let j = 1; j < smp.n; j++) c.lineTo(x(smp.t[j]), yAlt(Math.max(0, smp.alt[j])));
  c.stroke();

  // coaching cue bands
  const sevColor = { good: '#0ca30c', warn: '#fab219', serious: '#ec835a', critical: '#d03b3b' };
  for (const ev of S.events) {
    c.fillStyle = sevColor[ev.sev] + 'cc';
    c.fillRect(x(ev.t0), H - 18, Math.max(2, x(ev.t1) - x(ev.t0)), 5);
  }

  drawRuler();

  // markers
  for (const mk of S.markers) {
    const mx = x(mk.t);
    c.strokeStyle = CAT[mk.cat]?.color || '#ffb000';
    c.lineWidth = 1.5;
    c.beginPath(); c.moveTo(mx, 4); c.lineTo(mx, H - 20); c.stroke();
    c.fillStyle = CAT[mk.cat]?.color || '#ffb000';
    c.beginPath(); c.moveTo(mx, 4); c.lineTo(mx + 7, 8); c.lineTo(mx, 12); c.closePath(); c.fill();
  }
  tlStatic = off;
}

function drawTimeline(tNow) {
  if (!tlStatic) return;
  const c = tl.getContext('2d');
  c.setTransform(1, 0, 0, 1, 0, 0);
  c.clearRect(0, 0, tl.width, tl.height);
  c.drawImage(tlStatic, 0, 0);
  const dpr = window.devicePixelRatio || 1;
  c.setTransform(dpr, 0, 0, dpr, 0, 0);
  const W = tl.width / dpr, H = 72;
  const px = tNow / S.duration * W;
  c.fillStyle = 'rgba(255,176,0,.14)';
  c.fillRect(0, 0, px, H);
  c.strokeStyle = '#ffb000'; c.lineWidth = 2;
  c.beginPath(); c.moveTo(px, 0); c.lineTo(px, H); c.stroke();
}

(function bindTimeline() {
  let down = false;
  const toT = e => clamp((e.clientX - tl.getBoundingClientRect().left) / tl.getBoundingClientRect().width, 0, 1) * S.duration;
  tl.addEventListener('pointerdown', e => { down = true; tl.setPointerCapture(e.pointerId); seekTo(toT(e)); });
  tl.addEventListener('pointermove', e => { if (down) seekTo(toT(e)); });
  tl.addEventListener('pointerup', () => { down = false; });
})();

// ============================================================
// MAP
// ============================================================
let map = null, trackGroup = null, droneMarker = null, droneEl = null;
let layerDark = null, layerSat = null;

function initMap() {
  if (!window.L) {
    $('map').innerHTML = '<div class="map-fallback">Map unavailable (Leaflet needs an internet connection).<br>Telemetry and charts still work.</div>';
    return;
  }
  map = L.map('map', { zoomControl: true });
  layerDark = L.tileLayer('https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png', {
    maxZoom: 20, attribution: '&copy; OpenStreetMap &copy; CARTO',
  });
  layerSat = L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', {
    maxZoom: 19, attribution: 'Esri World Imagery',
  });
  layerDark.addTo(map);
  trackGroup = L.layerGroup().addTo(map);
}

function buildTrack() {
  if (!map) return;
  trackGroup.clearLayers();
  const { smp } = S;
  const pts = [];
  for (let j = 0; j < smp.n; j++) {
    if (isFinite(smp.lat[j]) && Math.abs(smp.lat[j]) > 0.01) pts.push([smp.lat[j], smp.lon[j], smp.t[j]]);
  }
  if (!pts.length) return;

  const latlngs = pts.map(p => [p[0], p[1]]);
  L.polyline(latlngs, { color: '#0c0e0c', weight: 6, opacity: .55 }).addTo(trackGroup);
  const line = L.polyline(latlngs, { color: '#ffb000', weight: 2.5, opacity: .95 }).addTo(trackGroup);
  line.on('click', e => {
    // jump to the nearest point on the track
    let best = 0, bd = Infinity;
    for (let i = 0; i < pts.length; i++) {
      const d = (pts[i][0] - e.latlng.lat) ** 2 + (pts[i][1] - e.latlng.lng) ** 2;
      if (d < bd) { bd = d; best = i; }
    }
    seekTo(pts[best][2]);
  });

  L.circleMarker(latlngs[0], {
    radius: 7, color: '#0ca30c', weight: 2, fillColor: '#0ca30c', fillOpacity: .4,
  }).bindTooltip('Takeoff / home', { direction: 'top' }).addTo(trackGroup);

  const icon = L.divIcon({
    className: 'drone-icon',
    iconSize: [30, 30], iconAnchor: [15, 15],
    html: `<div style="width:30px;height:30px;transform:rotate(0deg)">
      <svg viewBox="0 0 30 30" width="30" height="30">
        <polygon points="15,3 24,25 15,19 6,25" fill="#ffb000" stroke="#141005" stroke-width="1.4"/>
      </svg></div>`,
  });
  droneMarker = L.marker(latlngs[0], { icon, interactive: false, zIndexOffset: 1000 }).addTo(trackGroup);
  droneEl = null; // fetched lazily — the marker has to be in the DOM first

  map.fitBounds(line.getBounds(), { padding: [28, 28] });
}

let lastMapPan = 0;
function updateMapMarker(i) {
  if (!map || !droneMarker || !S.tele) return;
  const T = S.tele;
  if (!isFinite(T.lat[i]) || Math.abs(T.lat[i]) < 0.01) return;
  const ll = [T.lat[i], T.lon[i]];
  droneMarker.setLatLng(ll);
  if (!droneEl) droneEl = droneMarker.getElement()?.firstElementChild;
  if (droneEl) droneEl.style.transform = `rotate(${T.heading[i]}deg)`;
  if (S.follow && !video.paused && performance.now() - lastMapPan > 400) {
    lastMapPan = performance.now();
    if (!map.getBounds().pad(-0.25).contains(ll)) map.panTo(ll, { animate: true, duration: .4 });
  }
}

// ============================================================
// HUD
// ============================================================
const hudEls = {
  alt: $('tAlt'), speed: $('tSpeed'), absAlt: $('tAbsAlt'), vspeed: $('tVSpeed'),
  heading: $('tHeading'), headingArrow: $('tHeadingArrow'), dist: $('tDist'),
  iso: $('tIso'), shutter: $('tShutter'), fnum: $('tFnum'), ev: $('tEv'),
  ct: $('tCt'), color: $('tColor'), gps: $('tGps'),
  clock: $('tClock'), frameNo: $('tFrameNo'),
};
let prevIso = null, prevShut = null;

function flash(el) {
  el.classList.add('flash');
  setTimeout(() => el.classList.remove('flash'), 350);
}

function updateHud(i, tNow) {
  const T = S.tele, smp = S.smp;
  hudEls.alt.textContent = T.relAlt[i].toFixed(1);
  hudEls.speed.textContent = (T.speed[i] * 3.6).toFixed(1);
  hudEls.absAlt.textContent = T.absAlt[i].toFixed(1);
  const vs = T.vspeed[i];
  hudEls.vspeed.textContent = (vs > 0.2 ? '↑ ' : vs < -0.2 ? '↓ ' : '') + Math.abs(vs).toFixed(1);
  hudEls.heading.textContent = Math.round(T.heading[i]) + '° ' + compass(T.heading[i]);
  hudEls.headingArrow.style.display = 'inline-block';
  hudEls.headingArrow.style.transform = `rotate(${T.heading[i]}deg)`;
  hudEls.headingArrow.textContent = '↑';
  const j = bisect(smp.t, tNow);
  hudEls.dist.textContent = smp.dist[j].toFixed(0);

  const iso = T.iso[i];
  hudEls.iso.textContent = isFinite(iso) ? iso.toFixed(0) : '—';
  if (prevIso !== null && iso !== prevIso) flash(hudEls.iso);
  prevIso = iso;
  const sd = T.shutDen[i];
  hudEls.shutter.textContent = isFinite(sd) ? '1/' + Math.round(sd) : '—';
  if (prevShut !== null && sd !== prevShut) flash(hudEls.shutter);
  prevShut = sd;
  hudEls.fnum.textContent = isFinite(T.fnum[i]) ? 'f/' + T.fnum[i].toFixed(1) : '—';
  hudEls.ev.textContent = (T.ev[i] > 0 ? '+' : '') + T.ev[i].toFixed(1);
  hudEls.ct.textContent = isFinite(T.ct[i]) ? T.ct[i].toFixed(0) : '—';
  hudEls.color.textContent = (S.tele.colorMd || '—').toUpperCase() + ' · ' + T.focal[i].toFixed(0) + 'mm';
  hudEls.gps.textContent = T.lat[i].toFixed(6) + ', ' + T.lon[i].toFixed(6);

  hudEls.clock.textContent = fmtClock(T.clock[i]);
  hudEls.frameNo.textContent = (i + 1).toLocaleString('en-US');
  $('hudFrame').textContent = `${T.count.toLocaleString('en-US')} frames · ${S.fps} fps`;
}

// ============================================================
// VIDEO & TRANSPORT
// ============================================================
function seekTo(t) {
  if (!S.duration) return;
  video.currentTime = clamp(t, 0, Math.max(0, S.duration - 0.02));
}

function setPlaying(playing) {
  if (playing) video.play().catch(() => {});
  else video.pause();
}
video.addEventListener('play', () => { $('btnPlay').textContent = '❚❚'; $('bigPlay').classList.add('hidden'); });
video.addEventListener('pause', () => { $('btnPlay').textContent = '▶'; $('bigPlay').classList.remove('hidden'); });
video.addEventListener('error', () => {
  if (S.quality === 'full') {
    toast('This browser cannot play the 4K file — switching back to the proxy.');
    setQuality('proxy');
  }
});

$('btnPlay').onclick = () => setPlaying(video.paused);
$('bigPlay').onclick = () => setPlaying(true);
$('videoStage').addEventListener('click', e => {
  if (e.target === video) setPlaying(video.paused);
});
$('btnPrevFrame').onclick = () => { video.pause(); seekTo(video.currentTime - 1 / S.fps); };
$('btnNextFrame').onclick = () => { video.pause(); seekTo(video.currentTime + 1 / S.fps); };

// scrolling over the video or timeline scrubs frame by frame (the finest step)
let wheelAcc = 0;
function wheelScrub(e) {
  e.preventDefault();
  wheelAcc += (Math.abs(e.deltaY) >= Math.abs(e.deltaX) ? e.deltaY : e.deltaX);
  const notch = 28;                      // sensitivity: ±1 frame per notch
  while (Math.abs(wheelAcc) >= notch) {
    const dir = Math.sign(wheelAcc);
    wheelAcc -= dir * notch;
    video.pause();
    seekTo(video.currentTime + dir / S.fps);
  }
}
$('videoStage').addEventListener('wheel', wheelScrub, { passive: false });
tl.addEventListener('wheel', wheelScrub, { passive: false });

document.querySelectorAll('.spd').forEach(b => {
  b.onclick = () => {
    document.querySelectorAll('.spd').forEach(x => x.classList.toggle('active', x === b));
    video.playbackRate = +b.dataset.spd;
  };
});

function setQuality(q, resetTime = false) {
  const clip = S.clip;
  if (!clip) return;
  if (q === 'proxy' && !clip.lrf) q = 'full';
  if (q === 'full' && !clip.mp4) q = 'proxy';
  S.quality = q;
  const t = resetTime ? 0 : video.currentTime;
  const wasPlaying = !resetTime && !video.paused;
  const rate = video.playbackRate;
  video.src = q === 'proxy' ? clip.lrf : clip.mp4;
  video.load();
  video.addEventListener('loadedmetadata', function once() {
    video.removeEventListener('loadedmetadata', once);
    video.currentTime = t;
    video.playbackRate = rate;
    if (wasPlaying) video.play().catch(() => {});
  });
  $('btnProxy').classList.toggle('active', q === 'proxy');
  $('btnFull').classList.toggle('active', q === 'full');
}
$('btnProxy').onclick = () => setQuality('proxy');
$('btnFull').onclick = () => setQuality('full');

// ============================================================
// MARKERS
// ============================================================
const storeKey = () => 'gs-markers-' + (S.clip?.id || 'x');
function loadMarkers() {
  try { S.markers = JSON.parse(localStorage.getItem(storeKey()) || '[]'); }
  catch { S.markers = []; }
}
function saveMarkers() {
  localStorage.setItem(storeKey(), JSON.stringify(S.markers));
  renderMarkers(); renderTimelineStatic();
}
function telemetryAt(t) {
  const T = S.tele;
  if (!T) return {};
  const i = bisect(T.t, t);
  return {
    lat: +T.lat[i].toFixed(6), lon: +T.lon[i].toFixed(6),
    altitude_m: +T.relAlt[i].toFixed(1), speed_kmh: +(T.speed[i] * 3.6).toFixed(1),
    iso: T.iso[i], clock: fmtClock(T.clock[i]),
  };
}
function addMarker(cat, note) {
  const t = video.currentTime;
  S.markers.push({ id: Date.now() + '' + Math.floor(Math.random() * 1e4), t, cat, note });
  S.markers.sort((a, b) => a.t - b.t);
  saveMarkers();
  toast(`⚑ Marker dropped at ${fmtT(t)}`);
}
$('markerForm').addEventListener('submit', e => {
  e.preventDefault();
  addMarker($('markerCat').value, $('markerNote').value.trim());
  $('markerNote').value = '';
});
$('btnMarker').onclick = () => {
  document.querySelector('[data-tab="markers"]').click();
  addMarker($('markerCat').value, $('markerNote').value.trim());
  $('markerNote').value = '';
};

function renderMarkers() {
  const ul = $('markerList');
  ul.innerHTML = '';
  for (const mk of S.markers) {
    const cat = CAT[mk.cat] || CAT.observation;
    const li = document.createElement('li');
    li.className = 'log-item';
    li.innerHTML =
      `<span class="li-time">${fmtT(mk.t)}</span>` +
      `<span class="li-sev" style="color:${cat.color}"><span class="sev-ic">⚑</span>${cat.label}</span>` +
      `<span class="li-text">${mk.note ? escapeHtml(mk.note) : '<i>no note</i>'}</span>` +
      `<button class="li-del" title="Delete">✕</button>`;
    li.onclick = e => {
      if (e.target.classList.contains('li-del')) {
        S.markers = S.markers.filter(m => m.id !== mk.id);
        saveMarkers();
      } else seekTo(mk.t);
    };
    ul.appendChild(li);
  }
  $('markerCount').textContent = S.markers.length;
}
function escapeHtml(s) {
  return s.replace(/[&<>"']/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
}

function download(name, mime, text) {
  const a = document.createElement('a');
  a.href = URL.createObjectURL(new Blob([text], { type: mime }));
  a.download = name;
  a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 5000);
}
$('btnExportJson').onclick = () => {
  const data = {
    clip: S.clip.id, exported: new Date().toISOString(),
    markers: S.markers.map(m => ({ ...m, timecode: fmtT(m.t), telemetry: telemetryAt(m.t) })),
  };
  download(S.clip.id + '_markers.json', 'application/json', JSON.stringify(data, null, 2));
};
$('btnExportCsv').onclick = () => {
  const rows = [['time_s', 'timecode', 'clock', 'category', 'note', 'lat', 'lon', 'altitude_m', 'speed_kmh']];
  for (const m of S.markers) {
    const tel = telemetryAt(m.t);
    rows.push([m.t.toFixed(2), fmtT(m.t), tel.clock, m.cat, (m.note || '').replace(/;/g, ','),
               tel.lat, tel.lon, tel.altitude_m, tel.speed_kmh]);
  }
  download(S.clip.id + '_markers.csv', 'text/csv', rows.map(r => r.join(';')).join('\n'));
};
$('importFile').addEventListener('change', async e => {
  const f = e.target.files[0];
  if (!f) return;
  try {
    const data = JSON.parse(await f.text());
    const arr = Array.isArray(data) ? data : data.markers;
    let added = 0;
    for (const m of arr || []) {
      if (typeof m.t === 'number' && !S.markers.some(x => x.id === m.id)) {
        S.markers.push({ id: m.id || Date.now() + '' + added, t: m.t, cat: m.cat || 'observation', note: m.note || '' });
        added++;
      }
    }
    S.markers.sort((a, b) => a.t - b.t);
    saveMarkers();
    toast(`${added} marker(s) imported`);
  } catch { toast('Could not read that file as marker JSON.'); }
  e.target.value = '';
});

// ============================================================
// CUE LIST & TABS
// ============================================================
function renderEvents() {
  const ul = $('coachList');
  ul.innerHTML = '';
  for (const ev of S.events) {
    const sev = SEV[ev.sev];
    const li = document.createElement('li');
    li.className = 'log-item';
    li.innerHTML =
      `<span class="li-time">${fmtT(ev.t0)}–${fmtT(ev.t1)}</span>` +
      `<span class="li-sev ${sev.cls}"><span class="sev-ic">${sev.ic}</span>${sev.label}</span>` +
      `<span class="li-text"><b>${ev.title}</b> — ${ev.desc}</span>`;
    li.onclick = () => seekTo(ev.t0);
    ul.appendChild(li);
  }
  $('coachCount').textContent = S.events.length;
}

document.querySelectorAll('.tab').forEach(tab => {
  tab.onclick = () => {
    document.querySelectorAll('.tab').forEach(t => t.classList.toggle('active', t === tab));
    $('tabCoach').classList.toggle('hidden', tab.dataset.tab !== 'coach');
    $('tabMarkers').classList.toggle('hidden', tab.dataset.tab !== 'markers');
  };
});

// ============================================================
// STATISTICS
// ============================================================
function renderStats(stats) {
  const items = [
    [fmtTs(stats.dur), 'duration'],
    [(stats.dist >= 1000 ? (stats.dist / 1000).toFixed(2) + ' km' : Math.round(stats.dist) + ' m'), 'distance'],
    [stats.maxAlt.toFixed(0) + ' m', 'max altitude'],
    [stats.maxKmh.toFixed(0) + ' km/h', 'max speed'],
    [stats.avgKmh.toFixed(0) + ' km/h', 'avg speed'],
    [String(Math.round(stats.maxIso)), 'max iso'],
  ];
  $('statsStrip').innerHTML = items.map(([v, l]) =>
    `<div class="stat"><b>${v}</b><span>${l}</span></div>`).join('');
}

// ============================================================
// ESTIMATE MODE
// ============================================================
function setEstimate(on) {
  S.estimate = on;
  document.body.classList.toggle('estimate', on);
  $('btnEstimate').classList.toggle('active', on);
  renderTimelineStatic();
  toast(on ? 'Estimate mode ON — telemetry hidden. Let the trainee call out altitude and speed.'
           : 'Estimate mode OFF — telemetry visible again.');
}
$('btnEstimate').onclick = () => setEstimate(!S.estimate);

// ============================================================
// KEYBOARD
// ============================================================
document.addEventListener('keydown', e => {
  if (['INPUT', 'SELECT', 'TEXTAREA'].includes(e.target.tagName)) return;
  const spds = { Digit1: 0.25, Digit2: 0.5, Digit3: 1, Digit4: 2 };
  if (e.code === 'Space') { e.preventDefault(); setPlaying(video.paused); }
  else if (e.code === 'ArrowLeft') { e.preventDefault(); video.pause(); seekTo(video.currentTime - (e.shiftKey ? 1 : 1 / S.fps)); }
  else if (e.code === 'ArrowRight') { e.preventDefault(); video.pause(); seekTo(video.currentTime + (e.shiftKey ? 1 : 1 / S.fps)); }
  else if (e.code === 'KeyM') { $('btnMarker').click(); }
  else if (e.code === 'KeyT') { setEstimate(!S.estimate); }
  else if (spds[e.code]) {
    document.querySelector(`.spd[data-spd="${spds[e.code]}"]`)?.click();
  }
});

// ============================================================
// MAP CONTROLS & GPS COPY
// ============================================================
$('btnFollow').onclick = () => {
  S.follow = !S.follow;
  $('btnFollow').classList.toggle('active', S.follow);
};
$('btnLayer').onclick = () => {
  if (!map) return;
  S.sat = !S.sat;
  if (S.sat) { map.removeLayer(layerDark); layerSat.addTo(map); $('btnLayer').textContent = 'DARK'; }
  else { map.removeLayer(layerSat); layerDark.addTo(map); $('btnLayer').textContent = 'SAT'; }
};
$('btnCopyGps').onclick = () => {
  navigator.clipboard?.writeText(hudEls.gps.textContent)
    .then(() => toast('GPS coordinates copied: ' + hudEls.gps.textContent));
};

// ============================================================
// CLIP LOADING
// ============================================================
async function loadClip(idx) {
  if (idx === S.clipIdx) return;
  const clip = S.clips[idx];
  S.clipIdx = idx; S.clip = clip;
  const loader = $('loader');
  loader.classList.remove('done');
  $('loaderMsg').textContent = 'Loading telemetry… (' + clip.id + ')';

  document.querySelectorAll('.clip-tab').forEach((el, i) =>
    el.classList.toggle('active', i === idx));
  $('tcClip').textContent = 'CLIP ' + clip.num;
  $('footClip').textContent = clip.id + ' · ' + (clip.lrf ? 'proxy ' + fmtBytes(clip.lrfSize) + ' · ' : '') + 'full ' + fmtBytes(clip.mp4Size);

  // fetch + parse the SRT
  const text = await (await fetch(clip.srt)).text();
  $('loaderMsg').textContent = 'Processing telemetry…';
  await new Promise(r => setTimeout(r, 20));   // give the loader a chance to paint
  const T = parseSrt(text);
  const { smp, stats, fps, dur } = derive(T);
  S.tele = T; S.smp = smp; S.fps = fps; S.duration = dur;
  S.events = detectEvents(smp);
  prevIso = prevShut = null;

  // video (jump back to the start when switching clips)
  setQuality(clip.lrf ? 'proxy' : 'full', true);

  // build the UI
  loadMarkers();
  buildCharts();
  renderTimelineStatic();
  renderEvents();
  renderMarkers();
  renderStats(stats);
  buildTrack();

  loader.classList.add('done');
  toast(`Clip ${clip.num} loaded — ${T.count.toLocaleString('en-US')} telemetry frames @ ${fps} fps`);
}

function buildTabs() {
  const nav = $('clipTabs');
  nav.innerHTML = '';
  S.clips.forEach((c, i) => {
    const b = document.createElement('button');
    b.className = 'clip-tab';
    b.innerHTML = `<span class="ct-num">CLIP ${c.num}</span><span class="ct-meta">${c.time || ''} · ${fmtBytes(c.mp4Size)}</span>`;
    b.onclick = () => loadClip(i);
    nav.appendChild(b);
  });
}

// ============================================================
// MAIN LOOP
// ============================================================
let lastT = -1;
function tick() {
  requestAnimationFrame(tick);
  if (!S.tele || !S.duration) return;
  const t = video.currentTime;
  $('timeReadout').textContent = fmtTc(t) + ' / ' + fmtTc(S.duration);
  if (t !== lastT) {
    lastT = t;
    const i = bisect(S.tele.t, t);
    updateHud(i, t);
    updateMapMarker(i);
    drawTimeline(t);
  }
  for (const ch of charts) ch.draw(t);
}

window.addEventListener('resize', () => {
  for (const ch of charts) ch.layout();
  renderTimelineStatic();
});

// ============================================================
// BOOT
// ============================================================
(async function init() {
  if (location.protocol === 'file:') {
    $('loaderMsg').innerHTML =
      'This page was opened as a plain file, so it cannot reach the video files.<br>' +
      'Run <b>python3 server.py</b> in the project folder (or double-click the start script)<br>' +
      'and open <b>http://localhost:8765</b>.';
    return;
  }

  // reach the server first; if that fails, explain and keep retrying
  let data = null;
  try {
    const res = await fetch('api/clips', { cache: 'no-store' });
    data = await res.json();
  } catch {
    $('loaderMsg').innerHTML =
      'Cannot reach the GROUNDSTATION server.<br>' +
      'Start it with <b>python3 server.py</b> in the project folder.<br>' +
      '<span class="retry-note">This page continues on its own once the server is up…</span>';
    const timer = setInterval(async () => {
      try {
        const r = await fetch('api/clips', { cache: 'no-store' });
        if (r.ok) { clearInterval(timer); location.reload(); }
      } catch { /* server not up yet */ }
    }, 2000);
    return;
  }

  try {
    S.clips = data.clips;
    if (!S.clips.length) {
      $('loaderMsg').innerHTML =
        'No clips found in the media folder:<br><b>' + escapeHtml(data.mediaDir || '?') + '</b><br>' +
        'Put the drone MP4/LRF/SRT files there and reload this page.';
      return;
    }
    buildTabs();
    initMap();
    tick();
    await loadClip(0);
  } catch (err) {
    $('loaderMsg').textContent = 'Loading failed: ' + err.message;
  }
})();
