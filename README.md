# GROUNDSTATION

**Drone flight analysis for pilot training.** GROUNDSTATION matches the
telemetry DJI drones write into an `.SRT` sidecar file to the video itself,
frame by frame, and turns it into a debrief tool for training inspectors,
camera operators and emergency-response pilots.

No dependencies: **Python 3** (standard library only) and a modern browser.
Everything stays on your own machine — only the map tiles and web fonts are
loaded from the internet.

## Quick start

```bash
python3 server.py
```

Then open <http://localhost:8765>. On macOS you can also double-click
**`Start GROUNDSTATION.command`**, which starts the server and opens the
browser for you.

## Where the video goes

Each flight consists of the files the drone writes to its card: `*.MP4`
(the 4K original), `*.LRF` (the 720p proxy, optional but recommended) and
`*.SRT` (the telemetry). GROUNDSTATION looks for them in this order:

1. an explicit path: `python3 server.py 8765 /path/to/media`
2. the `GROUNDSTATION_MEDIA` environment variable
3. a `media_path.txt` file next to `server.py`, containing one line with a path
4. `./media` — created on first run if nothing else is found

Telemetry only ends up in the `.SRT` if the drone is set to record it: look for
the **"Video Subtitles"** setting. Without an `.LRF` proxy the tool falls back
to the 4K original, which scrubs less smoothly.

## What it does

- **Live telemetry HUD** — altitude, ground speed, climb/descent rate, heading,
  distance from home, GPS, ISO, shutter, aperture, EV, color temperature,
  camera clock and frame number, all synchronised to the video down to the
  single frame. The video image itself stays clean; everything lives in the
  interface.
- **Frame-accurate timecode** in `min:sec:frame`, with the scroll wheel over the
  video or timeline scrubbing exactly one frame per notch.
- **Flight path on a map** (dark or satellite) with a drone marker that turns to
  the current heading. Click anywhere on the track to jump the video there.
- **Charts** for altitude, speed and ISO — click or drag to seek, hover for
  values.
- **Coaching cues**, detected automatically from the telemetry and shown as
  bands on the timeline:

  | Cue | Why it matters in training |
  |---|---|
  | Stable hover | a usable inspection window — the shot a client can work with |
  | High speed close to the ground | little time to react to obstacles |
  | Rapid climb or descent | vortex ring state risk when descending fast |
  | High ISO | noise; discuss exposure choices |
  | Shutter vs. the 180° rule | stuttery motion for cinematic work |
  | Approaching the 120 m limit | EU open-category ceiling |

- **Markers** — drop timestamped notes by category (observation, risk, camera,
  navigation, POI), stored per clip in the browser, with export to JSON and CSV
  including the GPS position and telemetry at that moment. Import brings a
  colleague's markers back in, which makes debriefs shareable.
- **Estimate mode (T)** — hides all telemetry, including the timeline profile
  and the cue list, so trainees have to call out altitude, speed and position
  from the image alone. Toggle it back to compare their answer with the data.
- **Proxy / 4K switch** — scrub on the light 720p proxy, inspect detail on the
  4K original.

## Keyboard

| Key | Action |
|---|---|
| Space | play / pause |
| ← / → | one frame back / forward |
| Scroll over video or timeline | one frame per notch |
| Shift + ← / → | one second back / forward |
| M | drop a marker at the current moment |
| T | toggle estimate mode |
| 1–4 | speed ¼× · ½× · 1× · 2× |

## How it works

`server.py` is a small HTTP server with Range support, which is what lets the
browser seek inside multi-gigabyte video files; it also exposes `/api/clips`
listing the flights it found. The browser does the rest: `app.js` parses the
`.SRT` into typed arrays, derives ground speed, vertical speed, heading and
cumulative distance over a ±0.5 s window, downsamples for the charts and map,
and looks up the frame for the current playback position with a binary search
on every animation frame.

| File | Role |
|---|---|
| `server.py` | local server, video streaming, clip discovery |
| `index.html` + `style.css` | interface |
| `app.js` | SRT parsing, telemetry, HUD, charts, map, cues, markers |
| `Start GROUNDSTATION.command` | one-click start on macOS |

## Notes and limits

- Written against the `.SRT` format of recent DJI models (Mini, Air, Mavic).
  Other formats may parse partially; unknown fields simply show as `—`.
- Speed and heading are derived from GPS positions, not from the flight
  controller, so they lag slightly behind sharp manoeuvres.
- 4K HEVC playback depends on the browser; Safari and recent Chrome on macOS
  handle it, others may need the proxy.
- Markers live in the browser's local storage, per clip. Export them if they
  matter.

## License

MIT — see `LICENSE`.
