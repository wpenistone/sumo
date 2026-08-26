# Netconvert OSM-import reconciler — example fixtures

End-to-end fixtures that exercise the major features added on the
`netconvert/osm-reconciler` branch. Each subdirectory contains a hand-
written `input.osm.xml` and a `captured_output.txt` from running the
local netconvert build against it. To regenerate any output:

```
netconvert --osm-files <fixture>/input.osm.xml \
           [--osm.repair warn|infer] [--osm.lifecycle warn|include] \
           [--osm.date YYYY-MM-DD] \
           -o <fixture>/net.xml 2>&1
```

These are *not* part of the SUMO TextTest framework yet (the `tests/`
directory is excluded from this clone's sparse checkout). They live
under `docs/` so they're easy to inspect alongside the design doc; if
you want them as formal regression tests, copy them into
`tests/netconvert/import/OSM/` and add the standard
`options.netconvert` / `expected.*` files.

## Fixtures

### `lanes-balance/`

OSM way tagged `lanes=3`, `lanes:forward=2`,
`turn:lanes:backward=through|through` (2 pipes). The lane-count
balance constraint `total = forward + backward + both_ways` doesn't
hold (3 ≠ 2 + 2). With `--osm.repair=warn`, the importer emits:

```
Warning: Lane-count witnesses inconsistent for edge '1000':
total=3 (from lanes) but forward+backward=4 (forward=2 from
lanes:forward, backward=2 from turn:lanes:backward pipe count).
Resolved Edge fields unchanged.
```

Demonstrates: B2 (`checkLaneCountBalance`) observer wired to multiple
witness sources.

### `oneway-bus-contraflow/`

OSM way tagged `oneway=yes`, `oneway:bus=no`, `lanes=2`,
`lanes:backward=1`. This is the contraflow-bus-lane pattern — *not* a
contradiction. With `--osm.repair=warn`, the importer must NOT emit
the oneway/backward conflict warning, because the per-mode exception
explains the backward lane.

Demonstrates: `checkOnewayBackwardConflict` observer correctly
suppresses the warning when an `oneway:MODE=no` exception is present.

### `lifecycle-construction/`

Two ways: `3000` tagged `construction:highway=residential` with
`start_date=2025-06-01`; `3001` is operational `highway=residential`.

* Default (`--osm.lifecycle warn`): way `3000` is discarded with the
  message `Discarding non-operational way '3000' (lifecycle status:
  construction).` Only way `3001` ends up in the network.
* With `--osm.date 2026-05-12`: way `3000` is *included* as residential
  because the simulated date falls inside its `[start_date, end_date]`
  window. Verbose output shows `Including non-operational way '3000'
  (lifecycle: construction) because --osm.date=2026-05-12 falls within
  [start_date='2025-06-01', end_date='']`.

Demonstrates: E1 (lifecycle prefix) + E4 (date filter) interaction.

### `bidi-narrow/`

OSM way tagged `lanes=1` on a bidirectional `highway=unclassified`.

* Default: legacy half-width hack — two parallel SUMO lanes at
  `width=1.60` (half of the typemap default 3.20m).
* With `--osm.repair=infer`: emits a SUMO bidi edge pair —
  `<edge ... spreadType="center" bidi="-4000">` and the matching
  reverse — both edges share identical geometry and are marked as
  bidirectional partners.

Demonstrates: C1 (bidi-pair fix for `lanes=1` bidirectional ways).

### `access-vehicle-bus/`

OSM way tagged `vehicle=no`, `bus=yes`. Tests the strict-OSM bus
semantics from the most recent commits: `vehicle=no` disallows all
wheeled vehicles (motor + bicycle) without an implicit public-transport
carve-out, and `bus=yes` re-allows buses via the explicit-allow
override (`myExplicitlyAllowed` field + generalised conflict resolution
that replaces the old line-524 special case).

Resulting lane permissions include `bus` but NOT `passenger`/`taxi`/
`hgv`/`truck`/`motorcycle`/`moped` etc. (Bicycle remains allowed only
because `--osm.bike-access` is off by default — that's a pre-existing
SUMO design choice unrelated to this branch.)

Demonstrates: A1+A2+A3 (access broadening), explicit-allow tracking,
strict bus semantics.

### `maxspeed-per-lane/`

OSM way tagged `lanes=3 maxspeed=100 maxspeed:lanes=80|100|130`.
Verified by inspecting the resulting `<lane>` elements: each lane gets its
own `speed` attribute (right-to-left mapping from the OSM pipe order
because SUMO indexes lanes right-to-left in right-hand-traffic networks).

Demonstrates: D1 (per-lane maxspeed override).

### `fuzzy-typos/`

OSM way tagged `turn:lanes=lleft|throughleft|rigth` — three different
data-quality issues in one tag:

* `lleft` — typo, fuzzy-matched to `left` (Levenshtein distance 1).
* `throughleft` — missing `|` separator; the heuristic identifies it as
  `through` + `left` and warns accordingly.
* `rigth` — typo, fuzzy-matched to `right`.

Run with `--osm.repair=infer` to enable both fuzzy match and missing-pipe
detection. Captured output shows three distinct warning messages, each
naming the original code and the proposed interpretation.

Demonstrates: G2 (unknown-value warning), G3 (Levenshtein fuzzy
auto-repair), and the §4 missing-pipe heuristic.

### `plausibility-warnings/`

Five ways with deliberately suspicious tagging:

| Way | Tagging | Expected warning |
|---|---|---|
| 8001 | `highway=motorway lanes=1` | Suspicious lanes=1 on motorway. |
| 8002 | `highway=residential maxspeed=120` | Suspicious 120 km/h on residential. |
| 8003 | `highway=residential lanes=4` | Suspicious lanes=4 on residential. |
| 8004 | `highway=motorway lanes=14` | Implausible lanes=14 (above 10 cap). |
| 8005 | `lanes=4 width=6` | Implausibly narrow 1.50 m per lane. |

Demonstrates: `checkLaneCountPlausibility`, `checkSpeedPlausibility`,
`checkWidthPlausibility` from §4.

### `unit-normalization/`

Three ways with non-standard speed-unit casing / abbreviation:

* `maxspeed=50 MPH` → 22.35 m/s (uppercase MPH normalized to mph)
* `maxspeed=80 KMH` → 22.22 m/s (uppercase no-slash KMH normalized)
* `maxspeed=20 kts` → 10.29 m/s (knots abbreviation)

Verified by inspecting the resulting `<lane>` `speed` attributes.

Demonstrates: G7 (case-normalize speed units + accept knot/kts synonyms).

## What these fixtures don't do

* They aren't connected to the TextTest harness — running them is
  manual.
* No automated assert that the captured output matches the file on
  disk; if you change behaviour, regenerate by hand.
* They cover the headline features but are not exhaustive (no fixtures
  for D1 per-lane maxspeed, F2 multi-via, the per-lane access tags,
  speed plausibility, etc.). Easy to extend by following the same
  pattern.
