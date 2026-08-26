---
title: Netconvert OSM Import — Improvements & Auto-Healing Plan
status: draft
created: 2026-05-12
owner: William Harrison Davis
scope: src/netimport/NIImporter_OpenStreetMap.{cpp,h}; data/typemap/osm*.typ.xml; src/netimport/NIFrame.cpp
out_of_scope: pedestrian and cycleway tag handling (bicycle:*, foot:*, cycleway:*, sidewalk:*)
---

# Netconvert OSM Import — Improvements & Auto-Healing Plan

Audit-grounded list of improvements to SUMO's OSM importer, organized around an
evidence-based reconciliation framework rather than tag-by-tag patches. All
file:line references are against current `main` at audit time
(`b18877db5f8 updating debian packaging #3`).

## 1. Framing

### 1.1 Today's importer treats each tag as authoritative for one slot

Every fix to a real-world OSM-to-SUMO mismatch ends up as another `if (key ==
...)` branch in a 3000-line file. Tags that carry overlapping evidence
(`lanes`, `lanes:forward`, `turn:lanes`, `*:lanes`, per-direction width) do not
inform each other. Conflicts are silently resolved by whichever branch ran
last; missing data is silently filled with typemap defaults.

### 1.2 The right model is evidence-based reconciliation

Every OSM tag is a *witness* for one or more derived attributes
(`lanes_total`, `lanes_forward`, `lanes_backward`, `is_oneway`, `speed_*`,
`access_per_lane`, `lifecycle_status`). Each derived attribute is *resolved*
by a per-attribute rule that consults all witnesses, applies a priority +
conflict policy, and records provenance. Without this, every additional
inference rule deepens the existing spaghetti.

### 1.3 Lifecycle-prefix support and date-aware import are different features

These were lumped together in early discussion but should ship as separate
PRs:

- **Prefix support** (`construction:highway=*`, `disused:highway=*`, etc.) is
  a parser fix — current key allowlist at `NIImporter_OpenStreetMap.cpp:1913–1956`
  silently drops them. Tuesday-afternoon work.
- **`--osm.date` filtering** requires a small design (default behaviour,
  back-compat, fuzzy date handling) and a new CLI option in `NIFrame.cpp`.

### 1.4 Root-cause refactor under the lane fixes

`Edge::myNoLanes` and `Edge::myNoLanesForward` overload sign as direction:
backward lane count is stored as a *negative number in the forward field*
(`NIImporter_OpenStreetMap.cpp:2212`). Every reconciliation rule has to
remember the convention. **Almost every "lane arithmetic" gap shares this root
cause.** Repair on top of this representation will keep producing edge cases.
The recommended path is:

1. Phase A: extract the witness data into an explicit struct (no behaviour
   change).
2. Phase B+: build inference rules on top of the explicit struct.

## 2. Inference framework

### 2.1 Per-edge evidence table

```cpp
template <typename T> struct Evidence {
    T value;
    std::string source_tag;          // e.g. "turn:lanes pipe count"
    enum class Confidence { High, MediumHigh, Medium, Low, Lowest };
    Confidence confidence;
};

class OSMTagEvidence {
    std::vector<Evidence<int>>     lanes_total_witnesses;
    std::vector<Evidence<int>>     lanes_forward_witnesses;
    std::vector<Evidence<int>>     lanes_backward_witnesses;
    std::vector<Evidence<bool>>    oneway_witnesses;
    std::vector<Evidence<double>>  speed_forward_witnesses;
    std::vector<Evidence<double>>  speed_backward_witnesses;
    // ...
};
```

The existing parse loop populates this struct. A `resolve()` pass writes the
final `NBEdge` fields. Two non-negotiable properties:

- **Provenance**: every resolved value records its winning witness (and any
  losers) as `<param>` on the SUMO edge:
  `<param osm.lanes.source="turn:lanes pipe count" osm.lanes.alt="lanes=2 (rejected: smaller)"/>`
  Without this, the auto-repair becomes unreviewable.
- **Opt-in via CLI**: `--osm.repair <off|warn|infer|aggressive>`, default
  `warn`.
  - `off`: pre-refactor behaviour (silent fallthroughs).
  - `warn`: today's silent issues become warnings; nothing else changes.
  - `infer`: cross-witness rules from §3 fill missing values.
  - `aggressive`: rules can override explicit tags when other witnesses
    strongly disagree.

### 2.2 Uniform resolver math

To avoid re-inventing per-attribute logic, define three primitives that every
rule and every attribute uses.

#### 2.2.1 `resolve<T>` — single primitive for picking a value from witnesses

```cpp
enum class ConflictPolicy {
    PickHighestConfidence,   // tie-break: latest-added wins
    PickMax,                 // numeric only; for "trust the larger source"
    PickMin,                 // numeric only; for safety-conservative attrs
    MostRestrictive,         // for access bitmasks: AND them
    LeastRestrictive,        // for access bitmasks: OR them
    UnanimousOrWarn,         // require all witnesses to agree
    PickHighestConfidenceWarnOnDisagreement,  // common case
};

template <typename T>
struct ResolveResult {
    T value;
    Evidence<T> winner;            // for provenance
    std::vector<Evidence<T>> losers;
    bool had_conflict;
};

template <typename T>
ResolveResult<T> resolve(
    const std::vector<Evidence<T>>& witnesses,
    ConflictPolicy policy,
    std::optional<T> fallback);
```

Every per-attribute resolver in §3 is one call to `resolve<T>` with a chosen
policy and the witness list. No bespoke per-attribute resolution code. The
choice of policy is the only per-attribute decision:

| Attribute | Policy |
|---|---|
| `lanes_total` | `PickHighestConfidence`; `lane_count_balance` cross-check (§2.2.2) |
| `lanes_forward`, `lanes_backward`, `lanes_both_ways` | Derived from `lane_count_balance` when possible; fallback `PickHighestConfidence` |
| `is_oneway` | `PickHighestConfidence` |
| `speed_forward`, `speed_backward` | `PickHighestConfidence` |
| `access_per_lane` (bitmask per lane) | `MostRestrictive` (intersection across witnesses) |
| `access_per_edge` | `LeastRestrictive` (union; per-lane refines) |
| `lifecycle_status` | `PickHighestConfidence` |

#### 2.2.2 Constraints — separate from rules

A **constraint** is a statement that must hold among resolved attribute values
on a single edge, regardless of where the values came from. Constraints are
declarative; the resolver uses them to fill missing values and to detect
contradictions.

Initial constraint set. Each has a descriptive identifier — no opaque
numeric labels like `C4` in code or warnings; the rule registry uses the
descriptive names below. (Note: improvement-list items in §5 still use
short IDs like A1, B2, C1 — those are stable cross-reference handles for
this document, not constraint labels.) When emitted in output, warnings
name what's actually wrong in plain language, citing attribute and witness
values:

```
lane_count_balance:
    lanes_total = lanes_forward + lanes_backward + lanes_both_ways

oneway_blocks_backward_unless_excepted:
    is_oneway implies lanes_backward = 0,
    except when at least one oneway:MODE=no witness exists

per_lane_pipe_count_matches_direction:
    for any per-lane suffixed tag (width:lanes:DIR, turn:lanes:DIR,
    maxspeed:lanes:DIR, hgv:lanes:DIR, bus:lanes:DIR, ...):
        len(tag) in {0, lanes_DIR}

lane_counts_nonnegative_and_bounded:
    0 <= lanes_DIR <= lanes_total  for each direction

speed_positive_or_unset:
    speed_DIR > 0  OR  speed_DIR == MAXSPEED_UNGIVEN

per_lane_widths_fit_in_total_width:
    sum(width:lanes:DIR) <= width  (when both are set)
```

Constraint identifiers are internal — they appear as enum names in the rule
registry, not in user-facing warnings. A warning reads:

> way/12345: turn:lanes:backward has 2 entries but resolved 1 backward
> lane (lanes=3, lanes:forward=2 → backward=1). Truncated turn-sign data
> to 1 entry.

— not "Constraint per_lane_pipe_count_matches_direction violated."

Resolution algorithm per attribute:

```
1. Collect witnesses (parse loop fills them).
2. Apply constraints in topological order (§2.2.3) to derive any missing
   values that constraints can compute (e.g., lanes_backward from
   lane_count_balance).
3. Call resolve<T> on the (witnesses + constraint-derived values) list.
4. If resolved value violates a constraint:
   - At repair=warn: keep the value, emit a plain-language warning naming
     the conflicting witnesses and the resolved value.
   - At repair=infer: re-resolve excluding the witnesses involved in the
     violation; if a consistent value exists, use it; warn.
   - At repair=aggressive: pick the value that satisfies the most
     constraints and the highest-confidence witnesses; warn.
```

Adding a new tag = adding a new witness to the right list. Adding a new
relationship = adding a new constraint. Neither requires editing the resolver.

#### 2.2.3 Dependency graph and topological order

Attributes have dependencies: per-lane access cannot be resolved before
`lanes_forward`/`lanes_backward` because the per-lane pipe-count constraint
needs the lane count. Express the order as a DAG:

```
is_oneway  ──┐
             ├──> lanes_total ──┬──> lanes_forward ──┐
oneway:MODE ─┘                  ├──> lanes_backward ─┼──> per_lane_access
                                └──> lanes_both_ways ┘                     │
                                                                            ├──> turn_signs_per_lane
lifecycle_status ──> (gates whether edge is created at all)                 ├──> width_per_lane
                                                                            └──> speed_per_lane
speed_forward, speed_backward (independent of lane resolution)
```

The DAG is small and acyclic by construction. A topological sort gives the
resolution order. Cyclic dependencies (if any are introduced later) are a
modeling error — the registry should refuse to add a rule that creates a
cycle, rather than allowing fixpoint iteration.

#### 2.2.4 What this buys

- One `resolve<T>` to debug, test, instrument.
- Constraints are declarative — readable as a list, not buried in if/else
  chains in 30 different functions.
- Adding a new tag is mechanical: extend the witness list, run tests.
- Adding a new constraint is mechanical: append to the list, run tests.
- The "math" is the same for lanes, speeds, access — only the policy differs.

#### 2.2.5 Limits — what this framework will *not* fix

Honest accounting. The framework is leverage, not magic.

- **Single-witness wrong data.** If only `maxspeed=200` is tagged on a
  residential street, no other witness contradicts it; the resolver picks
  `200`. Negative-inference warnings from §4 catch *some* of these but only
  the ones we anticipated.
- **Cross-edge reasoning.** Divided carriageways, name continuity along a
  route, paired one-ways — none are per-edge. The framework is per-edge by
  design; cross-edge passes are a separate post-processing layer.
- **Time-varying / conditional values.** `maxspeed:conditional`,
  `access:conditional`, `oneway:conditional`. Resolver has no value to pick
  because the value is a function over time. These are punted to `<param>`
  per A8/D4 — the framework doesn't pretend to solve them.
- **Country-/locale-dependent rule variation.** Left-hand-traffic countries
  reverse the meaning of `turn:lanes` lane order. This must enter as global
  context (an option or driving-side detection), not as a per-edge witness.
  Until then, B6 stays warn-only.
- **Genuinely ambiguous tag combinations.** Cases where two tags are
  individually correct but jointly underdetermined (e.g. asymmetric
  `lanes:forward` + `lanes:both_ways` with no per-direction widths). The
  resolver picks one interpretation; the other is silently wrong. Document
  the chosen convention and live with it.
- **Novel tag patterns the rule-author didn't anticipate.** Framework
  executes rules you wrote. It doesn't discover new ones. New OSM conventions
  require new witnesses/constraints to be added by hand.
- **Rules that interact non-locally.** "If this edge has a bus lane and the
  next edge doesn't, drop the bus designation here too." Not expressible per
  edge; needs a connection-aware pass.

Realistic ceiling: this gets ~70–80% of "the OSM has overlapping evidence and
needs reconciliation" cases handled cleanly, with provenance, behind one
flag. The remaining 20–30% is either (a) hard heuristics that should stay
warn-only (turn:lanes flip detection, parallel-way detection), (b) genuinely
ambiguous cases where SUMO must pick a convention, or (c) cross-edge / global
context that doesn't fit a per-edge model. The framework's job is to make the
70–80% legible and the 20–30% explicitly out-of-scope, instead of all of it
buried in a 3000-line file.

### 2.3 Architectural placement

Insert `OSMTagReconciler` between the existing parse loop and `insertEdge()`,
replacing scattered ad-hoc reconciliation at:

- `NIImporter_OpenStreetMap.cpp:677–695` (lane allocation by direction)
- `NIImporter_OpenStreetMap.cpp:753–767` (the `lanes=1` bidirectional half-width hack)
- `NIImporter_OpenStreetMap.cpp:1563–1577` (`applyTurnSigns`)
- `NIImporter_OpenStreetMap.cpp:2170–2230` (lane-count tag parsing)

Rules registered in an array of functions:
`bool apply(OSMTagEvidence&, RepairLevel)`. Adding a new rule = adding one
function. Each rule testable in isolation against an OSM-XML fixture (see §6).

## 3. Per-attribute reconciliation rules

### 3.1 `lanes_total`

Witnesses, in roughly trust-descending order:

| Witness | Confidence | Notes |
|---|---|---|
| `lanes=N` | High | Today: authoritative. With B4: warn on `lanes=3;4`. |
| `lanes:forward + lanes:backward + lanes:both_ways` | High | Today: silently wrong on mismatch (`:2198`). New: warn, prefer explicit sum. |
| `max(pipe_count(turn:lanes:forward/backward/both_ways))` summed appropriately | Medium-High | Today: never used to infer (`:1574` discards on mismatch). New: if `lanes` unset, use this. |
| `max(pipe_count(*:lanes))` for `*` ∈ {`vehicle`, `motor_vehicle`, `motorcar`, `hgv`, `taxi`, `bus`, `psv`, `motorcycle`, `moped`, `width`, `maxspeed`, `change`, `destination`, `placement`, `surface`, `smoothness`} | Medium | None used for inference today. |
| `width / SUMO_const_laneWidth` (rounded) | Low | Sanity check; flag if disagrees with explicit `lanes` by >1. |
| Highway-class default from typemap | Lowest | Current fallback. |

**Resolution rule**: if explicit `lanes=` exists, use it but warn if any
pipe-count witness disagrees. If `lanes=` absent, use highest pipe-count among
witnesses, fall back to typemap default.

### 3.2 Directional split (`lanes_forward`, `lanes_backward`)

| Witness | Confidence |
|---|---|
| `lanes:forward` + `lanes:backward` (both present) | High; cross-check sum vs. `lanes` |
| Only one of `lanes:forward` / `lanes:backward` + total `lanes` | High; subtract |
| `pipe_count(turn:lanes:forward)`, `pipe_count(turn:lanes:backward)` | Medium |
| `pipe_count(*:lanes:forward)` for any `*` | Medium |
| `oneway=yes` + total `lanes` → all forward | High |
| `oneway:MODE=no` (any per-mode exception: `bus`, `psv`, `hgv`, `motor_vehicle`, ...) → witness for `lanes_backward >= 0` with the lane-budget rule (§3.2.1) | Medium-High |
| Highway-class default split (motorway: balanced; trunk_link: 1+0) | Low |

#### 3.2.0 Worked example: `lanes=3` + `lanes:both_ways=1` (center TWLTL)

OSM `lanes:both_ways=N` denotes a center two-way left-turn lane (TWLTL),
the shared center lane on many suburban arterials. The framework handles the
*arithmetic* with no new code:

1. Witnesses: `lanes_total←3` (High), `lanes_both_ways←1` (High).
2. `lane_count_balance` (`total = forward + backward + both_ways`) derives
   `forward + backward = 2`.
3. With no per-direction witnesses, symmetric default (Low) gives
   `forward=1, backward=1`.
4. `resolve` writes `(1, 1, 1)`. Provenance:
   `<param osm.lanes.split.source="symmetric default after lane-count balance"/>`.

**SUMO model gap**: SUMO has no native TWLTL primitive (unlike the
bidirectional `lanes=1` case in C1, which is correctly representable as a
bidi edge pair — see C1 for that pattern). Treatment options for
the center turn lane:

| Option | Geometry | Capacity | Turn semantics | Cost |
|---|---|---|---|---|
| A. Drop both_ways; build 1-fwd/1-bwd | Wrong (2-lane look) | Right | Wrong (no turn refuge) | One-line |
| B. Add as extra lane in dominant direction | Wrong (asymmetric) | Wrong | Partial | Trivial |
| C. Forward + backward lane sharing the same geometric strip, both restricted to left-turn access | Right | Slight overstate | Right | Medium |
| D. Build the both_ways lane only at junctions where left turns happen (turn-pocket modeling) | Right | Right | Right | Large — needs junction-aware import logic |
| E. Use SUMO `changeLeft`/`changeRight` to mark a normal lane as left-turn refuge | Wrong | Right | Approximate | Small (after B10) |

**Recommended default: A** (drop both_ways with warning + `<param osm.lanes.both_ways="1"/>` retained). Make C/E available behind
`--osm.both-ways-lane=geom-overlap` / `=change-marking` once underlying support exists.

Asymmetric variant (`lanes=3 lanes:both_ways=1 lanes:forward=2`):
`lane_count_balance` derives `backward=0`. That's the genuine contradiction
— a road with 2 forward + 1 turn lane + 0 backward is inconsistent with the
way being bidirectional (the OSM default for non-motorway, non-roundabout
ways). Resolver picks the explicit `lanes:forward=2` witness over the
default split; warn that the bidirectional resolution requires at least one
backward lane. Under `infer`, prefer dropping `lanes:forward=2` and keeping
the symmetric split.

#### 3.2.0b Worked example: `lanes=3 turn:lanes:forward=through|through` (clean inference)

Witnesses:
- `lanes_total ← 3` (High)
- `lanes_forward ← pipe_count(turn:lanes:forward) = 2` (Medium-High, via the
  per-lane pipe-count constraint)

`lane_count_balance` derives `lanes_backward = 3 − 2 − 0 = 1`. No conflict,
no warning.

Today's importer arrives at the same `(2, 1)` split via the asymmetric
ceiling default, but without the `turn:lanes:forward` evidence — same
answer, different epistemics. The framework's value is that it can *explain*
the choice and warn if any contradicting witness appears.

#### 3.2.0c Worked example: `lanes=3 lanes:forward=2 turn:lanes:backward=through|through` (irreducible contradiction)

Witnesses:
- `lanes_total ← 3` (High)
- `lanes_forward ← 2` (High, from `lanes:forward`)
- `lanes_backward`:
  - derived via `lane_count_balance`: `3 − 2 = 1` (High, derived from two
    High witnesses)
  - via per-lane pipe-count: `pipe_count(turn:lanes:backward) = 2`
    (Medium-High)

No consistent assignment exists. Either `backward=1` (and the
`turn:lanes:backward` pipe count is too long) or `backward=2` (and
`lanes:forward + lanes:backward = 4` exceeds the declared total of 3). The
OSM data is wrong somewhere; the framework cannot determine which of three
tags is the bug:

- `lanes=3` might be wrong (should be 4)
- `lanes:forward=2` might be wrong (should be 1)
- `turn:lanes:backward=through|through` might be wrong (extra pipe)

Resolver behaviour:
- At `warn`/`infer`/`aggressive`: pick by `PickHighestConfidence` →
  `forward=2, backward=1`. Truncate `turn:lanes:backward` to 1 entry. Emit a
  warning naming all three candidate fixes.
- No auto-flip of the explicit `lanes:forward` value. That's the kind of
  dangerous heuristic B6 cautions against — the cost of a wrong flip exceeds
  the cost of a warning.

Today: `applyTurnSigns` (`:1574`) silently drops the entire backward turn
vector on count mismatch. The user has no signal that anything is wrong.
Framework value here is not "fixed it" — it's "surfaced the contradiction
with all three explanations and recorded provenance."

#### 3.2.1 Contraflow lane handling (oneway + per-mode exception)

**This is a real, intentional pattern, not a contradiction.** Examples seen
in real OSM:

- `oneway=yes` + `oneway:bus=no` + `lanes:backward=1` — physical contraflow
  lane reserved for buses.
- `oneway=yes` + `bus:lanes:backward=designated` — same intent, more explicit
  per-lane form.
- `oneway=yes` + `oneway:psv=no` (and friends).
- `oneway=yes` + `lanes=2` + `lanes:forward=1` + `oneway:bus=no` — total of
  2 lanes, 1 forward, the *other one* is the contraflow bus lane (implicit
  from the lane budget arithmetic).
- `oneway=yes` + `lanes=1` + `oneway:bus=no` — no extra lane to spare; buses
  share the single forward lane bidirectionally as a legal exception.

Today: importer at `NIImporter_OpenStreetMap.cpp:2161–2165` handles
`oneway:bus=no` and `oneway:psv=no` to "create backward busway", but the
interaction with `lanes`, `lanes:forward`, `lanes:backward`, and
`bus:lanes:backward` is not reconciled.

**Treat `oneway:*=no` as a first-class witness, not a special case.**
`oneway:bus=no` should appear in the `lanes_backward` witness list, asserting
"there is some backward travel possibility for buses." Whether that becomes a
dedicated physical lane or a shared bidirectional lane falls out of the lane
budget:

```
lane_budget_for_backward = lanes_total - lanes_forward
                                       - lanes_both_ways

if any oneway:MODE=no exists:
    if lanes:backward is explicit:
        lanes_backward = lanes:backward
        (lanes are dedicated/restricted to the exception modes)
    elif lane_budget_for_backward >= 1:
        lanes_backward = lane_budget_for_backward
        (lanes_backward is the implicit contraflow lane(s), restricted to
         the exception modes; no warning)
    else:
        lanes_backward = 0
        (buses share the forward lane bidirectionally; emit
         <param osm.bus.shared_contraflow="true"/> for downstream awareness)
```

Concrete walkthroughs:

| Tags | Resolved |
|---|---|
| `oneway=yes` `lanes=2` `oneway:bus=no` (no `lanes:forward/backward`) | forward=1 (default split), backward=1 bus-only (budget=2-1=1). Provenance: backward inferred from `oneway:bus=no` + lane budget. |
| `oneway=yes` `lanes=2` `lanes:forward=1` `oneway:bus=no` | forward=1, backward=1 bus-only (budget=2-1=1). Provenance: explicit `lanes:forward`, backward from budget+exception. |
| `oneway=yes` `lanes=3` `lanes:forward=2` `oneway:bus=no` | forward=2, backward=1 bus-only (budget=3-2=1). |
| `oneway=yes` `lanes=1` `oneway:bus=no` | forward=1, backward=0; buses traverse the forward lane in the opposite direction (shared contraflow). Emit `<param osm.bus.shared_contraflow="true"/>`. |
| `oneway=yes` `lanes=2` `lanes:backward=1` `oneway:bus=no` `bus:lanes:backward=designated` | forward=1 (budget), backward=1 bus-designated. All witnesses agree. |
| `oneway=yes` `lanes=2` `lanes:forward=2` `oneway:bus=no` | forward=2, backward=0 (budget=0); fall back to shared-contraflow case. Emit warning under `warn` that `lanes:forward` exhausts the budget despite an exception. |

**Warning condition** (more careful than the original suggestion): warn only
if `oneway=yes` + `lanes:backward >= 1` AND no per-mode `oneway:*=no`
exception AND no `*:lanes:backward` access tag. That's the genuine
contradiction.

This is the kind of flexibility the framework is for: rather than special-case
each combination (`oneway=yes` + `lanes:backward`, `oneway=yes` +
`oneway:bus=no`, `oneway=yes` + `lanes=2` + `lanes:forward=1` +
`oneway:bus=no`, ...), express `oneway:*=no` as a witness for backward access
and let the lane-budget rule derive the lane count. One rule, all combinations
handled uniformly.

#### 3.2.2 Other healing examples

- `oneway=yes` + `lanes:backward=2` with no per-mode exceptions → warn
  ("backward lanes ignored due to oneway"), drop the backward.
- `lanes=4` + `lanes:forward=2` + `turn:lanes:forward=left|through|through|right`
  (4 pipes) → conflict; trust pipe count under `infer`, warn under `warn`.
- `lanes=3` on a bidirectional way with no directional split tags → today:
  2 forward / 1 backward (silent ceil at `:684`). New: emit
  `<param osm.lanes.split="implicit-asymmetric"/>` so user can audit.

### 3.3 `is_oneway`

Witnesses beyond `oneway=*`:

- `junction=roundabout` → implies `oneway=yes`. (Verify current behaviour;
  if `oneway=no` is explicit on a roundabout, recommend roundabout wins +
  warn.)
- `highway=motorway` → strong implicit oneway prior. Today via typemap.
- `highway=*_link` connecting to a oneway way → topological hint.
- `turn:lanes:backward` populated with no per-mode oneway exception while
  `oneway=yes` → contradiction; under `aggressive`, suggest unsetting
  oneway; under `warn`, just report.
- Two parallel ways within ~5 m sharing `name` and `ref` with opposite
  digitization → divided-carriageway hint (don't auto-merge, flag for
  inspection).

### 3.4 `speed_forward` / `speed_backward`

Witnesses:

- `maxspeed=*` (raw, parsed) — High
- `maxspeed:forward`, `maxspeed:backward` — High, override per direction
- `maxspeed:type=DE:urban` — Medium, fallback if `maxspeed` unset
- `zone:maxspeed=*` — Medium
- `source:maxspeed=sign` — informational only
- `mean(maxspeed:lanes)` — Sanity check (warn if mean ≠ overall by >5 km/h)
- `traffic_calming=*` set → cap at 30 km/h if `maxspeed` unset (Low; opt-in
  only)
- `surface=unpaved|gravel|dirt` + `maxspeed` unset → cap at 50 km/h (Low;
  opt-in only)
- Highway-class default — Lowest

**Healing**: `maxspeed=signals` (currently → `MAXSPEED_UNGIVEN`,
`:1771`) should fall back to typemap default *and* emit
`<param osm.speed.note="signals"/>`, rather than ending up with no usable
speed.

### 3.5 Per-lane access reconciliation

Today there is no reconciliation between, e.g., `bus:lanes=no|no|designated`
and `psv:lanes=no|no|yes` and the per-edge `bus=yes`. Rules:

- Per-lane tag count must equal resolved `lanes_forward` (resp. `_backward`).
  If not — warn; if `--osm.repair=infer` and the count matches a *different*
  lanes witness, prefer that lanes value.
- `bus:lanes=...|designated|...` on a lane that `vehicle:lanes` says `no` →
  not a contradiction (designated bus lane closed to general traffic). Resolve
  by taking *intersection* of allow-sets per lane.
- `psv:lanes` and `bus:lanes` should agree on bus lanes; warn on disagreement.
- Per-edge `bus=yes` + `bus:lanes=no|no|no` → not a contradiction (bus allowed
  in mixed traffic). Per-edge tag wins; per-lane tag refines.
- `access=no` + `bus=yes` → only buses. Today A1 doesn't read `access` values;
  this rule depends on A1 landing first.

### 3.6 Lifecycle reconciliation (depends on §5.E)

Witnesses for `lifecycle_status`:

- `highway=construction` + `construction=residential` → status=construction,
  resolved_class=residential
- `construction:highway=residential` (prefix form) → same
- `disused:highway=*`, `abandoned:highway=*`, `razed:highway=*`,
  `was:highway=*` → status from prefix
- `disused=yes` + `highway=residential` → status=disused,
  resolved_class=residential
- `start_date` in future, no `end_date` → status=planned/construction
- `end_date` in past → status=disused/historic
- `temporary=yes` + `start_date`/`end_date` → handle window

**Healing**: with `--osm.date YYYY-MM-DD`, *include* construction/proposed
ways whose date window covers the simulated date, importing them as their
resolved class. *Exclude* operational ways whose `end_date` < simulated date.
Today: `highway=construction` is unconditionally discarded by typemap
(`osmNetconvert.typ.xml:43`), losing both the geometry and the future
inclusion option.

## 4. Negative-inference warnings (data-quality flags)

Cheap and high-value — these don't auto-heal, just warn. Controlled by
`--osm.warn-suspicious` (default on); emit a structured per-edge warning so a
downstream report can summarize them.

| Pattern | Why suspicious | Notes |
|---|---|---|
| `lanes=1` + `highway=motorway` | Almost always mistagged. | |
| `lanes >= 4` + `highway=residential` | Likely should be `tertiary`+. | |
| `lanes > 10` (total) | Real-world max is rare; almost always a tagging error. | **Added.** |
| `lanes > 6` + not in {`motorway`, `trunk`, `primary`} | Suspicious for the class. | |
| `maxspeed >= 100` + `highway=residential` | Class probably wrong. | |
| `oneway=yes` + `lanes:backward >= 1` + **no per-mode `oneway:*=no` exception** + **no `*:lanes:backward`** | Genuine contradiction. | **Refined** — bare condition is too aggressive (contraflow lanes are real). |
| `turn:lanes` pipe count ≠ resolved `lanes` | Today silently dropped (`:1575`). | |
| `turn:lanes:forward=right\|left\|left` (left turns clustered on right side) | Probable direction flip. | Warn-only, never auto-flip. |
| `width / lanes < 2.0 m` | Implausibly narrow. | |
| `width / lanes > 6.0 m` AND **not** `junction=roundabout` AND **not** `highway=service` | Implausibly wide → probably `lanes` undercount. | **Refined** — wide carriageways at roundabouts and parking aisles are normal. |
| `maxspeed:lanes` length ≠ `lanes` | Tag/data mismatch. | |
| `*:lanes` non-empty + `lanes` unset | Easy free inference (don't warn under `infer`; warn otherwise). | |
| `highway=construction` + missing `construction=*` | Can't resolve class. | |
| `start_date` in past + still tagged `highway=construction` | Stale OSM data. | |
| `end_date` set + no lifecycle prefix/value | OSM author meant the road is gone but didn't say so. | |
| Tag value not in canonical set for that key (e.g. `turn:lanes` containing `thruough`, `oneway` containing `truee`) | Likely typo; today silently dropped without warning. See §5.G2. | **Added.** |
| Pipe-separated value with one entry whose length is ≥ 2× the longest canonical code | Probable missing `\|` separator (`throughleft`, `leftleft`). See §5.G3. | **Added.** |

## 5. Concrete improvement list

References: all `:N` are line numbers in
`src/netimport/NIImporter_OpenStreetMap.cpp` unless otherwise noted.

### A. Access / vehicle-class restrictions

| ID | Improvement | Severity | Notes |
|---|---|---|---|
| A1 | Parse generic `access=*` values beyond `no`: `private`, `destination`, `customers`, `permissive`, `agricultural`, `forestry`, `delivery`. Today only `access=no` does anything (`:2118`). | High | Single switch in existing block. `destination` should map to `SVC_DELIVERY`-permissive semantics. |
| A2 | Parse `motor_vehicle=*` and `motorcar=*` as siblings of `vehicle=*`. Currently neither is read despite being OSM-standard for "no cars but other modes allowed". | High | Importer currently can't distinguish from full no-access. |
| A3 | Parse `hgv=*`, `taxi=*`, `motorcycle=*`, `moped=*` directly — none are read today. The `:lanes` variants of these (A6) are also missing. | High | Trivial to add to the existing if/else chain at `:2097–2161`. |
| A4 | Treat `designated` differently from `yes` for `bus`, `psv`, etc. Today identical (`:2097–2109`). `designated` should also raise the lane's priority for that mode (e.g., bus route generation). | Medium | `myExtraAllowed` is a flat bitmask — needs a "designated" flag like the per-lane case at `:2467`. |
| A6 | Add per-lane parsing for `hgv:lanes`, `motorcar:lanes`, `motor_vehicle:lanes`, `taxi:lanes`, `motorcycle:lanes`, `moped:lanes`. Infrastructure (`interpretLaneUse()`) exists; 5-line addition per tag at `:2284–2303`. | Medium | High value/effort ratio. |
| A7 | Parse `oneway:hgv`, `oneway:motor_vehicle` — currently only `oneway:bus` and `oneway:psv` actually create reverse access (`:2161–2165`). | High | Common on European urban roads. |
| A8 | Stub `access:conditional` / `motor_vehicle:conditional` to emit `<param>` on the edge so downstream tools (TraCI, route restrictions) can act on it. Don't try to interpret `Mo-Fr 07:00-19:00` syntax in netconvert. | Low | Pragmatic — full conditional language is a rabbit hole. |

### B. Lane arithmetic and `turn:lanes`

| ID | Improvement | Severity | Notes |
|---|---|---|---|
| B1 | Refactor `myNoLanes` / `myNoLanesForward` sign-encoding into an explicit struct (Phase A of §2). Pre-requisite for clean B2–B8. | High | Touches ~30 sites. |
| B2 | Validate `lanes:forward + lanes:backward == lanes` and warn on mismatch with the way's `id`. Today silently computes `lanes - lanes:forward` (`:2198`) even when both directional tags contradict the total. | High | Real-world OSM has this constantly. |
| B3 | Properly handle `oneway=yes` + `lanes:backward`: if a per-mode exception (`oneway:bus=no` etc.) exists, treat as contraflow (§3.2.1); only warn in the bare contradiction case. | Medium | **Updated to handle contraflow lanes.** |
| B4 | Replace the `lanes=3;4` semicolon-min behavior at `:2170–2192` with a warning + use of either max or first value. Min is the worst pick — discards lanes the highway actually has. | Low-Medium | Backward-compat concern: gate behind `--osm.repair=infer` if needed. |
| B5 | Reconcile `turn:lanes` pipe count with `lanes` *during parsing*, not at apply time. Today `applyTurnSigns()` (`:1563`) silently drops the entire turn vector if the count differs (`:1574`); pipe count never used to *infer* lane count. **Rule**: if `lanes` unset and `turn:lanes` has N pipes, set `lanes=N`. | High | Big quality win on under-tagged ways. |
| B6 | Detect "wrong-direction" turn:lanes (left turns clustered on right side). Warn only. **Never auto-flip without `--osm.turn-lanes.repair` opt-in** — a wrong flip causes silent topology errors that are miserable to debug. Hard to do robustly because of left-hand-traffic. | Medium | Heuristic risk. |
| B7 | Ensure `turn:lanes` (unsuffixed) is not blindly mapped to `:forward`. Today: `:2342: if (StringUtils::endsWith(key, "lanes") || StringUtils::endsWith(key, "lanes:forward"))` mixes them. For `oneway=-1` ways, an unsuffixed `turn:lanes` really means the *backward* (driving) direction. | High | Symptom of treating "OSM way direction" as "driving direction". |
| B8 | Infer additional lane count from `:lanes`-suffixed tags: if `psv:lanes=no\|no\|designated` is set on a way with no `lanes=`, count = 3. Today not used for inference. | Medium | Cheap once B5 lands. |
| B9 | Parse `lanes:both_ways=*` (currently only `turn:lanes:both_ways` recognized — `:2346`). | Low | Niche but well-defined. |
| B10 | Parse `change:lanes=*`, `change:lanes:forward=*` (lane-change prohibitions, e.g. `not_left\|yes\|not_right`) into NBEdge lane connection constraints. | Medium | Map to SUMO's `changeLeft`/`changeRight` lane attributes. |
| B11 | Parse `placement=*` (geometric offset of lanes vs. way centerline). Already noted at `:2166` as extra tag but not acted on. | Low | Fine-grained. |
| B12 | **Robust `turn:lanes` value parsing.** Currently the per-code if/else at `:2331–2346` silently drops anything not in the canonical set (`through/none/left/right/sharp_*/slight_*/reverse/merge_to_*`). A typo (`thruough`, `lleft`, `rigth`), a non-canonical synonym (`u_turn` for `reverse`, `straight` for `through`, `forward` for `through`), or a missing pipe collapsing two values (`throughleft`) all silently degrade to "no turn data for this lane" with no warning. **Rule**: when a code doesn't match, emit a structured warning naming the way, lane index, and offending code, and (under `--osm.repair=infer`) attempt fuzzy match via Levenshtein distance ≤ 2 against the canonical set. Document the canonical set centrally so the parser, the warning, and the fuzzy matcher share one source of truth. **`reverse` (U-turn) verification**: confirm it round-trips to `LinkDirection::TURN` correctly through `applyTurnSigns` and the B6 direction-flip detector treats it appropriately (a `reverse` on the leftmost lane in left-hand-traffic is normal; on the rightmost in right-hand-traffic, suspect). | Medium | Real-world OSM data has typos. Today they're invisible. |

### C. Lane width / "narrow road" handling

| ID | Improvement | Severity | Notes |
|---|---|---|---|
| C1 | **Fix the `lanes=1` bidirectional-way representation by emitting a bidi edge pair** (`:755–763`). Current code produces two parallel half-width SUMO lanes from what OSM means as a single shared bidirectional carriageway — wrong geometrically (two narrow strips instead of one shared strip) and wrong on capacity (sim permits simultaneous opposing traffic on a physically one-vehicle road). Trigger condition is a way the importer resolves as bidirectional (no `oneway=yes`, not a roundabout, not a motorway-class way) with `lanes=1` — `oneway=no` need not be explicitly tagged since it's the OSM default. The right SUMO representation is a **bidi pair**: two edges connecting the same nodes, sharing identical geometry, with `spreadType="center"` on both, and the pair marked as bidi so SUMO's right-of-way logic prevents simultaneous opposing traffic. Both edges fall out of the single OSM way's geometry, no manual coordinate adjustment needed. Drop the `/= 2` width halving and the `narrow` `routingType` flag (the bidi marking already conveys "vehicles take turns"). | High | Real bug. SUMO's bidi-edge primitive is the correct representation; importer just doesn't know about it. |
| C2 | When `width:lanes` count mismatches actual lane count, fall back to *partial* application + warning instead of dropping all width data (`:967`). | Low | Today: warning, then defaults applied to all. |

### D. Maxspeed

| ID | Improvement | Severity | Notes |
|---|---|---|---|
| D1 | Parse `maxspeed:lanes=*` and `maxspeed:lanes:forward/backward`. **Blocker**: edge has a single `myMaxSpeed`; per-lane needs `NBEdge::Lane::speed` override (already in data model — `setLaneSpeed`) wired through. | Medium | Real but rare on OSM. Worth doing because data model already supports it. |
| D2 | Parse `maxspeed:hgv`, `maxspeed:bus`. **Blocker**: SUMO edges have one speed per lane, not per (lane, vehicle-class). Model gap, not just parser gap. Recommend `<param>` for now; argue separately whether to extend `NBEdge`. | Low | Surface as discussion, not implementation. |
| D3 | Parse `maxspeed:advisory` as `<param>`. Don't change actual edge speed. | Low | Cheap. |
| D4 | Stub `maxspeed:conditional` as `<param>` (like A8). | Low | |
| D5 | Stub `maxspeed:variable=yes` as `<param>` so VSL-aware downstream tools can pick it up. | Low | |
| D6 | Document and re-examine the hardcoded `walk = 5 km/h` (`:1771`). OSM wiki: pedestrian flow 1.0–1.4 m/s ≈ 3.6–5 km/h, so 5 km/h is reasonable but undocumented. Add a one-line comment with the wiki reference. | Trivial | Trust improvement. |

### E. Lifecycle / temporal

| ID | Improvement | Severity | Notes |
|---|---|---|---|
| E1 | Add lifecycle prefixes to the key allowlist at `:1913–1956`. Today `construction:highway`, `proposed:highway`, `disused:highway`, `abandoned:highway`, `razed:highway`, `was:highway`, `planned:highway` are all dropped at the filter. **Behaviour after the fix**: strip the prefix and re-apply the importer with the resolved class, *plus* set a status flag. Default action: discard like `highway=construction` already does (`typ.xml:43`). | High | The "we just don't see proposed roads" problem. |
| E2 | Add a typemap entry for `highway.proposed` (currently missing — `addType()` at `:2088` produces `highway.proposed`, no match, falls through). | Medium | One-line typemap addition. |
| E3 | New CLI option `--osm.lifecycle <discard\|include\|warn>` (default `discard` for back-compat) controlling lifecycle-tagged way handling. Granularity: `--osm.lifecycle.construction`, `--osm.lifecycle.proposed`, `--osm.lifecycle.disused`. | Medium | Register in `NIFrame.cpp:181–240`. |
| E4 | New CLI option `--osm.date YYYY-MM-DD` (or `--osm.date today`). When set, the importer:<br>• reads `start_date=*` and `end_date=*`<br>• drops ways where the date is outside `[start_date, end_date]`<br>• combined with E1: a `proposed:highway=*` way with `opening_date=2025-01-01` is included if `--osm.date >= 2025-01-01`. | High | The "set date" feature. **Design question**: how to parse OSM date values like `2025`, `2025-Q1`, `~2026`, `summer 2026`? Recommendation: only accept ISO dates and warn on anything fuzzy. |
| E5 | Distinguish `disused` (physical road exists, closed to traffic) from `abandoned` (decaying, possibly impassable) from `razed/demolished` (gone). Default mapping: `disused` → discard but keep geometry available via `osm.all-attributes`; `abandoned` → discard; `razed` → never imported. | Low | "Road is on the map but not in the sim" vs. "data is historical". |
| E6 | Treat `highway=construction` more carefully. Today `osmNetconvert.typ.xml:43` discards unconditionally. With E3+E4, the user could opt to *include* construction roads as their resolved class (from `construction=*` tag). | Medium | Required for any "future road network" use case. |

### F. Cross-cutting and adjacent

| ID | Item | Why it matters |
|---|---|---|
| F1 | **A test corpus.** Minimal targeted unit tests exist for tag parsing — most coverage is integration via `tests/netconvert/import/osm/*`. Several improvements (B2, B5, B7, E1) are easy to write, hard to verify without synthetic-OSM fixtures per case. Add `tests/netconvert/import/osm/{access,lanes,maxspeed,lifecycle}/` with one OSM XML per scenario. | Without this, every PR is a regression risk. |
| F2 | **Implement `via-way` / `via-node` turn restriction relations** — `:2940` has the only on-topic `XXX` in the file. Today multi-segment turn restrictions (very common at large junctions) are silently dropped. Independent of this list but adjacent and high-value. | Already a known bug per existing TODO. |
| F5 | **Audit `--osm.all-attributes` interaction with the allowlist.** Today the `:1913–1956` allowlist drops keys *before* `--osm.all-attributes` can see them. So if a user wants to retain `start_date` as a `<param>`, they currently can't. Decouple "which keys do we *interpret*" from "which keys do we *retain*". | Quietly degrades extensibility. |

(F3 and F4 from earlier draft removed: pedestrian / cycleway scope.)

### G. Malformed input / data-quality auto-repair

OSM data is hand-edited by humans. Typos, missing pipe separators, casing
inconsistencies, and non-canonical synonyms are common. Today the importer
silently drops anything that doesn't match an exact-string check, with no
warning. The result: bad data degrades the network in ways the user can't
diagnose without re-running with verbose logging or reading the source.

| ID | Improvement | Severity | Notes |
|---|---|---|---|
| G1 | **Centralize canonical value sets per tag.** Today `turn:lanes` codes (`:2331–2346`), `oneway` values (`:2237`), `access` values (`:2118`), `bus`/`psv`/`foot` value mappings (`:2097–2158`) each have their own ad-hoc if/else with no shared source of truth. Extract to a single header (e.g. `NIOSMCanonicalValues.h`) with named sets per tag. Pre-requisite for G2 and G3. | Medium | Refactor; no behaviour change initially. |
| G2 | **Warn on unknown values for known tags.** Once G1 lands, every parse site that consults a canonical set can report unknowns: way id, tag key, offending value, candidate canonical values. Today these go to /dev/null. | High | Cheapest big visibility win. |
| G3 | **Optional fuzzy auto-repair behind `--osm.repair=infer`.** Levenshtein-distance-1-or-2 match against the canonical set fixes common typos (`thruough` → `through`, `lleft` → `left`, `rigth` → `right`). At `infer`, also detect missing-pipe collisions: a token that starts and ends with two distinct canonical codes (`throughleft`, `leftleft`) likely indicates a missing `\|`. Always emit a warning naming the original and the chosen replacement. **Never enable at `warn`** — silent value mutation without a flag is a data-corruption surprise. | Medium | Low risk if gated; high value for messy regions. |
| G4 | **Case normalization at the canonical-set lookup.** `Yes`/`yes`/`YES` should all be the same; today comparisons are case-sensitive in most sites. Trivial. | Low | |
| G5 | **`turn:lanes` synonym table.** `u_turn` → `reverse`, `straight` → `through`, `forward` → `through` (used in some regional schemas). Either accept silently as canonical aliases or accept-with-warning under `infer`. Stays separate from G3 (synonyms are *known* alternatives, not typos). | Low | |
| G6 | **`oneway` value normalization.** OSM accepts `yes`/`true`/`1`, `no`/`false`/`0`, `-1`/`reverse`. Today the importer stores the raw string into `myIsOneWay` and downstream comparisons are inconsistent — `myIsOneWay == "yes"` doesn't match `"true"`, etc. (`tracks=1` path even sets it to `"true"` while `oneway=yes` sets it to `"yes"`.) Normalize on parse to a tri-state. | Medium | Latent bug today. |
| G7 | **Maxspeed unit normalization survey.** `parseSpeed` already handles `mph`/`km/h`/`m/s`/`knots`. Audit for missed variants (`kph`, `kmh`, `kts`, no-space variants like `50kmh`). Warn on unknown units instead of failing parse. | Low | |

## 6. Pushback — what NOT to do

- **Don't auto-flip `turn:lanes` direction even at `aggressive`.** A wrong
  flip causes a silent topology error that's miserable to debug. Stay
  warn-only.
- **Don't infer `oneway` from geometry of paired ways.** "Two ways within X
  meters with the same name" fires on slip lanes, narrow alleys, indoor mall
  paths. Surface as a *report*, never an action.
- **Don't auto-correct `maxspeed` from `traffic_calming` or `surface` without
  explicit opt-in.** Some users care about legal speed only; they'd consider
  this corruption.
- **Don't enable fuzzy auto-repair (G3) at `--osm.repair=warn`.** Silently
  rewriting `thruough` to `through` without an explicit opt-in surprises
  users who were relying on the old behaviour (drop unknown). The warn
  level should stay strictly observational.
- **Don't fuzzy-match below Levenshtein distance 1.** A single-character
  typo is plausible; a two-character distance match risks corrupting
  intentional non-canonical values (regional dialects, proprietary
  schemas). Cap distance at 2 even at `aggressive`, and never replace if
  multiple canonical candidates tie.
- **Don't widen scope to `name:*` reconciliation, `ref` mismatch detection,
  or relations of relations.** Each is its own rabbit hole — the framework
  supports them later, but do not ship them in v1.
- **`<param>` provenance is not optional.** If you can't justify shipping the
  per-attribute source tag, don't ship the inference. Reviewing 100k
  auto-healed edges by eye is impossible without it.
- **D2 (per-vehicle-class maxspeed) is over-scoped relative to its real-world
  payoff.** Most OSM doesn't tag this. SUMO's data model doesn't trivially
  accommodate it. Skip unless a concrete scenario emerges.

## 7. Suggested execution order

1. **F1** test corpus (one OSM fixture per inference rule).
2. **B1 + Phase A** of the refactor (extract `OSMTagEvidence` — pure mechanical, no behaviour change).
3. **B2 + B3 + B5 + B8** delivered together via the new framework (lane reconciliation; was 5 PRs, now 1 coherent one).
4. **A1 + A2 + A3 + A7** access broadening (independent, no framework dep).
5. **§4 negative-inference warnings** (cheap; build operator trust before shipping auto-healers).
6. **Phase B** rules registry, then turn on `--osm.repair=infer`.
7. **E1–E4** lifecycle + `--osm.date`, on the clean foundation.
8. `<param>`-stub items (A8, D3–D5) anytime.

Headline: **the right unit of work is the reconciler architecture, not the
individual tag parsers.** Each tag-by-tag improvement on its own is fine;
together, without a shared evidence model, they pile up as new conditional
branches in an already 3000-line file. The framework is the leverage point.

## 8. Open design questions

- §3.2.1: how exactly should backward contraflow lane(s) inherit lane
  attributes (turn signs, widths) when they exist as per-mode access carve-outs
  rather than full bidirectional lanes?
- §3.3: when `junction=roundabout` + explicit `oneway=no` conflict, does
  current code already resolve in favour of the roundabout? (Verify before
  changing.)
- §5.E4: date parsing — accept only ISO, or also handle `YYYY-MM`, `YYYY`,
  `~YYYY`? Recommendation: ISO-only with warn-and-skip on fuzzy.
- §5.E1: when re-applying after stripping a lifecycle prefix, do we want the
  re-applied tags to carry a marker so the rest of the importer can know the
  edge is non-operational?
- §5.C1: bidi-edge pair is the correct representation; remaining question is
  whether the importer should emit bidi pairs *only* for the
  bidirectional `lanes=1` case, or also for bidirectional ways with
  `width < 2 × default` and no explicit `lanes` tag (narrow carriageway
  implies the same shared geometry).
- §3.4: should `surface=*` / `traffic_calming=*` speed-capping live behind
  a separate `--osm.repair.infer-speed-from-context` flag, or as
  `--osm.repair=aggressive`?

## 9. Audit references

The detailed per-area gap audits this plan was distilled from are summarized in
the chat history (2026-05-12 session). Key findings:

- **Access**: only `access=no` parsed; `motor_vehicle`, `hgv`, `taxi`,
  `motorcycle`, `moped` unparsed; per-lane access only for
  `vehicle/bus/psv/bicycle`.
- **Lane arithmetic**: `lanes=3;4` takes minimum (`:2170–2192`); no validation
  `lanes:fwd+lanes:bwd == lanes`; `oneway=yes + lanes:backward` not warned;
  turn:lanes count never reconciled with `lanes` except at apply time then
  ignored; no direction-flip detection.
- **Maxspeed**: no per-lane, no conditional, no per-vehicle, no variable, no
  advisory. Per-direction works. ~60 country implicit values handled
  (`:1778–1855`).
- **Lifecycle**: `highway=construction` discarded by typemap; `highway=proposed`
  has no typemap entry; lifecycle prefixes (`construction:highway=*` etc.)
  silently dropped by key allowlist (`:1913–1956`); no `start_date`/`end_date`
  parsing; no `--osm.date` option.
- **Width**: `lanes=1` on a bidirectional way halves width if no width tag
  (`:753–767`) — see C1 for why this is a bug, not a feature.

Recent OSM-import work (per `docs/web/docs/ChangeLog/Changes_in_2024_releases.md`):
units in distances/speeds (#14885), out-of-order warning (#14892), trolleybus
(#14932), edge widths import + `--ignore-widths` (#4392), restricted
turn-lane info (#14476), `access=no` import (#14650).
