# How KoTR PhysFix works

Everything here refers to the GOG 1.3 `king.exe` (SHA-1 `295d134d103bf91b8467d28dc9bf74d90193c5bc`,
image base `0x400000`). Offsets are from the start of the object or structure named. Names are ours;
the game ships with no symbols, only its own error strings.

## The step pipeline

Every moving body (vehicle, trailer, barrel, crate) is a `MovingItem`. Each physics step it runs the
driver `0x583a00`, which loops over virtual calls:

| vtable | function | what it does |
|---|---|---|
| `+0x64` | `0x581b40` | save the start pose: translation to `+0x7d8`, a reference point to `+0x7cc` |
| `+0x68` | `0x581c80` | integrate the **proposed end pose** into the matrix at `+0x6a0` (translation `+0x6c4`) |
| `+0x6c` | `0x58d990` (vehicles), `0x581ea0` (others) | depenetrate from last step's contact, then the collision query `vtbl+0x44`, storing hit flags in `+0x688` and the free fraction of the step in `+0x5b0` |
| `+0x70` | | no hit: commit the step |
| `+0x78`, `+0x7c` | `MoveItem_2_3` `0x582310`, `0x583310` | hit: move by the fraction, restore the last good pose, impulse response |

Poses are 12 floats: a 3×3 matrix then the translation, `world = A·local + T`, with
`A[r][c] = M[r + 3c]`. The start pose is at `+0x224` (its translation `+0x248` is what the game's own
debug output calls `old_pos`).

The collision query for vehicles (`0x58d6f0` → `Collide_General` `0x58a820`) runs three contact
generators into one contact set, takes the contact with the smallest time of impact (TOI), and
multiplies the step fraction by it. Hit flags: 1–4 = world geometry, 8 = another body, plus high bits.

**Body-vs-body contacts** come from `0x589720(a, M, set, b, sign)`, called twice from the moving
body's `0x58a530`:

| call site | a | b | sign | sweeps |
|---|---|---|---|---|
| `0x58a736` | neighbour | moving body | −1 | the neighbour's box nodes against the moving body |
| `0x58a7e6` | moving body | neighbour | +1 | the moving body's box nodes against the neighbour |

Each box node is swept as a segment from its start-pose position to its end-pose position against the
other body's surface. Contact set layout: points `+0x000` (64 × vec3), normals `+0x300` (pointing from
the neighbour to the moving body), TOI `+0x800`, clamped TOI `+0x900`, a depth-like value `+0xa00`,
count `+0xb00`, and the call's sign in `+0xb04`.

## Bug 1: unclamped TOI (`0x58a4a6`)

`0x589720` stores `toi = (|contact − start| − 0.01) / |motion|` without the clamp to 1 that the other
generators, and its own sibling value at `+0x900`, apply. For a slow body `|motion|` is tiny and the
TOI comes out in the hundreds (measured up to 2·10⁷). Because the step fraction is multiplied by the
smallest TOI, a body could be allowed to move many times its own velocity in one step, straight into
the other one. **Fix:** clamp to 1 (`toi_cave`).

## Bug 2: garbage contact point (`MoveItem_2_3`, `0x582310`)

Rarely, the contact point handed to `MoveItem_2_3` is nonsense, e.g. `(-1771, 480377, 2682821)` for a
parked truck; the first thing that function does is `Check_V(col.p, 10000)`, which raises
`FATAL dynamic error … MoveItem_2_3 #1 MI.col.p` and quits. Root cause not found. **Fix:** on entry,
a non-finite point, or one more than 50 m from the body, is replaced with the body's position
(`col_check`).

## Bug 3: wedged, frozen and merged vehicles

Three faults combine:

1. **No direction check on touching contacts.** A node that already touches the other body at the
   start of the step (millimetre contact: resting against or sliding along it) hits at the segment
   start, so its TOI is 0 and the whole step is cancelled, whether that node is moving deeper,
   sliding along the surface or pulling away. Measured, truck sliding along a barrel: contact depth
   2–6 mm, relative speed 0.85 m/s, almost all of it tangential; the truck stayed frozen for 20 s
   at ~300 collision steps per second.
2. **Every step with a contact is undone.** The driver sends any step with a hit through the
   collision branch, which restores the pose even when the free fraction is 1.
3. **The missing depenetration case.** `0x58d990` begins with
   `switch (previous hit) { case 1: case 2: case 3: case 4: push the proposed pose out of the last contact plane }`.
   Body contacts (8, usually with high flag bits such as `0x08000008`) have no case, so two
   overlapping bodies are never pushed apart.

The fix is three hooks that only make sense together (`[Fixes] Contacts=1`):

- **`contact_fix`** at `0x58a4c4`, as each contact is stored. For a touching (TOI 0) contact:
  - pick the moving body by the call's `sign` (see the table above);
  - exchange the approaching part of the relative normal velocity between the two bodies, split by
    mass (perfectly inelastic, plus a small separating speed when overlapping). This goes through the
    body's own `set_V` (`vtbl+0x90`): vehicles keep their real velocity in a physics sub-object at
    `+0x5454` and overwrite the `+0x1a0` copy every step, so writing `+0x1a0` directly does nothing;
  - compute how far the node actually moves into the surface this step, rotation included:
    `d = A_end · A_startᵀ · (C − T_start) + T_end − C`, and subtract exactly the into-surface part
    from the step's translation, then mark the contact non-blocking (TOI 1).
    Subtracting it, rather than just letting the step through, matters: the node test only detects a
    node *entering* a surface, so a node that leaks slightly inside becomes invisible to it and the
    body creeps in unseen;
  - a fast impact (more than 2 cm per step into the surface) keeps the game's original handling, so
    hard crashes still bounce.
- **`driver_filter`** at `0x583a42`: a step whose only contacts were body contacts, all resolved
  above (fraction still 1), has its hit flag cleared so it commits through the normal path.
- **`pre_query`** at the entry of `0x58d990`, the missing case: if the body touched another body in
  its previous query, its proposed translation is pushed 1 cm along that contact normal, scaled by
  the other body's share of the combined mass (a barrel barely moves a truck).

## Patch sites

| site | original bytes | hook |
|---|---|---|
| `0x58a4a4` | `d8 f9 d9 9c 96 00 08 00 00` | `toi_cave` (Bug 1) |
| `0x582310` | `55 8b ec 83 e4 f8` | `col_cave` (Bug 2) |
| `0x58a4c4` | `ff 86 00 0b 00 00` | `cnt_cave` → `contact_fix` |
| `0x583a42` | `8b 86 88 06 00 00` | `drv_cave` → `driver_filter` |
| `0x58d990` | `81 ec 80 00 00 00` | `pre_cave` → `pre_query` |

Each site is compared byte for byte before patching; the three contact hooks go in together or not
at all.

## Things we tried that did not work

Kept here so nobody repeats them:

- Only un-blocking touching contacts: the step was free (fraction 1) and the body still did not move,
  because of fault 2.
- Velocity changes written to `+0x1a0`: no effect (see `set_V` above). Clearing the hit on top of that
  made body collisions pass straight through.
- Deciding block/pass from velocity, or blocking or passing the whole step: either slow bodies creep
  into each other (the node test goes blind once a node is inside) or they wedge again.
- Treating the second generator call (sign +1) like the first: half of all corrections were applied to
  the neighbour with a reversed normal, and the pushes cancelled out.

## Tools

Ghidra 12 headless and radare2. If you load `king.exe` into Ghidra, name `0x61d4f0` `__alloca_probe`
**before** auto-analysis, or every large physics function loses its arguments.
