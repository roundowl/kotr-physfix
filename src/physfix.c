/*
 * KoTR_PhysFix - collision fixes for King of the Road / Hard Truck 2 (GOG 1.3)
 *
 * An ASI plugin (load it with the Ultimate ASI Loader, e.g. the one shipped with KoTR Modern Patch).
 * Target: GOG 1.3 king.exe, sha1 295d134d103bf91b8467d28dc9bf74d90193c5bc. Every patch site is
 * checked byte for byte before it is touched; a different exe is left alone (see the log).
 *
 * How a moving body steps (per body, per physics step; MoveItem driver 0x583a00):
 *   vtbl+0x64 0x581b40  save the start pose: translation -> +0x7d8, reference point -> +0x7cc
 *   vtbl+0x68 0x581c80  integrate the proposed end pose (matrix +0x6a0, translation +0x6c4)
 *   vtbl+0x6c 0x58d990  depenetrate from last step's contact, then collision query (vtbl+0x44):
 *                       hit flags -> +0x688, fraction of the step that is free -> +0x5b0
 *   no hit:   vtbl+0x70 commit the step
 *   hit:      vtbl+0x78 MoveItem_2_3 / +0x7c collision handling (moves by the fraction, restores
 *             the last good pose, applies the impulse response)
 * Body-vs-body contacts come from 0x589720(a, M, contacts, b, sign), called twice from the moving
 * body's query 0x58a530: (neighbour, self, -1) at 0x58a736 and (self, neighbour, +1) at 0x58a7e6.
 * It sweeps each box node of `a` from start to end pose against `b`'s surface.
 *
 * Fix 1 - toi clamp (0x58a4a6). That generator stores time of impact
 *   toi = (|contact - start| - 0.01) / |motion| without the clamp to 1 that every other generator
 *   applies. A slow body gets toi in the hundreds (measured up to 2e7); the earliest toi scales the
 *   step fraction, so the body was allowed to travel many times its velocity into the other one.
 *
 * Fix 2 - contact point guard (MoveItem_2_3 entry, 0x582310). A non-finite contact point, or one
 *   more than 50 m from the body, is replaced with the body's position instead of letting
 *   Check_V abort the game with "FATAL dynamic error ... MoveItem_2_3 #1 MI.col.p".
 *   Seen in the wild: col.p = (-1771, 480377, 2682821) on a parked truck. Root cause not found.
 *
 * Fix 3 - wedged / frozen / merged vehicles. Three cooperating hooks, on together or not at all.
 *   The bug: a node that already touches the other body at the start of a step (mm-level contact:
 *   resting against or sliding along something) hits at the segment start, so toi = 0 and the
 *   whole step is cancelled, whether the node moves deeper, slides along the surface or pulls
 *   away. Every step with a contact also goes through the collision branch, which restores the
 *   pose. And the query's depenetration switch has cases for world contacts (hit types 1..4) but
 *   none for body contacts (bit 8), so overlapping bodies are never pushed apart. Result: vehicles
 *   freeze after a side hit, wedge against each other, or sit merged spinning their wheels.
 *   a) contact_fix (0x58a4c4, as each contact is stored): for a touching (toi 0) contact
 *      - exchange the approaching normal velocity between the two bodies, split by mass
 *        (perfectly inelastic, plus a small separating speed when overlapping), through the
 *        body's own set_V (vtbl+0x90; vehicles keep their real velocity in a sub-object);
 *      - compute how far the node actually moves into the surface over this step, rotation
 *        included, and remove exactly that from the step's translation, then mark the contact
 *        non-blocking (toi 1). Removing it matters: the node test only sees a node ENTERING a
 *        surface, so a node that leaks slightly inside becomes invisible and the body creeps in.
 *      - fast impacts (more than FAST_STEP per step into the surface) keep the original block,
 *        so the game's own bounce still happens.
 *   b) driver_filter (0x583a42): a step whose only contacts were body contacts, all resolved by
 *      (a) (fraction still 1), commits through the normal path instead of the restoring one.
 *   c) pre_query (0x58d990 entry, the missing depenetration case): if the body touched another
 *      body last step, its proposed translation is pushed DEPEN_STEP along that contact normal,
 *      scaled by the other body's share of the combined mass.
 *
 * KoTR_PhysFix.ini next to the plugin:
 *   [Fixes] Contacts=1     Fix 3 (Fixes 1 and 2 are always on)
 *   [Debug] Log=1          statistics and stuck-vehicle reports in KoTR_PhysFix.log
 *           TraceKey=0     F12 arms a per-step trace of the next two bodies that touch something
 *                          (KoTR_PhysFix-trace1/2.log, 400 steps each)
 */
#include <windows.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdarg.h>

#define PHYSFIX_VERSION "1.0"

/* patch sites: address, original bytes, where the cave returns to */
#define TOI_SITE   0x0058a4a4  /* fdivr st,st(1) ; fstp dword [esi+edx*4+0x800] */
#define TOI_RESUME 0x0058a4ad
#define COL_SITE   0x00582310  /* MoveItem_2_3 entry: push ebp; mov ebp,esp; and esp,-8 */
#define COL_RESUME 0x00582316
#define CNT_SITE   0x0058a4c4  /* contact stored: inc dword [esi+0xb00] */
#define CNT_RESUME 0x0058a4ca
#define DRV_SITE   0x00583a42  /* driver, after the query: mov eax,[esi+0x688] */
#define DRV_RESUME 0x00583a48
#define PRE_SITE   0x0058d990  /* vehicle query entry: sub esp,0x80 */
#define PRE_RESUME 0x0058d996

static const unsigned char toi_orig[9] = {0xd8,0xf9, 0xd9,0x9c,0x96,0x00,0x08,0x00,0x00};
static const unsigned char col_orig[6] = {0x55, 0x8b,0xec, 0x83,0xe4,0xf8};
static const unsigned char cnt_orig[6] = {0xff,0x86,0x00,0x0b,0x00,0x00};
static const unsigned char drv_orig[6] = {0x8b,0x86,0x88,0x06,0x00,0x00};
static const unsigned char pre_orig[6] = {0x81,0xec,0x80,0x00,0x00,0x00};

/* moving body (MovingItem) fields */
#define MI_INFO    0x140   /* -> struct with char *name at +0xc */
#define MI_V       0x1a0   /* linear velocity (a copy; write it through set_V) */
#define MI_NEW_POS 0x218
#define MI_OLD_POS 0x248
#define MI_START_M 0x224   /* pose at step start: 3x3 then translation, world = A*local + T */
#define MI_FRAC    0x5b0   /* free fraction of the step after the query */
#define MI_COL_P   0x5ec   /* last contact point */
#define MI_COL_N   0x5f8   /* last contact normal */
#define MI_HIT     0x688   /* hit flags: 1..4 world, 8 another body, high bits extra info */
#define MI_END_M   0x6a0   /* proposed end pose */
#define MI_END_T   0x6c4   /* its translation */
#define MI_MASS    0xcc4
#define VT_SET_V   0x90

/* contact set filled by the generators */
#define CS_TOI     0x800   /* float[64] */
#define CS_TOI2    0x900   /* float[64], clamped copy */
#define CS_DEPTH   0xa00   /* float[64] */
#define CS_COUNT   0xb00
#define CS_NORMALS 0x300   /* vec3[64]; points [0] */

/* tuning */
#define COL_P_MAX_DIST 50.0f   /* m from the body before a contact point counts as garbage */
#define FAST_STEP      0.02f   /* m per step into a surface: faster keeps the game's own block */
#define DEPEN_STEP     0.01f   /* m per step pushed apart while touching (x mass share) */
#define DEPEN_GAIN     2.0f    /* separating speed per metre of overlap (1/s) */
#define DEPEN_MAX      0.5f    /* m/s */
#define DEPEN_SLOP     0.01f   /* overlap tolerated without a separating speed */
#define HEAVY_MASS     1e6f    /* mass <= 0 or above this counts as immovable */

static char g_logpath[MAX_PATH];
static int g_log = 1, g_tracekey = 0;

/* statistics */
volatile LONG g_toi_calls, g_toi_clamped, g_col_calls, g_colp_fixed;
volatile LONG g_c_sep, g_c_resolved, g_c_fast, g_drv_cleared, g_pushes, g_stuck;
volatile float g_toi_last;

static void plog(const char *fmt, ...)
{
    if (!g_log) return;
    FILE *f = fopen(g_logpath, "a");
    if (!f) return;
    SYSTEMTIME t; GetLocalTime(&t);
    fprintf(f, "%02d:%02d:%02d ", t.wHour, t.wMinute, t.wSecond);
    va_list ap; va_start(ap, fmt); vfprintf(f, fmt, ap); va_end(ap);
    fputc('\n', f);
    fclose(f);
}

static const char *item_name(unsigned char *item)
{
    unsigned char *info = *(unsigned char **)(item + MI_INFO);
    if (!info || IsBadReadPtr(info + 0xc, 4)) return "?";
    const char *n = *(const char **)(info + 0xc);
    if (!n || IsBadStringPtrA(n, 64)) return "?";
    return n;
}

static float mass_of(unsigned char *item)
{
    float m = *(float *)(item + MI_MASS);
    return m > 0.0f && m < HEAVY_MASS ? m : 0.0f;   /* 0 = immovable */
}

/* ---- per-step trace (debug) ---- */

#define TRACE_STEPS 400
#define TRACE_LOOKBACK 10000   /* ms */
static DWORD g_trace_armed;
static struct { unsigned char *item; int left; FILE *f; } g_tr[2];

static FILE *tr_file(unsigned char *item)
{
    for (int k = 0; k < 2; k++)
        if (g_tr[k].item == item && g_tr[k].left > 0) return g_tr[k].f;
    return NULL;
}

static void trace(unsigned char *item, const char *fmt, ...)
{
    FILE *f = tr_file(item);
    if (!f) return;
    va_list ap; va_start(ap, fmt); vfprintf(f, fmt, ap); va_end(ap);
}

static void trace_poll_key(void)
{
    if (g_tracekey && !g_trace_armed && (GetAsyncKeyState(VK_F12) & 0x8001)) {
        g_trace_armed = GetTickCount();
        plog("trace armed (F12): the next two bodies touching another body (contact begun up to 10 s ago) get traced");
    }
}

static void trace_maybe_start(unsigned char *item)
{
    for (int k = 0; k < 2; k++) {
        if (g_tr[k].item == item) return;
        if (g_tr[k].item) continue;
        char path[MAX_PATH];
        strcpy(path, g_logpath);
        sprintf(strrchr(path, '.'), "-trace%d.log", k + 1);
        g_tr[k].f = fopen(path, "w");
        if (!g_tr[k].f) return;
        g_tr[k].item = item;
        g_tr[k].left = TRACE_STEPS;
        fprintf(g_tr[k].f, "trace of %s %p mass %g\n", item_name(item), item, *(float *)(item + MI_MASS));
        plog("tracing %s %p to %s", item_name(item), item, path);
        return;
    }
}

/* ---- stuck-vehicle report (debug): in collision handling for 3 s while moving < 0.5 m ---- */

#define STUCK_MS   3000
#define STUCK_GAP  300
#define STUCK_DIST 0.5f
#define NTRACK     128
static struct { unsigned char *item; DWORD first, last; LONG calls; float start[3]; int logged; } g_track[NTRACK];

static void track_stuck(unsigned char *item)
{
    DWORD now = GetTickCount();
    const float *pos = (const float *)(item + MI_NEW_POS);
    typeof(&g_track[0]) t = &g_track[((DWORD)item >> 4) % NTRACK];
    if (t->item != item || now - t->last > STUCK_GAP) {
        if (t->item && t->logged)
            plog("  unstuck: %s %p after %.1f s", item_name(t->item), t->item, (t->last - t->first) / 1000.0);
        t->item = item; t->first = now; t->calls = 0; t->logged = 0;
        memcpy(t->start, pos, sizeof t->start);
    }
    t->last = now;
    t->calls++;
    if (t->logged || now - t->first < STUCK_MS) return;
    float dx = pos[0] - t->start[0], dy = pos[1] - t->start[1], dz = pos[2] - t->start[2];
    if (dx * dx + dy * dy + dz * dz > STUCK_DIST * STUCK_DIST) {
        t->first = now; t->calls = 0;
        memcpy(t->start, pos, sizeof t->start);
        return;
    }
    t->logged = 1;
    if (InterlockedIncrement(&g_stuck) <= 100)
        plog("STUCK: %s %p mass %g at (%.2f %.2f %.2f), %ld collision steps in %.1f s, hit %08lx frac %g",
             item_name(item), item, *(float *)(item + MI_MASS), pos[0], pos[1], pos[2],
             t->calls, (now - t->first) / 1000.0, *(DWORD *)(item + MI_HIT), *(float *)(item + MI_FRAC));
}

/* ---- Fix 1: toi clamp ---- */

/* entry: st0 = |motion|, st1 = dist - 0.01; edx = contact index; esi = contact set */
__attribute__((naked)) static void toi_cave(void)
{
    __asm__ volatile (
        "lock incl %[calls]\n\t"
        ".byte 0xd8, 0xf9\n\t"              /* fdivr st,st(1): st0 = toi (original insn; gas reverses the mnemonic) */
        "fld1\n\t"
        ".byte 0xdb, 0xf1\n\t"              /* fcomi st,st(1): CF = (1 < toi) or unordered */
        "jae 1f\n\t"
        "fxch %%st(1)\n\t"
        "fstps %[last]\n\t"                 /* record and pop toi: st0 = 1 */
        "lock incl %[count]\n\t"
        "jmp 2f\n"
        "1:\n\t"
        "fstp %%st(0)\n"                    /* drop the 1.0: st0 = toi */
        "2:\n\t"
        "fstps 0x800(%%esi,%%edx,4)\n\t"    /* original store */
        "push %[resume]\n\t"
        "ret\n\t"
        : [last] "=m"(g_toi_last), [count] "+m"(g_toi_clamped), [calls] "+m"(g_toi_calls)
        : [resume] "i"(TOI_RESUME));
}

/* ---- Fix 2: contact point guard ---- */

static int finite3(const float *v)
{
    for (int i = 0; i < 3; i++)
        if (!(v[i] == v[i]) || v[i] > 1e7f || v[i] < -1e7f) return 0;
    return 1;
}

static void __cdecl col_check(unsigned char *item)
{
    InterlockedIncrement(&g_col_calls);
    if (g_log) track_stuck(item);
    float *p = (float *)(item + MI_COL_P), *n = (float *)(item + MI_COL_N);
    const float *pos = (const float *)(item + MI_NEW_POS), *old = (const float *)(item + MI_OLD_POS);
    float dx = p[0] - pos[0], dy = p[1] - pos[1], dz = p[2] - pos[2];
    if (finite3(p) && dx * dx + dy * dy + dz * dz <= COL_P_MAX_DIST * COL_P_MAX_DIST) return;
    if (InterlockedIncrement(&g_colp_fixed) <= 200)
        plog("bad contact point fixed: %s %p hit %08lx p=(%g %g %g) n=(%g %g %g) pos=(%g %g %g) old=(%g %g %g)",
             item_name(item), item, *(DWORD *)(item + MI_HIT), p[0], p[1], p[2], n[0], n[1], n[2],
             pos[0], pos[1], pos[2], old[0], old[1], old[2]);
    memcpy(p, pos, 3 * sizeof(float));
    if (!finite3(n) || n[0] * n[0] + n[1] * n[1] + n[2] * n[2] > 4.0f) {
        n[0] = 0.0f; n[1] = 0.0f; n[2] = 1.0f;
    }
}

/* entry of MoveItem_2_3 (thiscall): ecx = item */
__attribute__((naked)) static void col_cave(void)
{
    __asm__ volatile (
        "pushal\n\t"
        "push %%ecx\n\t"
        "call %P[f]\n\t"
        "add $4, %%esp\n\t"
        "popal\n\t"
        "push %%ebp\n\t"                    /* original prologue */
        "mov %%esp, %%ebp\n\t"
        "and $-8, %%esp\n\t"
        "push %[resume]\n\t"
        "ret\n\t"
        :: [f] "i"(col_check), [resume] "i"(COL_RESUME));
}

/* ---- Fix 3a: resolve touching body contacts ---- */

/* what a body touched during its current query: drives the push in pre_query (3c) */
struct touch { unsigned char *item; DWORD since; float n[3]; float share; int touched, touching; };
#define NTOUCH 256
static struct touch g_touch[NTOUCH];

static struct touch *touch_of(unsigned char *item)
{
    struct touch *t = &g_touch[((DWORD)item >> 4) % NTOUCH];
    if (t->item != item) { memset(t, 0, sizeof *t); t->item = item; }
    return t;
}

typedef void (__attribute__((thiscall)) *set_v_fn)(void *self, const float *v, const char *tag);
static void set_v(unsigned char *obj, const float *v)
{
    set_v_fn f = *(set_v_fn *)(*(unsigned char **)obj + VT_SET_V);
    f(obj, v, "KoTR_PhysFix");
}

/* how far the contact node C moves along n over this step, rotation included:
 *   d = A_end * A_start^T * (C - T_start) + T_end - C,  A[r][c] = M[r + 3c], T = M[9..11] */
static float node_motion_n(unsigned char *item, const float *C, const float *n)
{
    const float *m0 = (const float *)(item + MI_START_M), *m1 = (const float *)(item + MI_END_M);
    float r[3] = {C[0] - m0[9], C[1] - m0[10], C[2] - m0[11]}, l[3], d[3];
    for (int c = 0; c < 3; c++)
        l[c] = m0[3 * c] * r[0] + m0[3 * c + 1] * r[1] + m0[3 * c + 2] * r[2];
    for (int k = 0; k < 3; k++)
        d[k] = m1[k] * l[0] + m1[k + 3] * l[1] + m1[k + 6] * l[2] + m1[9 + k] - C[k];
    return d[0] * n[0] + d[1] * n[1] + d[2] * n[2];
}

/* called as contact `i` is stored; ebp = frame of 0x589720(a, M, set, b, sign) */
static void __cdecl contact_fix(unsigned char *set, unsigned char *ebp)
{
    int i = *(int *)(set + CS_COUNT);
    float *toi = (float *)(set + CS_TOI + i * 4);
    if (*toi != 0.0f) return;                    /* not touching at the start: game handles it */

    int sign = *(int *)(ebp + 0x18);
    unsigned char *a = *(unsigned char **)(ebp + 8), *b = *(unsigned char **)(ebp + 0x14);
    unsigned char *item = sign > 0 ? a : b, *other = sign > 0 ? b : a;   /* item = the moving body */
    const float *n = (const float *)(set + CS_NORMALS + i * 0xc);       /* other -> item */
    float nl2 = n[0] * n[0] + n[1] * n[1] + n[2] * n[2];
    if (!(nl2 > 0.81f && nl2 < 1.21f)) return;

    float mi = mass_of(item), mo = mass_of(other);
    if (mi == 0.0f && mo == 0.0f) return;

    /* remember the contact for the push (3c): push away from the heaviest body touched */
    struct touch *t = touch_of(item);
    float share = (mi > 0.0f && mo > 0.0f) ? mo / (mi + mo) : 1.0f;
    if (!t->touching || share > t->share) {
        t->share = share;
        memcpy(t->n, n, sizeof t->n);
    }
    t->touching = 1;
    if (!t->since) t->since = GetTickCount();
    /* contacts that began up to TRACE_LOOKBACK before F12 count too (catch one in progress),
     * but not long-standing ones such as a parked truck resting against its trailer */
    if (g_trace_armed && t->since + TRACE_LOOKBACK >= g_trace_armed) trace_maybe_start(item);

    /* velocity: inelastic exchange of the approaching normal component, split by mass */
    const float *vi = (const float *)(item + MI_V), *vo = (const float *)(other + MI_V);
    float vn = (vi[0] - vo[0]) * n[0] + (vi[1] - vo[1]) * n[1] + (vi[2] - vo[2]) * n[2];
    float depth = *(float *)(set + CS_DEPTH + i * 4);
    float target = depth > DEPEN_SLOP ? fminf((depth - DEPEN_SLOP) * DEPEN_GAIN, DEPEN_MAX) : 0.0f;
    if (vn < target) {
        float dv = target - vn;
        float si = mo == 0.0f ? 1.0f : mi == 0.0f ? 0.0f : mo / (mi + mo);
        float nvi[3], nvo[3];
        for (int k = 0; k < 3; k++) {
            nvi[k] = vi[k] + n[k] * dv * si;
            nvo[k] = vo[k] - n[k] * dv * (1.0f - si);
        }
        if (si > 0.0f) set_v(item, nvi);
        if (si < 1.0f) set_v(other, nvo);
    }

    /* position: take the into-surface part out of this step, or leave fast impacts to the game */
    float dn = node_motion_n(item, (const float *)(set + i * 0xc), n);
    const char *what = "pass";
    if (dn < -FAST_STEP) {
        what = "block (fast)";
        InterlockedIncrement(&g_c_fast);
    } else {
        if (dn < 0.0f) {
            float *end = (float *)(item + MI_END_T);
            for (int k = 0; k < 3; k++) end[k] -= n[k] * dn;
            what = "project";
        }
        *toi = 1.0f;
        *(float *)(set + CS_TOI2 + i * 4) = 1.0f;
        InterlockedIncrement(vn < target ? &g_c_resolved : &g_c_sep);
    }
    trace(item, "  contact c%d %s with %s m=%g: n (%+.3f %+.3f %+.3f) depth %+.4f vn %+.3f dn %+.4f -> %s\n",
          i, sign > 0 ? "own node" : "their node", item_name(other), *(float *)(other + MI_MASS),
          n[0], n[1], n[2], depth, vn, dn, what);
}

__attribute__((naked)) static void cnt_cave(void)
{
    __asm__ volatile (
        "pushal\n\t"
        "pushfl\n\t"
        "push %%ebp\n\t"
        "push %%esi\n\t"
        "call %P[f]\n\t"
        "add $8, %%esp\n\t"
        "popfl\n\t"
        "popal\n\t"
        "incl 0xb00(%%esi)\n\t"             /* original insn */
        "push %[resume]\n\t"
        "ret\n\t"
        :: [f] "i"(contact_fix), [resume] "i"(CNT_RESUME));
}

/* ---- Fix 3b: commit fully resolved steps normally ---- */

static void __cdecl driver_filter(unsigned char *item)
{
    trace_poll_key();
    DWORD hit = *(DWORD *)(item + MI_HIT);
    if (tr_file(item)) {
        const float *p = (const float *)(item + MI_NEW_POS);
        trace(item, "  query: hit %08lx frac %.4f  pos (%.4f %.4f %.4f)\n", hit, *(float *)(item + MI_FRAC), p[0], p[1], p[2]);
        for (int k = 0; k < 2; k++)
            if (g_tr[k].item == item && --g_tr[k].left == 0) { fclose(g_tr[k].f); plog("trace %d done", k + 1); }
    }
    if ((hit & 0xff) != 8 || *(float *)(item + MI_FRAC) < 0.999f) return;
    *(DWORD *)(item + MI_HIT) = 0;
    InterlockedIncrement(&g_drv_cleared);
}

__attribute__((naked)) static void drv_cave(void)
{
    __asm__ volatile (
        "pushal\n\t"
        "push %%esi\n\t"
        "call %P[f]\n\t"
        "add $4, %%esp\n\t"
        "popal\n\t"
        "mov 0x688(%%esi), %%eax\n\t"       /* original insn */
        "push %[resume]\n\t"
        "ret\n\t"
        :: [f] "i"(driver_filter), [resume] "i"(DRV_RESUME));
}

/* ---- Fix 3c: the missing depenetration case for body contacts ---- */

static void __cdecl pre_query(unsigned char *item)
{
    struct touch *t = touch_of(item);
    t->touched = t->touching;                /* result of the previous query */
    if (!t->touching) t->since = 0;
    t->touching = 0;
    float push[3] = {0, 0, 0};
    if (t->touched) {
        float *end = (float *)(item + MI_END_T);
        for (int k = 0; k < 3; k++) {
            push[k] = t->n[k] * DEPEN_STEP * t->share;
            end[k] += push[k];
        }
        InterlockedIncrement(&g_pushes);
    }
    if (tr_file(item)) {
        const float *s = (const float *)(item + MI_START_M + 0x24), *e = (const float *)(item + MI_END_T);
        trace(item, "STEP start (%.4f %.4f %.4f) move (%+.4f %+.4f %+.4f) push (%+.4f %+.4f %+.4f) prev hit %08lx\n",
              s[0], s[1], s[2], e[0] - s[0], e[1] - s[1], e[2] - s[2], push[0], push[1], push[2],
              *(DWORD *)(item + MI_HIT));
    }
}

__attribute__((naked)) static void pre_cave(void)
{
    __asm__ volatile (
        "pushal\n\t"
        "push %%ecx\n\t"
        "call %P[f]\n\t"
        "add $4, %%esp\n\t"
        "popal\n\t"
        "sub $0x80, %%esp\n\t"              /* original insn */
        "push %[resume]\n\t"
        "ret\n\t"
        :: [f] "i"(pre_query), [resume] "i"(PRE_RESUME));
}

/* ---- setup ---- */

static DWORD WINAPI log_thread(LPVOID p)
{
    LONG seen = 0;
    for (;;) {
        Sleep(10000);
        LONG n = g_col_calls + g_c_sep + g_c_resolved + g_c_fast;
        if (n == seen) continue;
        seen = n;
        plog("collision steps %ld, bad contact points fixed %ld, stuck reports %ld | toi clamped %ld of %ld (last %g) | "
             "touching contacts: separating %ld, resolved %ld, fast (game) %ld | steps committed normally %ld, pushes %ld",
             g_col_calls, g_colp_fixed, g_stuck, g_toi_clamped, g_toi_calls, (double)g_toi_last,
             g_c_sep, g_c_resolved, g_c_fast, g_drv_cleared, g_pushes);
    }
    return 0;
}

static BOOL site_ok(DWORD site, const unsigned char *orig, size_t len)
{
    return memcmp((void *)site, orig, len) == 0;
}

static void patch_jmp(DWORD site, size_t len, void *dest)
{
    DWORD old;
    VirtualProtect((void *)site, len, PAGE_EXECUTE_READWRITE, &old);
    unsigned char *p = (unsigned char *)site;
    p[0] = 0xe9;
    *(DWORD *)(p + 1) = (DWORD)dest - (site + 5);
    memset(p + 5, 0x90, len - 5);
    VirtualProtect((void *)site, len, old, &old);
    FlushInstructionCache(GetCurrentProcess(), (void *)site, len);
}

BOOL WINAPI DllMain(HINSTANCE inst, DWORD reason, LPVOID res)
{
    if (reason != DLL_PROCESS_ATTACH) return TRUE;
    DisableThreadLibraryCalls(inst);

    char ini[MAX_PATH];
    GetModuleFileNameA(inst, g_logpath, MAX_PATH);
    char *dot = strrchr(g_logpath, '.');
    if (dot) strcpy(dot, ".log");
    strcpy(ini, g_logpath);
    strcpy(strrchr(ini, '.'), ".ini");
    int contacts = GetPrivateProfileIntA("Fixes", "Contacts", 1, ini);
    g_log = GetPrivateProfileIntA("Debug", "Log", 1, ini);
    g_tracekey = GetPrivateProfileIntA("Debug", "TraceKey", 0, ini);

    BOOL ok_toi = site_ok(TOI_SITE, toi_orig, sizeof toi_orig);
    BOOL ok_col = site_ok(COL_SITE, col_orig, sizeof col_orig);
    BOOL ok_contacts = site_ok(CNT_SITE, cnt_orig, sizeof cnt_orig) &&
                       site_ok(DRV_SITE, drv_orig, sizeof drv_orig) &&
                       site_ok(PRE_SITE, pre_orig, sizeof pre_orig);
    if (ok_toi) patch_jmp(TOI_SITE, sizeof toi_orig, (void *)toi_cave);
    if (ok_col) patch_jmp(COL_SITE, sizeof col_orig, (void *)col_cave);
    if (contacts && ok_contacts) {
        patch_jmp(CNT_SITE, sizeof cnt_orig, (void *)cnt_cave);
        patch_jmp(DRV_SITE, sizeof drv_orig, (void *)drv_cave);
        patch_jmp(PRE_SITE, sizeof pre_orig, (void *)pre_cave);
    }
    plog("KoTR_PhysFix %s loaded: toi clamp %s, contact point guard %s, contact fix %s",
         PHYSFIX_VERSION, ok_toi ? "on" : "FAILED", ok_col ? "on" : "FAILED",
         !contacts ? "off (ini)" : ok_contacts ? "on" : "FAILED");
    if (!ok_toi || !ok_col || (contacts && !ok_contacts))
        plog("unexpected bytes at a patch site: this is not the GOG 1.3 king.exe, or another mod patches the same code");
    if (g_log && (ok_toi || ok_col)) CreateThread(NULL, 0, log_thread, NULL, 0, NULL);
    return TRUE;
}
