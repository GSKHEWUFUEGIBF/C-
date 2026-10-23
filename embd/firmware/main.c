/* ============================================================================
 *  main.c  -  Dual-Stage Grid Power Quality Improvement System
 *             Stage 1: APFC with open-drain drive of 5 V active-LOW relays
 *             Stage 2: Active Harmonic Filter (TB6612FNG, 80 kHz PWM)
 *  Target   : STM32F401RE (NUCLEO-F401RE), pure CMSIS, zero HAL libraries
 *
 *  RELAY WIRING (5 V active-LOW modules on PB0 / PB1, SSR on PB2;
 *  any pin can be switched to an SSR with RELAY_SSR_MASK)
 *    Module VCC -> Nucleo 5V, GND -> GND, IN -> PB0 / PB1 directly.
 *    PB0/PB1 are OPEN-DRAIN: the pin only ever sinks current (relay ON) or
 *    floats (relay OFF). When it floats, the module's own resistor pulls IN
 *    up to 5 V, so the opto LED gets 0 V across it and the relay really
 *    releases. PB0/PB1 must be 5 V tolerant (FT) for this; do NOT enable the
 *    internal pull-up (it would clamp IN to 3.3 V again).
 *
 *  AHF: SELECTIVE HARMONIC CONTROL (this revision)
 *    Every 100 ms window the DFT gives one phasor per harmonic h = 2..15 for
 *      I  - the current to be cleaned (ACS712 #1)
 *      A  - the current the AHF actually injects (ACS712 #2)
 *    Each harmonic has its own closed loop:
 *      required  R_h = -I_h                 (cancel that harmonic)
 *      error     E_h = R_h - A_h
 *      duty      U_h += AHF_MU * E_h / G_h
 *    G_h (amps per unit duty, complex) is the bridge + coupling-inductor gain
 *    at that harmonic. It starts from a rough model (AHF_VDC, AHF_L_H,
 *    AHF_R_OHM) and is then measured on line as A_h / U_h, so the inductor's
 *    1/h attenuation, its ~90 deg lag and the PWM delay are corrected per
 *    harmonic instead of being guessed. The output table is synthesised from
 *    the U_h phasors only, so noise and content above h15 are not amplified.
 *
 *    The sampling clock (TIM2) is phase-locked to the grid voltage, so one
 *    cycle is always exactly 200 samples. Without this, a 50.1 Hz grid makes
 *    h15 drift ~54 deg per window against a fixed 50.000 Hz table.
 *    TIM4 (PWM) is reset by every TIM2 update, so the 8 PWM periods per sample
 *    stay aligned with the ADC sampling instant and the PWM ripple cannot alias
 *    into the measured harmonics.
 *
 *    A_SIGN must make sensor A read +ve when the AHF pushes current in the
 *    same direction sensor I reads +ve. If grid THD rises when the AHF is
 *    ACTIVE, flip A_SIGN.
 *
 *  Telemetry changes (backwards compatible, fields appended / reused)
 *    $M  ... ,freq                 measured grid frequency [Hz]
 *    $A  active,apk,ref_pk,duty_pk,resid_pct
 *    $P  ... ,lock,reason,wait_lock      grid lock, why the bank holds,
 *                                        caps waiting for the lock
 *    $W,C                          I + A (measured), was an ideal sine
 * ========================================================================== */

#include "stm32f401xe.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#pragma GCC optimize ("O2")

/* ------------------------------------------------------------------------- */
/*  Clocks & Sampling Constants                                              */
/* ------------------------------------------------------------------------- */
#define SYSCLK_HZ          64000000u
#define APB1_TIM_HZ        64000000u
#define APB1_HZ            32000000u
#define ADC_CLK_HZ         16000000u

#define FS_HZ              10000u                         /* nominal          */
#define F_GRID_HZ          50.0f
#define TWO_PI             6.28318531f
#define N_WIN              1000u
#define N_CYC              200u
#define CYC_PER_WIN        (N_WIN / N_CYC)
#define N_CH               3u
#define DMA_LEN            (2u * N_WIN * N_CH)
#define MAX_H              15u
#define WIN_MS             (1000u * N_WIN / FS_HZ)       /* 100 ms per window */

#define SLOT_I             0u
#define SLOT_V             1u
#define SLOT_A             2u

#define TIM2_N_NOM         (APB1_TIM_HZ / FS_HZ)          /* 6400 ticks       */

#define ADC_SMP_CYCLES     480u
#define ADC_SMP_CODE       7u
#define ADC_SLOT_S         ((float)(ADC_SMP_CYCLES + 12u) / (float)ADC_CLK_HZ)
#define ADC_CLIP_LO        8u
#define ADC_CLIP_HI        4087u

/* Grid lock (TIM2 sample clock follows the grid) */
#define PLL_F_MIN          47.5f
#define PLL_F_MAX          52.5f
#define PLL_KP_HZ          0.4f       /* Hz per rad of phase error           */
#define PLL_F_ALPHA        0.3f       /* smoothing of the reported frequency */
#define PLL_PHASE_SET      0.0f       /* V1 phase held at the window start   */

/* Sensor Calibration */
#define ADC_V_PER_LSB      (3.3f / 4095.0f)
#define ACS712_V_PER_A     0.185f
#define ACS_DIV            1.0f
#define ZMPT_V_PER_V       1014.0f
#define I_SIGN             1.0f
#define V_SIGN             1.0f
#define A_SIGN             1.0f
#define ZMPT_LAG_DEG       0.0f

#define V_DEADBAND         5.0f
#define I_DEADBAND         0.03f
#define A_DEADBAND         0.05f      /* sensor A rms below this is shown as 0 */
#define A_LPF_HZ           1000.0f    /* 2nd-order Butterworth low-pass on
                                       * sensor A (h15 = 750 Hz passes; its
                                       * gain/phase is divided back out per
                                       * harmonic, so the AHF loop is exact)  */

#define K_I   (I_SIGN * ADC_V_PER_LSB / (ACS712_V_PER_A * ACS_DIV))
#define K_A   (A_SIGN * ADC_V_PER_LSB / (ACS712_V_PER_A * ACS_DIV))
#define K_V   (V_SIGN * ADC_V_PER_LSB * ZMPT_V_PER_V)

/* APFC Constants */
#define PF_TARGET          0.99f      /* switch whenever a step improves PF   */
#define C_DEADBAND_UF      0.75f
#define CAP_RECLOSE_MS     5000u      /* min OFF time before a cap re-closes.
                                       * Without bleeder resistors the cap is
                                       * still charged after this; the
                                       * point-on-wave closing (RELAY_SYNC)
                                       * is what protects the contacts.       */
#define APFC_V_MIN         10.0f
#define APFC_P_MIN         0.5f
#define PFC_SETTLE_WIN     2u
#define TELEM_WIN          5u

/* Set to 1 to click R1, R2, R3 ON/OFF once at boot (bench test, mains off) */
#define RELAY_SELFTEST     0u

/* AHF Constants */
#define PWM_PER_SAMPLE     8u
#define PWM_PERIOD_NOM     (TIM2_N_NOM / PWM_PER_SAMPLE)  /* 800 -> 80 kHz    */
#define AHF_MOD_DEPTH      0.92f      /* max |duty| of the synthesised table  */
#define AHF_MIN_HARM_A     0.02f      /* below this target peak, do not drive */
#define AHF_PK_ALPHA       0.3f
#define AHF_BOOT_ACTIVE    1u

#define AHF_HARM_MASK      0xFFFCu    /* bit h set = cancel harmonic h (2..15) */
#define AHF_SENSE_GRID     0u         /* 0: sensor I sees the load current
                                       *    (upstream of the injection point)
                                       * 1: sensor I sees the grid current
                                       *    (downstream, already compensated) */
#define AHF_INJ_RATIO      1.0f       /* PCC amps per sensor-A amp (coupling
                                       * transformer ratio, 1 if direct)      */
#define AHF_VDC            12.0f      /* initial plant model only: TB6612 VM  */
#define AHF_L_H            0.010f     /*   coupling inductance [H]            */
#define AHF_R_OHM          1.0f       /*   coupling resistance [ohm]          */
#define AHF_DELAY_S        (1.5f / (float)FS_HZ)          /* PWM + update     */
#define AHF_MU             0.5f       /* loop gain per update, 0 < MU <= 1    */
#define AHF_MU_PROBE       0.15f      /* gain until G_h has been measured     */
#define AHF_ID_RATE        0.25f      /* plant estimate update rate           */
#define AHF_ID_MIN_DUTY    0.01f      /* identify G_h only when |U_h| > this  */
#define AHF_LEAK           0.002f     /* slow forgetting of unused harmonics  */

#define UART_BAUD          115200u
#define WAVE_DECIM         2u
#define TX_BUF_SZ          5632u
#define RX_LINE_MAX        32u

_Static_assert(APB1_TIM_HZ / (N_CYC * 50u) / PWM_PER_SAMPLE <= 65535u, "TIM4 ARR is 16 bit");
_Static_assert(APB1_TIM_HZ / PWM_PERIOD_NOM <= 100000u, "TB6612FNG max PWM is 100 kHz");
_Static_assert((uint64_t)(ADC_SMP_CYCLES + 12u) * N_CH * N_CYC * 53u < ADC_CLK_HZ,
               "ADC scan does not fit in one sample period at PLL_F_MAX");

/* ------------------------------------------------------------------------- */
/*  Relay pins                                                               */
/* ------------------------------------------------------------------------- */
/* PB0 = 2.5 uF, PB1 = 10 uF, PB2 = 20 uF. Each output is one of:
 *   relay module : 5 V active-LOW, open-drain. ON = sink LOW, OFF = float.
 *   SSR          : active-HIGH, push-pull.     ON = HIGH,     OFF = LOW.
 * Bit i of RELAY_SSR_MASK set = PB<i> drives an SSR. Zero-cross SSRs are
 * strongly recommended for capacitors: they close near 0 V, so a discharged
 * cap draws almost no inrush (a mechanical relay closing near the 325 V peak
 * sees hundreds of amps and its contacts weld).                            */
#define RELAY_SSR_MASK     0x4u       /* PB2 = SSR, PB0/PB1 = relay modules   */

/* Switching rules for the capacitor steps (AUTO and manual alike)          */
#define RELAY_MIN_ON_MS    5000u      /* a closed cap stays ON at least this  */
#define RELAY_LOCK_WAIT_MS 2000u      /* no grid lock after this: close anyway */
/*  CAP_RECLOSE_MS (above) is the matching minimum OFF time.                 */

/* Point-on-wave switching for relay modules (not SSRs). With no bleeder the
 * cap keeps the voltage it was disconnected at.
 *  Opening: the contacts part shortly before the +peak. A cap's current is
 *    ~0 there, and if an arc does form it runs on to the current zero, which
 *    is that same +peak - either way the cap is left near +Vpk.
 *  Closing: at the +peak, so the contacts see ~0 V. A cap whose charge is
 *    unknown (never opened by this firmware since boot) is closed at the
 *    rising zero crossing instead: at most Vpk across the contacts whether
 *    it is empty or charged either way (closing at the peak could be 2 Vpk).
 * The PLL holds the voltage peak at sample k = 0 of every cycle.           */
#define RELAY_SYNC         1u         /* 0 = switch immediately (old way)     */
#define RELAY_T_OP_MS      7.0f       /* coil ON  -> contacts closed (typ 5-9)*/
#define RELAY_T_REL_MS     5.0f       /* coil OFF -> contacts open   (typ 2-8)*/
#define RELAY_OPEN_LEAD_MS 2.0f       /* contacts part this long before +peak */
#define RELAY_MS_TO_K(ms)  ((uint32_t)((ms) * (float)FS_HZ / 1000.0f + 0.5f))
#define RELAY_K_PEAK       ((uint32_t)((float)N_CYC - ZMPT_LAG_DEG / 360.0f * (float)N_CYC) % N_CYC)
#define RELAY_K_ON         ((RELAY_K_PEAK + 2u * N_CYC - RELAY_MS_TO_K(RELAY_T_OP_MS)) % N_CYC)
#define RELAY_K_ON_ZERO    ((RELAY_K_PEAK + 2u * N_CYC - N_CYC / 4u - \
                             RELAY_MS_TO_K(RELAY_T_OP_MS)) % N_CYC)
#define RELAY_K_OFF        ((RELAY_K_PEAK + 2u * N_CYC - RELAY_MS_TO_K(RELAY_OPEN_LEAD_MS) - \
                             RELAY_MS_TO_K(RELAY_T_REL_MS)) % N_CYC)

/* ========================================================================= */
/*  DSP core - no register access (built and simulated on a PC by           */
/*  firmware/test/run_sim.sh, which copies everything between the markers)   */
/* ========================================================================= */
/* DSP-CORE-BEGIN */
typedef struct { float re, im; } cpx_t;

static inline cpx_t c_mk(float re, float im)   { cpx_t z = { re, im }; return z; }
static inline cpx_t c_add(cpx_t a, cpx_t b)    { return c_mk(a.re + b.re, a.im + b.im); }
static inline cpx_t c_sub(cpx_t a, cpx_t b)    { return c_mk(a.re - b.re, a.im - b.im); }
static inline cpx_t c_scale(cpx_t a, float k)  { return c_mk(a.re * k, a.im * k); }
static inline float c_abs2(cpx_t a)            { return a.re * a.re + a.im * a.im; }
static inline cpx_t c_mul(cpx_t a, cpx_t b)
{
    return c_mk(a.re * b.re - a.im * b.im, a.re * b.im + a.im * b.re);
}
static inline cpx_t c_div(cpx_t a, cpx_t b)
{
    const float d = c_abs2(b);
    return c_mk((a.re * b.re + a.im * b.im) / d, (a.im * b.re - a.re * b.im) / d);
}
/* dft() gives x = a cos(h th) + b sin(h th); as a phasor that is a - jb.    */
static inline cpx_t c_phasor(float a, float b) { return c_mk(a, -b); }

static float wrap_pi(float x)
{
    while (x >  3.14159265f) x -= TWO_PI;
    while (x < -3.14159265f) x += TWO_PI;
    return x;
}

static float cos_t[N_CYC], sin_t[N_CYC];
static float tan_phi_tgt;

typedef struct { float b0, b1, b2, a1, a2; } biquad_t;
static biquad_t a_lpf;                  /* sensor A low-pass                  */
static cpx_t    a_lpf_h[MAX_H + 1];     /* its response at each harmonic      */

static int16_t ahf_tab[2][N_CYC];       /* Q15 duty, one cycle, played by ISR */
static cpx_t   ahf_tab_u[2][MAX_H + 1]; /* duty phasors that built each table */
static cpx_t   ahf_u[MAX_H + 1];        /* duty phasor per harmonic (state)   */
static cpx_t   ahf_g[MAX_H + 1];        /* plant gain, amps per unit duty     */
static uint8_t ahf_g_seen[MAX_H + 1];   /* G_h has been measured at least once*/
static float   ahf_resid_pct, ahf_duty_pk;

static float    pll_f_cmd = F_GRID_HZ;  /* sample clock = N_CYC * pll_f_cmd   */
static float    pll_f_est = F_GRID_HZ;  /* measured grid frequency            */
static float    pll_ph_prev;
static uint32_t pll_have_prev;
static uint32_t pll_locked;             /* window start sits on the V peak    */

static void ahf_model_init(void)
{
    for (uint32_t h = 0; h <= MAX_H; h++) {
        const float wh = TWO_PI * F_GRID_HZ * (float)h;
        const cpx_t z  = c_mk(AHF_R_OHM, wh * AHF_L_H);
        const cpx_t d  = c_mk(cosf(wh * AHF_DELAY_S), -sinf(wh * AHF_DELAY_S));
        ahf_g[h] = c_mul(c_div(c_mk(AHF_VDC, 0.0f), z), d);
    }
}

/* 2nd-order Butterworth low-pass (bilinear, pre-warped) and its exact
 * response at every harmonic. The sample clock is locked to 200 samples per
 * cycle, so harmonic h is always at 2*pi*h/200 rad/sample.                 */
static void lpf_init(void)
{
    const float k = tanf(3.14159265f * A_LPF_HZ / (float)FS_HZ);
    const float n = 1.0f / (1.0f + 1.41421356f * k + k * k);
    a_lpf.b0 = k * k * n;
    a_lpf.b1 = 2.0f * a_lpf.b0;
    a_lpf.b2 = a_lpf.b0;
    a_lpf.a1 = 2.0f * (k * k - 1.0f) * n;
    a_lpf.a2 = (1.0f - 1.41421356f * k + k * k) * n;
    for (uint32_t h = 0; h <= MAX_H; h++) {
        const float w  = TWO_PI * (float)h / (float)N_CYC;
        const cpx_t z1 = c_mk(cosf(w), -sinf(w));            /* z^-1 */
        const cpx_t z2 = c_mul(z1, z1);
        const cpx_t num = c_add(c_mk(a_lpf.b0, 0.0f),
                                c_add(c_scale(z1, a_lpf.b1), c_scale(z2, a_lpf.b2)));
        const cpx_t den = c_add(c_mk(1.0f, 0.0f),
                                c_add(c_scale(z1, a_lpf.a1), c_scale(z2, a_lpf.a2)));
        a_lpf_h[h] = c_div(num, den);
    }
}

static void tables_init(void)
{
    for (uint32_t k = 0; k < N_CYC; k++) {
        float th = TWO_PI * (float)k / (float)N_CYC;
        cos_t[k] = cosf(th);
        sin_t[k] = sinf(th);
    }
    tan_phi_tgt = sqrtf(1.0f - PF_TARGET * PF_TARGET) / PF_TARGET;
    memset(ahf_tab, 0, sizeof ahf_tab);
    ahf_model_init();
    lpf_init();
}

static void dft(const float *x, float *a, float *b)
{
    for (uint32_t h = 1; h <= MAX_H; h++) {
        float sa = 0.0f, sb = 0.0f;
        uint32_t idx = 0;
        for (uint32_t k = 0; k < N_CYC; k++) {
            sa += x[k] * cos_t[idx];
            sb += x[k] * sin_t[idx];
            idx += h;
            if (idx >= N_CYC) idx -= N_CYC;
        }
        a[h] = sa * (2.0f / (float)N_CYC);
        b[h] = sb * (2.0f / (float)N_CYC);
    }
}

static float thd_pct(const float *h)
{
    float s = 0.0f;
    for (uint32_t k = 2; k <= MAX_H; k++) s += h[k] * h[k];
    return (h[1] > 1e-6f) ? 100.0f * sqrtf(s) / h[1] : 0.0f;
}

static void ahf_reset(void)
{
    memset(ahf_u, 0, sizeof ahf_u);
    memset(ahf_tab_u, 0, sizeof ahf_tab_u);
    memset(ahf_tab, 0, sizeof ahf_tab);
    ahf_resid_pct = ahf_duty_pk = 0.0f;
}

/* One control step. ai/bi: DFT of sensor I, aa/ba: DFT of sensor A, taken
 * over a window during which table `applied` played unchanged. Writes the
 * next table into ahf_tab[applied ^ 1]; the caller hands it to the ISR.     */
static void ahf_control(const float *ai, const float *bi, const float *aa, const float *ba,
                        uint32_t applied, bool drive)
{
    static float u[N_CYC];
    float r2 = 0.0f, e2 = 0.0f;

    for (uint32_t h = 2; h <= MAX_H; h++) {
        if (!(AHF_HARM_MASK & (1u << h))) { ahf_u[h] = c_mk(0.0f, 0.0f); continue; }
        const cpx_t ii = c_phasor(ai[h], bi[h]);
        const cpx_t m  = c_phasor(aa[h], ba[h]);
        const cpx_t ua = ahf_tab_u[applied][h];

        /* plant identification: what this duty phasor actually produced.
         * The first measurement replaces the model outright: blending from a
         * badly wrong model would pass through |G| ~ 0 and blow up E / G.  */
        if (c_abs2(ua) > AHF_ID_MIN_DUTY * AHF_ID_MIN_DUTY) {
            const cpx_t gm = c_div(m, ua);
            const float r  = c_abs2(gm) / c_abs2(ahf_g[h]);
            if (!ahf_g_seen[h]) {
                if (c_abs2(gm) > 0.0f) { ahf_g[h] = gm; ahf_g_seen[h] = 1u; }
            } else if (r > 1e-4f && r < 1e4f) {
                ahf_g[h] = c_add(ahf_g[h], c_scale(c_sub(gm, ahf_g[h]), AHF_ID_RATE));
            }
        }

        if (!drive) { ahf_u[h] = c_scale(ahf_u[h], 0.5f); continue; }
#if AHF_SENSE_GRID
        const cpx_t e = c_scale(ii, -1.0f / AHF_INJ_RATIO);   /* grid residual */
        const cpx_t r = c_add(e, m);
#else
        const cpx_t r = c_scale(ii, -1.0f / AHF_INJ_RATIO);   /* -load harmonic */
        const cpx_t e = c_sub(r, m);
#endif
        r2 += c_abs2(r);
        e2 += c_abs2(e);
        const float mu = ahf_g_seen[h] ? AHF_MU : AHF_MU_PROBE;
        ahf_u[h] = c_scale(c_add(ahf_u[h], c_scale(c_div(e, ahf_g[h]), mu)),
                           1.0f - AHF_LEAK);
    }
    ahf_resid_pct = (r2 > 1e-10f) ? 100.0f * sqrtf(e2 / r2) : 0.0f;

    /* synthesise one cycle from the harmonic phasors */
    memset(u, 0, sizeof u);
    for (uint32_t h = 2; h <= MAX_H; h++) {
        const float ur = ahf_u[h].re, ui = ahf_u[h].im;
        if (ur == 0.0f && ui == 0.0f) continue;
        uint32_t idx = 0;
        for (uint32_t k = 0; k < N_CYC; k++) {
            u[k] += ur * cos_t[idx] - ui * sin_t[idx];
            idx += h;
            if (idx >= N_CYC) idx -= N_CYC;
        }
    }
    float pk = 0.0f;
    for (uint32_t k = 0; k < N_CYC; k++) if (fabsf(u[k]) > pk) pk = fabsf(u[k]);
    if (pk > AHF_MOD_DEPTH) {                 /* saturation: scale, no windup */
        const float s = AHF_MOD_DEPTH / pk;
        for (uint32_t h = 2; h <= MAX_H; h++) ahf_u[h] = c_scale(ahf_u[h], s);
        for (uint32_t k = 0; k < N_CYC; k++) u[k] *= s;
        pk = AHF_MOD_DEPTH;
    }
    ahf_duty_pk = pk;

    int16_t *t = ahf_tab[applied ^ 1u];
    for (uint32_t k = 0; k < N_CYC; k++) {
        const float f = u[k] * 32767.0f;
        t[k] = (int16_t)(f + (f >= 0.0f ? 0.5f : -0.5f));
    }
    memcpy(ahf_tab_u[applied ^ 1u], ahf_u, sizeof ahf_u);
}

/* Grid lock from the voltage fundamental (a1, b1) of the last window. With
 * the clock locked, the window start sits at a fixed point of the cycle and
 * every harmonic phasor is stationary from one window to the next.         */
static void pll_update(float a1, float b1, bool valid)
{
    if (!valid) {
        pll_have_prev = 0;
        pll_locked    = 0;
        pll_f_cmd += 0.2f * (F_GRID_HZ - pll_f_cmd);
        return;
    }
    const float ph = atan2f(-b1, a1);
    if (pll_have_prev) {
        /* phase gained over CYC_PER_WIN cycles of the commanded frequency */
        const float f_grid = pll_f_cmd *
            (1.0f + wrap_pi(ph - pll_ph_prev) / (TWO_PI * (float)CYC_PER_WIN));
        pll_f_est += PLL_F_ALPHA * (f_grid - pll_f_est);
        float f = f_grid + PLL_KP_HZ * wrap_pi(ph - PLL_PHASE_SET);
        if (f < PLL_F_MIN) f = PLL_F_MIN;
        if (f > PLL_F_MAX) f = PLL_F_MAX;
        pll_f_cmd  = f;
        pll_locked = fabsf(wrap_pi(ph - PLL_PHASE_SET)) < 0.35f;    /* ~1 ms  */
    }
    pll_ph_prev   = ph;
    pll_have_prev = 1;
}

/* TIM2 period for frequency f: n whole ticks plus a 16-bit fraction. */
static void pll_period(float f, uint32_t *n, uint32_t *frac)
{
    const float t = (float)APB1_TIM_HZ / ((float)N_CYC * f);
    *n    = (uint32_t)t;
    *frac = (uint32_t)((t - (float)*n) * 65536.0f);
}
/* DSP-CORE-END */

/* ------------------------------------------------------------------------- */
/*  Globals                                                                  */
/* ------------------------------------------------------------------------- */
static volatile uint16_t adc_buf[DMA_LEN];
static uint16_t          snap[N_WIN * N_CH];
static volatile int32_t  win_ready = -1;
static volatile uint8_t  win_clean[2];      /* no table swap inside the half */
static volatile uint32_t win_dropped, adc_ovr_cnt, tx_skip;
static volatile bool     resync_req;
static volatile uint32_t ms_ticks;          /* advanced by the ADC DMA ISR   */

static volatile uint32_t tim2_n = TIM2_N_NOM, tim2_frac, tim2_acc;
static volatile uint32_t pwm_period = PWM_PERIOD_NOM;

static float w_v[N_CYC], w_i[N_CYC], w_a[N_CYC], w_c[N_CYC], w_t[N_CYC];
static float av[MAX_H + 1], bv[MAX_H + 1], ai[MAX_H + 1], bi[MAX_H + 1],
             aa[MAX_H + 1], ba[MAX_H + 1];

typedef struct {
    float vrms, irms, p, s, pf, p1, q1, dpf, dva;
    float thd_v, thd_i, crest, vpk, ipk, apk;
    float h_i[MAX_H + 1], h_a[MAX_H + 1];
    float off[N_CH];
    uint16_t mn[N_CH], mx[N_CH];
    uint32_t clip[N_CH];
} win_t;

typedef struct { float sv2, si2, p, p1, q1, vpk, ipk; uint32_t n; } acc_t;
typedef struct { float vrms, irms, p, s, pf, p1, q1, dpf, dva, vpk, ipk; } avg_t;

static win_t   mw;
static acc_t   acc;
static uint32_t settle_win, win_count;

static volatile uint32_t ahf_sel, ahf_swap, ahf_swapped;
static volatile uint32_t ahf_idx;
static volatile uint32_t ahf_active = AHF_BOOT_ACTIVE;
static float             ahf_ref_pk_f;
static bool              ahf_is_reset;

static const float       cap_uF[3] = { 2.5f, 10.0f, 20.0f };
static volatile uint8_t  lock_mask = 0;
static volatile uint8_t  lock_state = 0;
static volatile bool     pfc_cmd_pending;
static uint8_t           relay_state = 0, relay_target = 0;
static uint32_t          relay_off_ms[3] = {0}, relay_on_ms[3] = {0};
static volatile uint8_t  relay_pend_on, relay_pend_on0, relay_pend_off;  /* TIM2 ISR */
static uint8_t           relay_v_known;     /* cap left near +Vpk by a timed open */
static bool              grid_v_present;
static uint8_t           pfc_hold = 1;
static float             pfc_c_need = 0.0f;
static uint8_t           pfc_reason;        /* 0 ok, 1 no load, 2 load below the
                                             * smallest step, 3 all manual     */
static uint8_t           relay_waiting;     /* close held back: no grid lock   */
static uint32_t          relay_wait_ms[3];
static float             lpf_z1, lpf_z2;    /* sensor A filter state           */
static bool              lpf_primed;

static char              tx_buf[TX_BUF_SZ];
static char             *tx_p;
static volatile bool     tx_busy;
static char              rx_line[RX_LINE_MAX];
static uint32_t          rx_len;
static volatile uint32_t cmd_ok, cmd_bad, rx_bytes;

/* ------------------------------------------------------------------------- */
/*  System Initialization                                                    */
/* ------------------------------------------------------------------------- */
static void clock_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    (void)RCC->APB1ENR;
    PWR->CR = (PWR->CR & ~PWR_CR_VOS) | (2u << PWR_CR_VOS_Pos);

    FLASH->ACR = FLASH_ACR_LATENCY_2WS | FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN;
    while ((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_2WS) {}

    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY)) {}
    RCC->CFGR = 0;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI) {}
    RCC->CR &= ~RCC_CR_PLLON;
    while (RCC->CR & RCC_CR_PLLRDY) {}

    RCC->PLLCFGR = (16u  << RCC_PLLCFGR_PLLM_Pos) |
                   (256u << RCC_PLLCFGR_PLLN_Pos) |
                   (1u   << RCC_PLLCFGR_PLLP_Pos) |
                   RCC_PLLCFGR_PLLSRC_HSI |
                   (8u   << RCC_PLLCFGR_PLLQ_Pos);
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) {}

    RCC->CFGR = RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV1 | RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {}
    SystemCoreClock = SYSCLK_HZ;
}

/* BSRR value that puts the relay pins in `bits` into state `st` */
static uint32_t relay_bsrr(uint8_t bits, uint8_t st)
{
    /* pin HIGH = SSR ON, or relay module OFF (released open-drain) */
    const uint32_t m  = (uint32_t)bits & 7u;
    const uint32_t hi = (((uint32_t)st & RELAY_SSR_MASK) |
                         (~(uint32_t)st & ~RELAY_SSR_MASK)) & m;
    return hi | ((m & ~hi) << 16);
}

static void relay_write(uint8_t st)
{
    GPIOB->BSRR = relay_bsrr(7u, st);
}

/* Called by the TIM2 ISR for every sample; i = sample index in the cycle.
 * Drives the coils that relay_service() scheduled for point-on-wave.     */
static inline void relay_isr_step(uint32_t i)
{
    if (relay_pend_on && i == RELAY_K_ON) {           /* close at +peak       */
        GPIOB->BSRR   = relay_bsrr(relay_pend_on, 7u);
        relay_pend_on = 0u;
    }
    if (relay_pend_on0 && i == RELAY_K_ON_ZERO) {     /* close at zero cross  */
        GPIOB->BSRR    = relay_bsrr(relay_pend_on0, 7u);
        relay_pend_on0 = 0u;
    }
    if (relay_pend_off && i == RELAY_K_OFF) {
        GPIOB->BSRR    = relay_bsrr(relay_pend_off, 0u);
        relay_pend_off = 0u;
    }
}

static void gpio_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN |
                    RCC_AHB1ENR_DMA1EN  | RCC_AHB1ENR_DMA2EN;
    (void)RCC->AHB1ENR;

    /* PA0, PA1, PA4 analog */
    GPIOA->MODER |= (3u << (0 * 2)) | (3u << (1 * 2)) | (3u << (4 * 2));
    GPIOA->PUPDR &= ~((3u << (0 * 2)) | (3u << (1 * 2)) | (3u << (4 * 2)));

    /* PA2/PA3 AF7 USART2 */
    GPIOA->MODER   = (GPIOA->MODER & ~((3u << (2 * 2)) | (3u << (3 * 2)))) |
                     (2u << (2 * 2)) | (2u << (3 * 2));
    GPIOA->AFR[0]  = (GPIOA->AFR[0] & ~((0xFu << (2 * 4)) | (0xFu << (3 * 4)))) |
                     (7u << (2 * 4)) | (7u << (3 * 4));
    GPIOA->PUPDR   = (GPIOA->PUPDR & ~(3u << (3 * 2))) | (1u << (3 * 2));

    /* PA5 LD2 */
    GPIOA->MODER   = (GPIOA->MODER & ~(3u << (5 * 2))) | (1u << (5 * 2));

    /* Relays. Order matters: output type and OFF level are set while the pins
     * are still inputs, so they never drive LOW (= relay ON) even briefly.  */
    GPIOB->OTYPER  = (GPIOB->OTYPER & ~7u) | (~RELAY_SSR_MASK & 7u);  /* modules OD */
    GPIOB->PUPDR  &= ~((3u << (0 * 2)) | (3u << (1 * 2)) | (3u << (2 * 2)));
    relay_write(0);                                          /* all OFF       */
    relay_state  = 0;
    relay_target = 0;
    GPIOB->MODER   = (GPIOB->MODER & ~((3u << 0) | (3u << 2) | (3u << 4))) |
                     (1u << 0) | (1u << 2) | (1u << 4);

    /* PB6/PB8 AF2 TIM4 */
    GPIOB->BSRR    = GPIO_BSRR_BR6 | GPIO_BSRR_BR8;
    GPIOB->PUPDR   = (GPIOB->PUPDR & ~((3u << (6 * 2)) | (3u << (8 * 2)))) |
                     (2u << (6 * 2)) | (2u << (8 * 2));
    GPIOB->OSPEEDR = (GPIOB->OSPEEDR & ~((3u << (6 * 2)) | (3u << (8 * 2)))) |
                     (2u << (6 * 2)) | (2u << (8 * 2));
    GPIOB->AFR[0]  = (GPIOB->AFR[0] & ~(0xFu << (6 * 4))) | (2u << (6 * 4));
    GPIOB->AFR[1]  = (GPIOB->AFR[1] & ~(0xFu << ((8 - 8) * 4))) | (2u << ((8 - 8) * 4));
    GPIOB->MODER   = (GPIOB->MODER & ~((3u << (6 * 2)) | (3u << (8 * 2)))) |
                     (2u << (6 * 2)) | (2u << (8 * 2));
}

#if RELAY_SELFTEST
/* Busy-wait on the DWT cycle counter (no interrupts needed). */
static void delay_ms(uint32_t ms)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    const uint32_t t0 = DWT->CYCCNT, n = ms * (SYSCLK_HZ / 1000u);
    while ((DWT->CYCCNT - t0) < n) {}
}

static void relay_selftest(void)
{
    for (uint8_t i = 0; i < 3u; i++) {
        relay_write((uint8_t)(1u << i));      /* LED + click ON  */
        delay_ms(1000);
        relay_write(0);                       /* LED + click OFF */
        delay_ms(1000);
    }
}
#endif

/* ========================================================================= */
/*  TIM2 (grid-locked sample clock), ADC1 + DMA2 Stream0                     */
/* ========================================================================= */
static void tim2_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    (void)RCC->APB1ENR;
    TIM2->CR1  = TIM_CR1_ARPE;
    TIM2->PSC  = 0;
    TIM2->ARR  = TIM2_N_NOM - 1u;
    TIM2->CR2  = TIM_CR2_MMS_1;               /* TRGO = update            */
    TIM2->EGR  = TIM_EGR_UG;
    TIM2->SR   = 0;
    TIM2->CNT  = 0;
    TIM2->DIER = TIM_DIER_UIE;
}

static void adc_dma_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    (void)RCC->APB2ENR;

    ADC->CCR    = (1u << ADC_CCR_ADCPRE_Pos);                  /* PCLK2 / 4  */
    ADC1->CR1   = ADC_CR1_SCAN | ADC_CR1_OVRIE;
    ADC1->SMPR2 = (ADC_SMP_CODE << ADC_SMPR2_SMP0_Pos) |
                  (ADC_SMP_CODE << ADC_SMPR2_SMP1_Pos) |
                  (ADC_SMP_CODE << ADC_SMPR2_SMP4_Pos);
    ADC1->SQR1  = (N_CH - 1u) << ADC_SQR1_L_Pos;
    ADC1->SQR3  = (0u << ADC_SQR3_SQ1_Pos) | (1u << ADC_SQR3_SQ2_Pos) | (4u << ADC_SQR3_SQ3_Pos);
    ADC1->CR2   = (6u << ADC_CR2_EXTSEL_Pos) |
                  (1u << ADC_CR2_EXTEN_Pos)  |
                  ADC_CR2_DDS | ADC_CR2_ADON;

    DMA2_Stream0->CR = 0;
    while (DMA2_Stream0->CR & DMA_SxCR_EN) {}
    DMA2_Stream0->PAR  = (uint32_t)&ADC1->DR;
    DMA2_Stream0->M0AR = (uint32_t)adc_buf;
    DMA2_Stream0->NDTR = DMA_LEN;
    DMA2_Stream0->FCR  = 0;
    DMA2_Stream0->CR   = (0u << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_PL_1 |
                         DMA_SxCR_MSIZE_0 | DMA_SxCR_PSIZE_0 | DMA_SxCR_MINC |
                         DMA_SxCR_CIRC | DMA_SxCR_HTIE | DMA_SxCR_TCIE | DMA_SxCR_TEIE;
}

static inline void win_post(int32_t half)
{
    if (win_ready >= 0) win_dropped++;
    __disable_irq();
    win_clean[half] = (uint8_t)(ahf_swapped ? 0u : 1u);
    ahf_swapped = 0u;
    __enable_irq();
    win_ready = half;
    ms_ticks += WIN_MS;               /* each half-buffer = 5 grid cycles */
}

void DMA2_Stream0_IRQHandler(void)
{
    uint32_t isr = DMA2->LISR;
    if (isr & DMA_LISR_TEIF0) { DMA2->LIFCR = DMA_LIFCR_CTEIF0; resync_req = true; }
    if (isr & DMA_LISR_HTIF0) { DMA2->LIFCR = DMA_LIFCR_CHTIF0; win_post(0); }
    if (isr & DMA_LISR_TCIF0) { DMA2->LIFCR = DMA_LIFCR_CTCIF0; win_post(1); }
}

void ADC_IRQHandler(void)
{
    if (ADC1->SR & ADC_SR_OVR) {
        ADC1->SR = ~ADC_SR_OVR;
        adc_ovr_cnt++;
        resync_req = true;
    }
}

/* ========================================================================= */
/*  TIM4  -  80 kHz TB6612FNG PWM, reset by every TIM2 update                */
/* ========================================================================= */
static void tim4_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    (void)RCC->APB1ENR;
    TIM4->CR1   = TIM_CR1_ARPE;               /* URS = 0: reset loads CCRs */
    TIM4->PSC   = 0;
    TIM4->ARR   = PWM_PERIOD_NOM - 1u;
    TIM4->CCMR1 = (6u << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
    TIM4->CCMR2 = (6u << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;
    TIM4->CCR1  = 0;
    TIM4->CCR3  = 0;
    TIM4->CCER  = TIM_CCER_CC1E | TIM_CCER_CC3E;
    TIM4->EGR   = TIM_EGR_UG;
    TIM4->SR    = 0;
    TIM4->SMCR  = (1u << TIM_SMCR_TS_Pos) | (4u << TIM_SMCR_SMS_Pos);  /* ITR1 = TIM2, reset */
    TIM4->DIER  = 0;
}

/* One sample: trim the sample period (grid lock) and step the AHF table. */
void TIM2_IRQHandler(void)
{
    TIM2->SR = ~TIM_SR_UIF;

    /* fractional period: ARR alternates between n-1 and n */
    const uint32_t a = tim2_acc + tim2_frac;
    tim2_acc  = a & 0xFFFFu;
    TIM2->ARR = tim2_n - 1u + (a >> 16);

    const uint32_t i = ahf_idx;
    if (i == 0u && ahf_swap) { ahf_sel ^= 1u; ahf_swap = 0u; ahf_swapped = 1u; }
    ahf_idx = (i + 1u < N_CYC) ? i + 1u : 0u;

    relay_isr_step(i);

    if (!ahf_active) {
        TIM4->CCR1 = 0;
        TIM4->CCR3 = 0;
        return;
    }

    /* Sign-magnitude, drive/brake (slow decay): one input held HIGH
     * (CCR > ARR), the other pulses LOW for d of the period.               */
    const int32_t  v = ahf_tab[ahf_sel][i];
    const uint32_t p = pwm_period;
    const uint32_t d = ((uint32_t)(v >= 0 ? v : -v) * p) >> 15;
    if (v >= 0) {
        TIM4->CCR1 = 0xFFFFu;
        TIM4->CCR3 = p - d;
    } else {
        TIM4->CCR3 = 0xFFFFu;
        TIM4->CCR1 = p - d;
    }
}

static void sampling_sync_start(void)
{
    __disable_irq();
    TIM2->CR1 &= ~TIM_CR1_CEN;
    TIM4->CR1 &= ~TIM_CR1_CEN;

    ADC1->CR2 &= ~(ADC_CR2_DMA | ADC_CR2_ADON);
    DMA2_Stream0->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream0->CR & DMA_SxCR_EN) {}
    DMA2->LIFCR = DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTEIF0 |
                  DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0;
    DMA2_Stream0->M0AR = (uint32_t)adc_buf;
    DMA2_Stream0->NDTR = DMA_LEN;
    DMA2_Stream0->CR  |= DMA_SxCR_EN;
    ADC1->SR = 0;
    ADC1->CR2 |= ADC_CR2_ADON;
    for (volatile uint32_t d = 0; d < 400u; d++) {}
    ADC1->CR2 |= ADC_CR2_DMA;

    TIM4->CCR1 = 0;
    TIM4->CCR3 = 0;
    TIM4->EGR  = TIM_EGR_UG;
    TIM4->SR   = 0;
    TIM4->CNT  = 0;
    TIM4->CR1 |= TIM_CR1_CEN;
    TIM2->CNT  = 0;
    TIM2->SR   = 0;
    tim2_acc   = 0;
    ahf_idx    = 0;                   /* table index == sample index mod 200 */
    lpf_primed = false;
    ahf_swapped = 0;
    win_ready  = -1;

    TIM2->CR1 |= TIM_CR1_CEN;
    __enable_irq();
}

/* Apply the PLL frequency to TIM2 (and keep 8 PWM periods per sample). */
static void pll_apply(void)
{
    uint32_t n, frac;
    pll_period(pll_f_cmd, &n, &frac);
    const uint32_t p = (n + PWM_PER_SAMPLE / 2u) / PWM_PER_SAMPLE;
    __disable_irq();
    tim2_n    = n;
    tim2_frac = frac;
    if (p != pwm_period) {
        pwm_period = p;
        TIM4->ARR  = p - 1u;
    }
    __enable_irq();
}

/* ========================================================================= */
/*  UART & Command Parsing                                                   */
/* ========================================================================= */
static void uart_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    (void)RCC->APB1ENR;
    USART2->CR1 = 0;
    USART2->BRR = (APB1_HZ + UART_BAUD / 2u) / UART_BAUD;
    USART2->CR3 = USART_CR3_DMAT;
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;

    DMA1_Stream6->CR = 0;
    while (DMA1_Stream6->CR & DMA_SxCR_EN) {}
    DMA1_Stream6->PAR = (uint32_t)&USART2->DR;
    DMA1_Stream6->FCR = 0;
    DMA1_Stream6->CR  = (4u << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MINC |
                        DMA_SxCR_DIR_0 | DMA_SxCR_TCIE | DMA_SxCR_TEIE;
}

static bool uart_tx_dma(uint32_t len)
{
    if (tx_busy || len == 0u) return false;
    tx_busy = true;
    DMA1_Stream6->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream6->CR & DMA_SxCR_EN) {}
    DMA1->HIFCR = DMA_HIFCR_CTCIF6 | DMA_HIFCR_CHTIF6 | DMA_HIFCR_CTEIF6 |
                  DMA_HIFCR_CDMEIF6 | DMA_HIFCR_CFEIF6;
    DMA1_Stream6->M0AR = (uint32_t)tx_buf;
    DMA1_Stream6->NDTR = len;
    USART2->SR &= ~USART_SR_TC;
    DMA1_Stream6->CR |= DMA_SxCR_EN;
    return true;
}

void DMA1_Stream6_IRQHandler(void)
{
    uint32_t isr = DMA1->HISR;
    if (isr & (DMA_HISR_TCIF6 | DMA_HISR_TEIF6)) {
        DMA1->HIFCR = DMA_HIFCR_CTCIF6 | DMA_HIFCR_CTEIF6;
        tx_busy = false;
    }
}

static const char *parse_u(const char *p, uint32_t *v)
{
    if (*p < '0' || *p > '9') return 0;
    uint32_t x = 0;
    while (*p >= '0' && *p <= '9') {
        x = x * 10u + (uint32_t)(*p++ - '0');
        if (x > 1000u) return 0;
    }
    *v = x;
    return p;
}

static void cmd_exec(const char *s)
{
    const char *p;
    uint32_t a, b, c;

    if (s[1] == 'C' && s[2] == ',') {
        if (!(p = parse_u(s + 3, &a)) || *p++ != ',') goto bad;
        if (!(p = parse_u(p, &b))     || *p++ != ',') goto bad;
        if (!(p = parse_u(p, &c))     || *p != '\0')  goto bad;
        if (a > 2u || b > 1u || c > 1u) goto bad;

        uint8_t bit = (uint8_t)(1u << a);
        if (b) {
            lock_mask  |= bit;
            lock_state  = c ? (lock_state | bit) : (lock_state & (uint8_t)~bit);
        } else {
            lock_mask  &= (uint8_t)~bit;
            lock_state &= (uint8_t)~bit;
        }
        pfc_cmd_pending = true;          /* main loop applies it at once */
        cmd_ok++;
        return;
    }
    if (s[1] == 'A' && s[2] == ',') {
        if (!(p = parse_u(s + 3, &a)) || *p != '\0' || a > 1u) goto bad;
        ahf_active = a;
        cmd_ok++;
        return;
    }
bad:
    cmd_bad++;
}

static void rx_feed(char ch)
{
    if (ch == '$') { rx_line[0] = ch; rx_len = 1; return; }
    if (ch == '\n' || ch == '\r') {
        if (rx_len) { rx_line[rx_len] = '\0'; cmd_exec(rx_line); }
        rx_len = 0;
        return;
    }
    if (rx_len == 0u) return;
    if (rx_len < RX_LINE_MAX - 1u) rx_line[rx_len++] = ch;
    else { rx_len = 0; cmd_bad++; }
}

void USART2_IRQHandler(void)
{
    uint32_t sr = USART2->SR;
    if (sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE)) {
        char ch = (char)USART2->DR;
        rx_bytes++;
        if (sr & (USART_SR_FE | USART_SR_NE)) { rx_len = 0; return; }
        rx_feed(ch);
    }
}

/* ========================================================================= */
/*  Signal Processing                                                        */
/* ========================================================================= */
static void process_window(const uint16_t *s, win_t *m)
{
    static uint32_t csum[N_CH][N_CYC];
    static float    csa[N_CYC];             /* sensor A after the low-pass    */
    uint32_t sum[N_CH] = { 0 };

    memset(csum, 0, sizeof csum);
    memset(csa, 0, sizeof csa);
    if (!lpf_primed) {                      /* start in steady state: no step */
        const float x0 = (float)s[SLOT_A];
        lpf_z2 = (a_lpf.b2 - a_lpf.a2) * x0;
        lpf_z1 = (a_lpf.b1 - a_lpf.a1) * x0 + lpf_z2;
        lpf_primed = true;
    }
    for (uint32_t c = 0; c < N_CH; c++) { m->mn[c] = 0xFFFFu; m->mx[c] = 0; m->clip[c] = 0; }

    for (uint32_t n = 0, k = 0; n < N_WIN; n++) {
        const uint16_t *x = &s[n * N_CH];
        for (uint32_t c = 0; c < N_CH; c++) {
            uint16_t r = x[c];
            sum[c]     += r;
            csum[c][k] += r;
            if (r < m->mn[c]) m->mn[c] = r;
            if (r > m->mx[c]) m->mx[c] = r;
            if (r <= ADC_CLIP_LO || r >= ADC_CLIP_HI) m->clip[c]++;
        }
        const float xa = (float)x[SLOT_A];                  /* Butterworth, DF2T */
        const float ya = a_lpf.b0 * xa + lpf_z1;
        lpf_z1 = a_lpf.b1 * xa - a_lpf.a1 * ya + lpf_z2;
        lpf_z2 = a_lpf.b2 * xa - a_lpf.a2 * ya;
        csa[k] += ya;
        if (++k == N_CYC) k = 0;
    }
    for (uint32_t c = 0; c < N_CH; c++) m->off[c] = (float)sum[c] / (float)N_WIN;

    const float oi = m->off[SLOT_I], ov = m->off[SLOT_V], oa = m->off[SLOT_A];
    float si2 = 0, sv2 = 0, svi = 0, ipk = 0, vpk = 0;
    for (uint32_t n = 0; n < N_WIN; n++) {
        const uint16_t *x = &s[n * N_CH];
        float fi = ((float)x[SLOT_I] - oi) * K_I;
        float fv = ((float)x[SLOT_V] - ov) * K_V;
        si2 += fi * fi;
        sv2 += fv * fv;
        svi += fv * fi;
        fi = fabsf(fi); fv = fabsf(fv);
        if (fi > ipk) ipk = fi;
        if (fv > vpk) vpk = fv;
    }
    m->irms = sqrtf(si2 / (float)N_WIN);
    m->vrms = sqrtf(sv2 / (float)N_WIN);

    if (m->vrms < V_DEADBAND) { m->vrms = 0.0f; vpk = 0.0f; }
    if (m->irms < I_DEADBAND) { m->irms = 0.0f; ipk = 0.0f; }

    m->p     = (m->vrms > 0.0f && m->irms > 0.0f) ? (svi / (float)N_WIN) : 0.0f;
    m->s     = m->vrms * m->irms;
    m->pf    = (m->s > 1e-3f) ? m->p / m->s : 0.0f;
    m->ipk   = ipk;
    m->vpk   = vpk;
    m->crest = (m->irms > 1e-4f) ? ipk / m->irms : 0.0f;

    const float inv = 1.0f / (float)CYC_PER_WIN;
    float apk = 0.0f;
    for (uint32_t k = 0; k < N_CYC; k++) {
        w_i[k] = (m->irms > 0.0f) ? (((float)csum[SLOT_I][k] * inv - oi) * K_I) : 0.0f;
        w_v[k] = (m->vrms > 0.0f) ? (((float)csum[SLOT_V][k] * inv - ov) * K_V) : 0.0f;
        w_a[k] = (csa[k] * inv - oa) * K_A;
        if (fabsf(w_a[k]) > apk) apk = fabsf(w_a[k]);
    }

    dft(w_i, ai, bi);
    dft(w_v, av, bv);
    dft(w_a, aa, ba);
    for (uint32_t h = 1; h <= MAX_H; h++) {     /* undo the filter per harmonic */
        const cpx_t x = c_div(c_phasor(aa[h], ba[h]), a_lpf_h[h]);
        aa[h] = x.re;
        ba[h] = -x.im;
    }

    /* Noise floor: below A_DEADBAND sensor A is shown as 0 (telemetry only;
     * the AHF loop still uses the real values in aa/ba).                   */
    float a2 = 0.0f;
    for (uint32_t k = 0; k < N_CYC; k++) a2 += w_a[k] * w_a[k];
    const bool a_on = sqrtf(a2 / (float)N_CYC) >= A_DEADBAND;
    if (!a_on) { memset(w_a, 0, sizeof w_a); apk = 0.0f; }
    m->apk = apk;
    float hv[MAX_H + 1];
    for (uint32_t h = 1; h <= MAX_H; h++) {
        m->h_i[h] = 0.70710678f * sqrtf(ai[h] * ai[h] + bi[h] * bi[h]);
        m->h_a[h] = a_on ? 0.70710678f * sqrtf(aa[h] * aa[h] + ba[h] * ba[h]) : 0.0f;
        hv[h]     = 0.70710678f * sqrtf(av[h] * av[h] + bv[h] * bv[h]);
    }
    m->h_i[0] = m->h_a[0] = 0.0f;
    m->thd_i = (m->irms > 0.0f) ? thd_pct(m->h_i) : 0.0f;
    m->thd_v = (m->vrms > 0.0f) ? thd_pct(hv)     : 0.0f;

    if (m->vrms > 0.0f && m->irms > 0.0f) {
        const float ph = ZMPT_LAG_DEG * 0.01745329f -
                         TWO_PI * pll_f_est * ADC_SLOT_S * (float)(SLOT_V - SLOT_I);
        const float cp = cosf(ph), sp = sinf(ph);
        const float a1v = av[1] * cp + bv[1] * sp;
        const float b1v = bv[1] * cp - av[1] * sp;
        m->p1  = 0.5f * (a1v * ai[1] + b1v * bi[1]);
        m->q1  = 0.5f * (a1v * bi[1] - b1v * ai[1]);
        const float s1 = sqrtf(m->p1 * m->p1 + m->q1 * m->q1);
        m->dpf = (s1 > 1e-3f) ? m->p1 / s1 : 0.0f;
        const float d2 = m->s * m->s - m->p * m->p - m->q1 * m->q1;
        m->dva = (d2 > 0.0f) ? sqrtf(d2) : 0.0f;
    } else {
        m->p1 = m->q1 = m->dpf = m->dva = 0.0f;
    }

    for (uint32_t k = 0; k < N_CYC; k++) {
        float i_fund = ai[1] * cos_t[k] + bi[1] * sin_t[k];
        w_t[k] = -(w_i[k] - i_fund);
        w_c[k] = w_i[k];
    }
#if !AHF_SENSE_GRID
    /* load + injection; the injection is rebuilt from its filter-corrected
     * harmonics so it lines up in time with the unfiltered sensor I        */
    if (a_on) {
        for (uint32_t h = 1; h <= MAX_H; h++) {
            uint32_t idx = 0;
            for (uint32_t k = 0; k < N_CYC; k++) {
                w_c[k] += AHF_INJ_RATIO * (aa[h] * cos_t[idx] + ba[h] * sin_t[idx]);
                idx += h;
                if (idx >= N_CYC) idx -= N_CYC;
            }
        }
    }
#endif
}

/* clean: the AHF table did not change during this window, so sensor A shows
 * the response to exactly one known table (needed for identification).    */
static void ahf_step(bool clean)
{
    float pk = 0.0f;
    for (uint32_t k = 0; k < N_CYC; k++) if (fabsf(w_t[k]) > pk) pk = fabsf(w_t[k]);
    ahf_ref_pk_f = (ahf_ref_pk_f <= 0.0f) ? pk
                 : (1.0f - AHF_PK_ALPHA) * ahf_ref_pk_f + AHF_PK_ALPHA * pk;

    if (!ahf_active) {
        if (!ahf_is_reset) { ahf_swap = 0u; ahf_reset(); ahf_is_reset = true; }
        return;
    }
    ahf_is_reset = false;
    if (!clean || ahf_swap) return;

    const bool drive = (mw.irms > 0.0f) && (ahf_ref_pk_f >= AHF_MIN_HARM_A);
    ahf_control(ai, bi, aa, ba, ahf_sel, drive);
    __DMB();
    ahf_swap = 1u;
}

static float cap_sum(uint8_t st)
{
    return ((st & 1u) ? cap_uF[0] : 0.0f) + ((st & 2u) ? cap_uF[1] : 0.0f) +
           ((st & 4u) ? cap_uF[2] : 0.0f);
}

static uint32_t popcnt3(uint8_t x) { return (x & 1u) + ((x >> 1) & 1u) + ((x >> 2) & 1u); }

static void acc_reset(void) { memset(&acc, 0, sizeof acc); }

static void acc_add(const win_t *m)
{
    acc.sv2 += m->vrms * m->vrms;
    acc.si2 += m->irms * m->irms;
    acc.p   += m->p;
    acc.p1  += m->p1;
    acc.q1  += m->q1;
    if (m->vpk > acc.vpk) acc.vpk = m->vpk;
    if (m->ipk > acc.ipk) acc.ipk = m->ipk;
    acc.n++;
}

static void make_avg(avg_t *a, const win_t *m)
{
    if (acc.n == 0u) {
        a->vrms = m->vrms; a->irms = m->irms; a->p = m->p; a->p1 = m->p1; a->q1 = m->q1;
        a->vpk = m->vpk;   a->ipk = m->ipk;
    } else {
        const float n = (float)acc.n;
        a->vrms = sqrtf(acc.sv2 / n); a->irms = sqrtf(acc.si2 / n);
        a->p = acc.p / n; a->p1 = acc.p1 / n; a->q1 = acc.q1 / n;
        a->vpk = acc.vpk; a->ipk = acc.ipk;
    }
    a->s   = a->vrms * a->irms;
    a->pf  = (a->s > 1e-3f) ? a->p / a->s : 0.0f;
    const float s1 = sqrtf(a->p1 * a->p1 + a->q1 * a->q1);
    a->dpf = (s1 > 1e-3f) ? a->p1 / s1 : 0.0f;
    const float d2 = a->s * a->s - a->p * a->p - a->q1 * a->q1;
    a->dva = (d2 > 0.0f) ? sqrtf(d2) : 0.0f;
}

static void locks_read(uint8_t *mask, uint8_t *st)
{
    __disable_irq();
    *mask = lock_mask & 7u;
    *st   = lock_state & 7u;
    __enable_irq();
}

static void apfc_step(const avg_t *a)
{
    uint8_t mask, lst;
    locks_read(&mask, &lst);

    if (mask == 7u) {
        relay_target = lst;
        pfc_hold = 1;
        pfc_c_need = cap_sum(lst);
        pfc_reason = 3u;
        return;
    }

    const uint8_t cur = (uint8_t)((relay_target & ~mask) | (lst & mask));

    float c_need = 0.0f;
    if (a->vrms >= APFC_V_MIN && fabsf(a->p1) >= APFC_P_MIN) {
        const float k      = TWO_PI * pll_f_est * a->vrms * a->vrms * 1e-6f;
        const float q_load = a->q1 + k * cap_sum(relay_state);
        const float q_tgt  = fabsf(a->p1) * tan_phi_tgt;
        c_need = (q_load - q_tgt) / k;
        if (c_need < 0.0f) c_need = 0.0f;
    }
    pfc_c_need = c_need;

    const float e_cur = fabsf(cap_sum(cur) - c_need);
    uint8_t best = cur;
    float   e_best = e_cur;
    for (uint8_t st = 0; st < 8u; st++) {
        if ((st & mask) != (lst & mask)) continue;
        const float e = fabsf(cap_sum(st) - c_need);
        if (e < e_best - 0.01f ||
            (e <= e_best + 0.01f && popcnt3(st ^ cur) < popcnt3(best ^ cur))) {
            best = st; e_best = e;
        }
    }

    if (best != cur && e_cur > C_DEADBAND_UF) { relay_target = (uint8_t)((best & ~mask) | (lst & mask)); pfc_hold = 0; }
    else                                      { relay_target = cur;  pfc_hold = 1; }

    if (a->vrms < APFC_V_MIN || fabsf(a->p1) < APFC_P_MIN) pfc_reason = 1u;
    else if (pfc_hold && cur == 0u && c_need > 0.0f)       pfc_reason = 2u;
    else                                                   pfc_reason = 0u;
}

static void relay_service(uint32_t now)
{
    uint8_t mask, lst;
    locks_read(&mask, &lst);

    relay_target = (uint8_t)((relay_target & ~mask) | (lst & mask));

    /* With no mains voltage there is nothing to time against and nothing
     * to protect: switch at once. Otherwise relay modules wait for the PLL. */
    const bool sync = RELAY_SYNC && grid_v_present;
    uint8_t st = relay_state, now_bits = 0, at_on = 0, at_on0 = 0, at_off = 0;
    relay_waiting &= (uint8_t)(relay_target & ~st);
    for (uint32_t i = 0; i < 3u; i++) {
        const uint8_t b    = (uint8_t)(1u << i);
        const bool    timed = sync && !(RELAY_SSR_MASK & b);
        if ((relay_target & b) && !(st & b)) {
            if ((now - relay_off_ms[i]) < CAP_RECLOSE_MS) continue;   /* min OFF  */
            bool at_wave = timed;
            if (timed && !pll_locked) {           /* wait for the peak timing ...  */
                if (!(relay_waiting & b)) { relay_waiting |= b; relay_wait_ms[i] = now; }
                if ((now - relay_wait_ms[i]) < RELAY_LOCK_WAIT_MS) continue;
                at_wave = false;                  /* ... but not forever          */
            }
            relay_waiting &= (uint8_t)~b;
            st |= b;
            relay_on_ms[i] = now;
            if (!at_wave) { now_bits |= b; relay_v_known &= (uint8_t)~b; }
            else if (relay_v_known & b) at_on  |= b;
            else                        at_on0 |= b;
        } else if (!(relay_target & b) && (st & b)) {
            if ((now - relay_on_ms[i]) < RELAY_MIN_ON_MS) continue;   /* min ON   */
            st &= (uint8_t)~b;
            relay_off_ms[i] = now;
            if (timed && pll_locked) { at_off |= b; relay_v_known |= b; }
            else { now_bits |= b; relay_v_known &= (uint8_t)~b; }
        }
    }
    if (st != relay_state) {
        __disable_irq();
        const uint8_t on_any = (uint8_t)(at_on | at_on0);
        relay_pend_on  = (uint8_t)((relay_pend_on  & ~(now_bits | at_off | at_on0)) | at_on);
        relay_pend_on0 = (uint8_t)((relay_pend_on0 & ~(now_bits | at_off | at_on))  | at_on0);
        relay_pend_off = (uint8_t)((relay_pend_off & ~(now_bits | on_any)) | at_off);
        if (now_bits) GPIOB->BSRR = relay_bsrr(now_bits, st);
        __enable_irq();
        relay_state = st;
        acc_reset();
        settle_win = PFC_SETTLE_WIN;
    }
}

/* ========================================================================= */
/*  Telemetry                                                                */
/* ========================================================================= */
static void tx_c(char c) { if (tx_p < tx_buf + TX_BUF_SZ) *tx_p++ = c; }
static void tx_s(const char *s) { while (*s) tx_c(*s++); }

static void tx_u(uint32_t v)
{
    char t[10];
    uint32_t n = 0;
    do { t[n++] = (char)('0' + v % 10u); v /= 10u; } while (v);
    while (n) tx_c(t[--n]);
}

static void tx_f(float x, uint32_t dec)
{
    static const uint32_t p10[] = { 1u, 10u, 100u, 1000u, 10000u };
    if (!(x == x) || x > 2.0e5f || x < -2.0e5f) { tx_c('0'); return; }
    const uint32_t sc = p10[dec];
    int32_t v = (int32_t)(x * (float)sc + (x >= 0.0f ? 0.5f : -0.5f));
    if (v < 0) { tx_c('-'); v = -v; }
    tx_u((uint32_t)v / sc);
    if (dec) {
        uint32_t fr = (uint32_t)v % sc;
        tx_c('.');
        for (uint32_t d = sc / 10u; d; d /= 10u) { tx_c((char)('0' + fr / d)); fr %= d; }
    }
}

static void tx_cf(float x, uint32_t dec) { tx_c(','); tx_f(x, dec); }
static void tx_cu(uint32_t v)            { tx_c(','); tx_u(v); }

static void tx_wave(char tag, const float *w, uint32_t dec)
{
    tx_s("$W,"); tx_c(tag);
    for (uint32_t k = 0; k < N_CYC; k += WAVE_DECIM) tx_cf(w[k], dec);
    tx_c('\n');
}

static void telemetry_send(const win_t *m, const avg_t *a)
{
    if (tx_busy) { tx_skip++; return; }
    tx_p = tx_buf;

    tx_wave('V', w_v, 1);
    tx_wave('I', w_i, 3);
    tx_wave('A', w_a, 3);
    tx_wave('T', w_t, 3);
    tx_wave('C', w_c, 3);

    tx_s("$M");
    tx_cf(a->vrms, 2); tx_cf(a->irms, 4); tx_cf(a->p, 2);   tx_cf(a->s, 2);
    tx_cf(a->q1, 2);   tx_cf(a->pf, 4);   tx_cf(m->thd_v, 2); tx_cf(m->thd_i, 2);
    tx_cf(m->crest, 3); tx_cf(a->vpk, 1); tx_cf(a->ipk, 4); tx_cf(a->dpf, 4);
    tx_cf(a->dva, 2);  tx_cf(pll_f_est, 3);
    tx_c('\n');

    tx_s("$H,I"); for (uint32_t h = 1; h <= MAX_H; h++) tx_cf(m->h_i[h], 4); tx_c('\n');
    tx_s("$H,A"); for (uint32_t h = 1; h <= MAX_H; h++) tx_cf(m->h_a[h], 4); tx_c('\n');

    uint8_t mask, lst;
    locks_read(&mask, &lst);
    tx_s("$P"); tx_cu(relay_state); tx_cf(cap_sum(relay_state), 1); tx_cf(a->pf, 4);
    tx_cf(a->q1, 2); tx_cu(pfc_hold); tx_cu(relay_target); tx_cf(pfc_c_need, 2);
    tx_cu(pll_locked); tx_cu(pfc_reason); tx_cu(relay_waiting); tx_c('\n');

    tx_s("$A"); tx_cu(ahf_active); tx_cf(m->apk, 4); tx_cf(ahf_ref_pk_f, 4);
    tx_cf(ahf_duty_pk, 3); tx_cf(ahf_resid_pct, 1); tx_c('\n');

    tx_s("$K"); tx_cu(mask); tx_cu(lst); tx_cu(cmd_ok); tx_cu(cmd_bad); tx_cu(rx_bytes);
    tx_c('\n');

    tx_s("$R");
    static const uint32_t order[N_CH] = { SLOT_V, SLOT_I, SLOT_A };
    for (uint32_t j = 0; j < N_CH; j++) {
        const uint32_t c = order[j];
        tx_cf(m->off[c], 1); tx_cu(m->mn[c]); tx_cu(m->mx[c]); tx_cu(m->clip[c]);
    }
    tx_cu(adc_ovr_cnt); tx_cu(win_dropped); tx_cu(tx_skip);
    tx_c('\n');

    tx_s("$E\n");
    uart_tx_dma((uint32_t)(tx_p - tx_buf));
}

/* ========================================================================= */
/*  Main                                                                     */
/* ========================================================================= */
static void nvic_init(void)
{
    NVIC_SetPriority(TIM2_IRQn,         0);
    NVIC_SetPriority(DMA2_Stream0_IRQn, 1);
    NVIC_SetPriority(ADC_IRQn,          1);
    NVIC_SetPriority(USART2_IRQn,       2);
    NVIC_SetPriority(DMA1_Stream6_IRQn, 3);
    NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_EnableIRQ(DMA2_Stream0_IRQn);
    NVIC_EnableIRQ(ADC_IRQn);
    NVIC_EnableIRQ(USART2_IRQn);
    NVIC_EnableIRQ(DMA1_Stream6_IRQn);
}

static void on_window(bool clean)
{
    process_window(snap, &mw);
    pll_update(av[1], bv[1], mw.vrms > 0.0f);
    grid_v_present = mw.vrms > 0.0f;
    pll_apply();
    ahf_step(clean);

    if (settle_win) settle_win--;
    else            acc_add(&mw);

    if (++win_count >= TELEM_WIN) {
        win_count = 0;
        avg_t a;
        make_avg(&a, &mw);
        if (acc.n >= 3u) apfc_step(&a);
        telemetry_send(&mw, &a);
        acc_reset();
        GPIOA->ODR ^= GPIO_ODR_OD5;
    }
}

int main(void)
{
    SCB->CPACR |= (0xFu << 20);
    __DSB(); __ISB();

    clock_init();
    gpio_init();                 /* relays forced OFF before anything else */
#if RELAY_SELFTEST
    relay_selftest();
#endif
    tables_init();
    uart_init();
    tim2_init();
    adc_dma_init();
    tim4_init();
    nvic_init();
    sampling_sync_start();

    for (;;) {
        if (resync_req) {
            resync_req = false;
            sampling_sync_start();
        }

        pfc_cmd_pending = false;
        relay_service(ms_ticks);       /* applies manual locks immediately */

        __disable_irq();
        const int32_t half = win_ready;
        win_ready = -1;
        __enable_irq();

        if (half >= 0) {
            const bool clean = win_clean[half] != 0u;
            memcpy(snap, (const void *)&adc_buf[(uint32_t)half * N_WIN * N_CH], sizeof snap);
            on_window(clean);
        } else {
            __WFI();
        }
    }
}
