/* PC simulation of the AHF selective-harmonic loop and the grid PLL.
 * The plant deliberately differs from the firmware's initial model
 * (other Vdc, L, R, delay and a reversed sensor) to show that the on-line
 * identification, not the model, is what makes the loop converge.        */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "core.h"

static int fails;
#define CHECK(cond, ...) do { if (cond) printf("  PASS  "); else { printf("  FAIL  "); fails++; } \
                              printf(__VA_ARGS__); printf("\n"); } while (0)

static double gauss(void)
{
    double u1 = (rand() + 1.0) / (RAND_MAX + 2.0), u2 = (rand() + 1.0) / (RAND_MAX + 2.0);
    return sqrt(-2.0 * log(u1)) * cos(6.283185307 * u2);
}

/* ---------------- plant ------------------------------------------------- */
static cpx_t g_true[MAX_H + 1];
static cpx_t load_h[MAX_H + 1];          /* load current phasors, A peak  */
static cpx_t dist_h[MAX_H + 1];          /* sensor-A current not caused by the bridge */

static void plant_init(double vdc, double l, double r, double delay_samples, double sign)
{
    for (int h = 0; h <= MAX_H; h++) {
        double wh = 6.283185307 * 50.0 * h, d = wh * delay_samples / 10000.0;
        cpx_t z = c_mk((float)r, (float)(wh * l));
        g_true[h] = c_scale(c_mul(c_div(c_mk((float)vdc, 0), z), c_mk((float)cos(d), (float)-sin(d))),
                            (float)sign);
    }
}

static void set_load(double scale)
{
    static const double amp[MAX_H + 1] = { 0, 2.0, 0.02, 0.45, 0.01, 0.28, 0.01, 0.14, 0.0, 0.07, 0.0, 0.05, 0.0, 0.035, 0.0, 0.025 };
    for (int h = 0; h <= MAX_H; h++) {
        double ph = 0.7 * h + 0.3 * h * h;      /* arbitrary but fixed */
        double a = (h == 1 ? 1.0 : scale) * amp[h];
        load_h[h] = c_mk((float)(a * cos(ph)), (float)(a * sin(ph)));
    }
}

/* phasors -> one cycle of samples (+ noise per sample, already 5-cycle averaged) */
static void synth(const cpx_t *ph, float *x, double noise)
{
    for (int k = 0; k < N_CYC; k++) {
        double s = 0;
        for (int h = 1; h <= MAX_H; h++) {
            double th = 6.283185307 * h * k / N_CYC;
            s += ph[h].re * cos(th) - ph[h].im * sin(th);
        }
        x[k] = (float)(s + noise * gauss());
    }
}

static double thd_of(const cpx_t *ph)
{
    double s = 0;
    for (int h = 2; h <= MAX_H; h++) s += c_abs2(ph[h]);
    return 100.0 * sqrt(s / c_abs2(ph[1]));
}

/* one 100 ms window: what the sensors see while table `applied` plays */
static double run_window(uint32_t applied, float *ai, float *bi, float *aa, float *ba, cpx_t *grid)
{
    float wi[N_CYC], wa[N_CYC], u[N_CYC], au[MAX_H + 1], bu[MAX_H + 1];
    cpx_t m[MAX_H + 1] = { { 0, 0 } };
    for (int k = 0; k < N_CYC; k++) u[k] = ahf_tab[applied][k] / 32767.0f;
    dft(u, au, bu);                              /* duty actually played (quantised) */
    for (int h = 2; h <= MAX_H; h++)
        m[h] = c_add(c_mul(g_true[h], c_phasor(au[h], bu[h])), dist_h[h]);
    synth(load_h, wi, 0.017);                    /* ACS712 noise after 5-cycle averaging */
    synth(m, wa, 0.017);
    dft(wi, ai, bi);
    dft(wa, aa, ba);
    for (int h = 1; h <= MAX_H; h++) grid[h] = c_add(load_h[h], m[h]);   /* grid = load + injection */
    return thd_of(grid);
}

/* windows alternate: control step on a clean window, the next one holds the swap */
static double run_loop(int updates, int verbose, double *resid)
{
    float ai[MAX_H + 1], bi[MAX_H + 1], aa[MAX_H + 1], ba[MAX_H + 1];
    cpx_t grid[MAX_H + 1];
    static uint32_t sel;
    double thd = 0;
    for (int n = 0; n < updates; n++) {
        thd = run_window(sel, ai, bi, aa, ba, grid);
        ahf_control(ai, bi, aa, ba, sel, true);
        if (verbose && (n < 8 || n % 10 == 9))
            printf("    update %3d  grid THD %6.2f %%   loop residual %6.2f %%   duty pk %.2f\n",
                   n + 1, thd, ahf_resid_pct, ahf_duty_pk);
        sel ^= 1u;                               /* ISR swaps at the next cycle start */
    }
    *resid = ahf_resid_pct;
    return thd;
}

int main(void)
{
    srand(1);
    tables_init();
    double resid, thd;

    printf("AHF loop: plant 15 V, 8 mH, 2 ohm, 2.7-sample delay, bridge polarity reversed\n"
           "          model 12 V, 10 mH, 1 ohm, 1.5-sample delay\n");
    plant_init(15.0, 0.008, 2.0, 2.7, -1.0);
    dist_h[5] = c_mk(0.004f, 0.002f);            /* PCC voltage harmonic seen by sensor A */
    set_load(1.0);
    cpx_t g0[MAX_H + 1];
    for (int h = 0; h <= MAX_H; h++) g0[h] = load_h[h];
    printf("  load THD %.2f %%\n", thd_of(g0));

    {   /* previous firmware: table = target shape scaled to 92 % peak, open loop */
        float t[N_CYC], a[MAX_H + 1], b[MAX_H + 1], pk = 0;
        cpx_t tgt[MAX_H + 1] = { { 0, 0 } }, grid[MAX_H + 1];
        for (unsigned h = 2; h <= MAX_H; h++) tgt[h] = c_scale(load_h[h], -1.0f);
        synth(tgt, t, 0.0);
        for (unsigned k = 0; k < N_CYC; k++) pk = fmaxf(pk, fabsf(t[k]));
        for (unsigned k = 0; k < N_CYC; k++) t[k] *= AHF_MOD_DEPTH / pk;
        dft(t, a, b);
        for (int pol = 1; pol >= -1; pol -= 2) {
            grid[1] = load_h[1];
            for (unsigned h = 2; h <= MAX_H; h++)
                grid[h] = c_add(load_h[h], c_scale(c_mul(g_true[h], c_phasor(a[h], b[h])), (float)-pol));
            printf("  previous open-loop method, %s polarity: grid THD %.1f %%\n",
                   pol > 0 ? "correct" : "reversed", thd_of(grid));
        }
    }
    thd = run_loop(40, 1, &resid);               /* 40 updates = 8 s at 5 updates/s */
    CHECK(thd < 3.0, "converges from a 180-deg-wrong model: grid THD %.2f %% (load %.1f %%)", thd, thd_of(g0));
    CHECK(resid < 5.0, "loop residual %.2f %%", resid);

    double gerr = 0;
    for (int h = 3; h <= MAX_H; h += 2)
        gerr = fmax(gerr, sqrt(c_abs2(c_sub(ahf_g[h], g_true[h])) / c_abs2(g_true[h])));
    CHECK(gerr < 0.15, "identified plant within %.1f %% on odd harmonics", 100 * gerr);

    printf("\nLoad step: harmonics x1.6\n");
    set_load(1.6);
    thd = run_loop(15, 0, &resid);
    CHECK(thd < 3.0, "re-converges in 3 s: grid THD %.2f %%", thd);

    printf("\nSaturation: harmonics x6 (needs more than %.0f %% duty)\n", 100 * AHF_MOD_DEPTH);
    set_load(6.0);
    thd = run_loop(15, 0, &resid);
    CHECK(fabsf(ahf_duty_pk - AHF_MOD_DEPTH) < 1e-3f, "duty clamped at %.2f, THD %.1f %% (partial, as expected)", ahf_duty_pk, thd);
    set_load(1.0);
    thd = run_loop(10, 0, &resid);
    CHECK(thd < 3.0, "no wind-up: back to %.2f %% within 2 s after the overload", thd);

    printf("\nGrid lock\n");
    const double fg[] = { 50.0, 49.83, 50.17, 51.2 };
    for (unsigned t = 0; t < sizeof fg / sizeof fg[0]; t++) {
        double tim = 0.1234, ph_last = 0;
        pll_have_prev = 0;
        pll_f_cmd = F_GRID_HZ;
        for (int w = 0; w < 60; w++) {
            float x[N_CYC] = { 0 }, a[MAX_H + 1], b[MAX_H + 1];
            for (unsigned n = 0; n < N_WIN; n++) {          /* sample at the commanded rate */
                x[n % N_CYC] += (float)(325.0 * cos(6.283185307 * fg[t] * tim) / CYC_PER_WIN);
                tim += 1.0 / (N_CYC * pll_f_cmd);
            }
            dft(x, a, b);
            pll_update(a[1], b[1], true);
            ph_last = atan2(-b[1], a[1]);
        }
        CHECK(fabs(pll_f_est - fg[t]) < 0.005 && fabs(ph_last) < 0.02,
              "grid %.2f Hz -> measured %.3f Hz, phase error %.1f deg", fg[t], pll_f_est, ph_last * 57.3);
    }

    printf("\nSensor A Butterworth low-pass (%.0f Hz)\n", (double)A_LPF_HZ);
    {   /* filter a harmonic-rich signal sample by sample, fold, DFT, undo H */
        cpx_t src[MAX_H + 1] = { { 0, 0 } };
        for (unsigned h = 1; h <= MAX_H; h += 2) src[h] = c_mk((float)(1.0 / h), (float)(0.3 / h));
        float one[N_CYC], fold[N_CYC] = { 0 }, a[MAX_H + 1], b[MAX_H + 1];
        synth(src, one, 0.0);
        float z1 = 0, z2 = 0;
        for (unsigned n = 0; n < 40 * N_CYC; n++) {            /* settle, then keep last 5 cycles */
            float x = one[n % N_CYC], y = a_lpf.b0 * x + z1;
            z1 = a_lpf.b1 * x - a_lpf.a1 * y + z2;
            z2 = a_lpf.b2 * x - a_lpf.a2 * y;
            if (n >= 35 * N_CYC) fold[n % N_CYC] += y / CYC_PER_WIN;
        }
        dft(fold, a, b);
        double err = 0, att15 = sqrt(c_abs2(a_lpf_h[15]));
        for (unsigned h = 1; h <= MAX_H; h += 2) {
            cpx_t x = c_div(c_phasor(a[h], b[h]), a_lpf_h[h]);
            err = fmax(err, sqrt(c_abs2(c_sub(x, src[h])) / c_abs2(src[h])));
        }
        CHECK(err < 1e-3, "filter undone per harmonic: worst error %.4f %% (h15 gain %.2f, %.0f deg)",
              100 * err, att15, atan2(a_lpf_h[15].im, a_lpf_h[15].re) * 57.3);
        float noise[N_CYC] = { 0 };
        z1 = z2 = 0;
        double in2 = 0, out2 = 0;
        for (unsigned n = 0; n < 200000; n++) {
            float x = (float)gauss(), y = a_lpf.b0 * x + z1;
            z1 = a_lpf.b1 * x - a_lpf.a1 * y + z2;
            z2 = a_lpf.b2 * x - a_lpf.a2 * y;
            in2 += x * x; out2 += y * y;
        }
        (void)noise;
        CHECK(out2 / in2 < 0.4, "white sensor noise reduced to %.0f %% rms", 100 * sqrt(out2 / in2));
    }

    printf("\n%s\n", fails ? "SOME CHECKS FAILED" : "all checks passed");
    return fails != 0;
}
