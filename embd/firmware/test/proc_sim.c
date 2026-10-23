/* PC test of process_window() from ../main.c with synthetic ADC samples:
 * a charger-like spiky current on sensor I, the matching AHF injection
 * (plus sensor noise) on sensor A, through the real Butterworth filter.  */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "core.h"
#include "proc_glob.h"

static volatile uint32_t ahf_active = 1;
static float lpf_z1, lpf_z2;
static bool  lpf_primed;
static win_t mw;

#include "proc_fn.h"

static double gauss(void)
{
    double u1 = (rand() + 1.0) / (RAND_MAX + 2.0), u2 = (rand() + 1.0) / (RAND_MAX + 2.0);
    return sqrt(-2.0 * log(u1)) * cos(6.283185307 * u2);
}

/* charger: 0.13 A spikes near each voltage peak, 1.2 ms wide */
static double i_load(double th)
{
    double s = 0;
    for (int p = 0; p < 2; p++) {
        double d = fmod(th - (p ? 3.14159265 : 0) + 9.42477796, 6.28318531) - 3.14159265;
        s += (p ? -1 : 1) * 0.13 * exp(-d * d / (2 * 0.19 * 0.19));
    }
    return s;
}

static int run(int active, double inj_gain, double *thd_before, double *thd_after, float *apk)
{
    static uint16_t buf[N_WIN * N_CH];
    /* the AHF injects minus the load's h2..h15 (what a converged loop does) */
    float il[N_CYC], a[MAX_H + 1], b[MAX_H + 1], inj[N_CYC] = { 0 };
    for (unsigned k = 0; k < N_CYC; k++) il[k] = (float)i_load(6.28318531 * k / N_CYC);
    dft(il, a, b);
    for (unsigned h = 2; h <= MAX_H; h++)
        for (unsigned k = 0; k < N_CYC; k++)
            inj[k] -= (float)inj_gain * (a[h] * cos_t[(h * k) % N_CYC] + b[h] * sin_t[(h * k) % N_CYC]);
    ahf_active = (uint32_t)active;
    ahf_duty_pk = active ? 0.1f : 0.0f;
    lpf_primed = false;
    for (int w = 0; w < 6; w++) {                       /* let the filter settle */
        for (unsigned n = 0; n < N_WIN; n++) {
            unsigned k = n % N_CYC;
            double v = 325 * cos(6.28318531 * k / N_CYC);
            buf[n * N_CH + SLOT_I] = (uint16_t)lrint(2048 + i_load(6.28318531 * k / N_CYC) / K_I + 4 * gauss());
            buf[n * N_CH + SLOT_V] = (uint16_t)lrint(2048 + v / K_V + 2 * gauss());
            buf[n * N_CH + SLOT_A] = (uint16_t)lrint(2048 + (active ? inj[k] : 0) / K_A + 6 * gauss());
        }
        process_window(buf, &mw);
    }
    float hb[MAX_H + 1], ha_[MAX_H + 1], x[MAX_H + 1], y[MAX_H + 1];
    dft(w_i, x, y);
    for (unsigned h = 1; h <= MAX_H; h++) hb[h] = sqrtf(x[h] * x[h] + y[h] * y[h]);
    dft(w_c, x, y);
    for (unsigned h = 1; h <= MAX_H; h++) ha_[h] = sqrtf(x[h] * x[h] + y[h] * y[h]);
    *thd_before = thd_pct(hb);
    *thd_after  = thd_pct(ha_);
    *apk = mw.apk;
    return 0;
}

int main(void)
{
    int fails = 0;
    double tb, ta;
    float apk;
    tables_init();
    printf("\nSensor A display (charger load, ~0.04 A injection, ACS712 noise)\n");

    run(1, 1.0, &tb, &ta, &apk);
    int ok = apk > 0.05f && ta < 0.35 * tb;
    printf("  %s  AHF driving: I_ahf peak %.3f A shown, grid THD %.0f %% -> %.0f %%\n",
           ok ? "PASS" : "FAIL", apk, tb, ta);
    fails += !ok;

    run(0, 0.0, &tb, &ta, &apk);
    ok = apk == 0.0f && fabs(ta - tb) < 1.0;
    printf("  %s  STANDBY: sensor A noise hidden (peak %.3f A), grid THD unchanged %.0f %%\n",
           ok ? "PASS" : "FAIL", apk, ta);
    fails += !ok;
    return fails;
}
