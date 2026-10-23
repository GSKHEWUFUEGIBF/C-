/* PC simulation of the capacitor relay switching in ../main.c.
 * relay_bsrr(), relay_isr_step(), relay_service() and locks_read() are
 * copied from main.c by run_sim.sh and run against a model of the mains,
 * the relay's mechanical delays, the opening arc and the capacitor's
 * trapped charge (no bleeder resistor).                                   */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

static struct { volatile uint32_t BSRR; } gpiob;
#define GPIOB (&gpiob)
#define __disable_irq() ((void)0)
#define __enable_irq()  ((void)0)

#include "core.h"

static volatile uint8_t lock_mask, lock_state;
static uint8_t  relay_state, relay_target;
static uint32_t relay_off_ms[3], relay_on_ms[3];
static volatile uint8_t relay_pend_on, relay_pend_on0, relay_pend_off;
static uint8_t relay_v_known;
static bool     grid_v_present = true;
static uint32_t settle_win;
static void acc_reset(void) {}

#include "relay_fns.h"

/* ---------------- physical model -------------------------------------- */
#define VPK     325.0
#define I_ARC   0.4          /* below this the contacts part without an arc */

typedef struct {
    double c_uf, t_op, t_rel;        /* true relay delays [ms]          */
    int    pin_hi;                   /* GPIO level                      */
    long   coil_change;              /* sample of the last coil change  */
    int    coil_on, closed;
    double v_cap;                    /* trapped voltage when open       */
    int    closes;
    double dv_max, dv_sum;
} relay_t;

static double v_at(long n) { return VPK * cos(6.283185307 * (n % N_CYC) / N_CYC); }
static double ic_at(long n, double c_uf)
{
    return -c_uf * 1e-6 * VPK * 6.283185307 * 50 * sin(6.283185307 * (n % N_CYC) / N_CYC);
}

static void apply_bsrr(relay_t *r, int i, long n)
{
    uint32_t w = gpiob.BSRR;
    if (w & (1u << i))        r->pin_hi = 1;
    if (w & (1u << (i + 16))) r->pin_hi = 0;
    int coil = (RELAY_SSR_MASK & (1u << i)) ? r->pin_hi : !r->pin_hi;   /* module: LOW = ON */
    if (coil != r->coil_on) { r->coil_on = coil; r->coil_change = n; }
}

static void contacts(relay_t *r, long n)
{
    double d = (n - r->coil_change) / 10.0;               /* ms since coil change */
    if (r->coil_on && !r->closed && d >= r->t_op) {
        double dv = fabs(v_at(n) - r->v_cap);
        r->closes++; r->dv_sum += dv; if (dv > r->dv_max) r->dv_max = dv;
        r->closed = 1;
    } else if (!r->coil_on && r->closed && d >= r->t_rel) {
        r->closed = 0;
        if (fabs(ic_at(n, r->c_uf)) > I_ARC) {             /* arc runs to current zero = next V peak */
            int k = (int)(n % N_CYC);
            r->v_cap = (k > N_CYC / 2) ? VPK : -VPK;
        } else {
            r->v_cap = v_at(n);
        }
    }
}

/* user clicks manual ON/OFF for relay `idx` every `period_ms`, for `secs` */
static relay_t run(int idx, double c_uf, double t_op, double t_rel, int period_ms, int secs, unsigned seed)
{
    relay_t r = { c_uf, t_op, t_rel, 1, 0, 0, 0, 0.0, 0, 0, 0 };
    memset(relay_off_ms, 0, sizeof relay_off_ms); memset(relay_on_ms, 0, sizeof relay_on_ms);
    relay_state = relay_target = 0; relay_pend_on = relay_pend_on0 = relay_pend_off = 0;
    relay_v_known = 0;
    pll_locked = 1;
    srand(seed);
    uint32_t next_click = 20000 + rand() % 20;              /* start after 20 s */
    int want = 0;
    lock_mask = (uint8_t)(1u << idx); lock_state = 0;
    for (long n = 0; n < (long)(20 + secs) * 10000; n++) {
        uint32_t now = (uint32_t)(n / 10);
        if (now >= next_click && now < 20000u + (uint32_t)secs * 1000u) {
            want ^= 1;
            lock_state = want ? lock_mask : 0;
            next_click = now + (uint32_t)period_ms + (uint32_t)(rand() % 20);   /* random phase */
        }
        relay_isr_step((uint32_t)(n % N_CYC));
        apply_bsrr(&r, idx, n);
        if (n % 10 == 0) { gpiob.BSRR = 0; relay_service(now); apply_bsrr(&r, idx, n); }
        gpiob.BSRR = 0;
        contacts(&r, n);
    }
    return r;
}

int main(void)
{
    int fails = 0;
    printf("Firmware: RELAY_SYNC %u, min ON %u ms, min OFF %u ms, coil ON at k=%u (zero k=%u), OFF at k=%u\n",
           (unsigned)RELAY_SYNC, (unsigned)RELAY_MIN_ON_MS, (unsigned)CAP_RECLOSE_MS,
           (unsigned)RELAY_K_ON, (unsigned)RELAY_K_ON_ZERO, (unsigned)RELAY_K_OFF);
    printf("Scenario: manual ON/OFF click every 1 s for 120 s, no bleeder resistor\n\n");
    printf("  cap     relay delays        closes   worst V across contacts   mean\n");
    const double cs[] = { 10.0, 2.5 };
    double worst10 = 0;
    for (unsigned c = 0; c < 2; c++)
        for (double t_op = 5; t_op <= 9.01; t_op += 2)
            for (double t_rel = 2; t_rel <= 8.01; t_rel += 3) {
                relay_t r = run(c ? 0 : 1, cs[c], t_op, t_rel, 1000, 120, 7);
                printf("  %4.1f uF  on %.0f ms, off %.0f ms   %4d     %5.0f V                 %5.0f V\n",
                       cs[c], t_op, t_rel, r.closes, r.dv_max, r.closes ? r.dv_sum / r.closes : 0);
                if (c == 0 && r.dv_max > worst10) worst10 = r.dv_max;
            }
    printf("\n10 uF worst case over all relay delays: %.0f V across the contacts\n", worst10);
    if (RELAY_SYNC) {
        relay_t r = run(1, 10.0, RELAY_T_OP_MS, RELAY_T_REL_MS, 1000, 120, 7);
        int ok1 = r.closes <= 7, ok2 = worst10 <= 200.0, ok3 = r.dv_max <= 20.0;
        printf("  %s  rapid clicking limited to %d closes in 120 s\n", ok1 ? "PASS" : "FAIL", r.closes);
        printf("  %s  10 uF never sees more than 200 V at closing (relay delays +-2 ms off)\n", ok2 ? "PASS" : "FAIL");
        printf("  %s  with the configured relay delays: %.0f V\n", ok3 ? "PASS" : "FAIL", r.dv_max);
        fails = !ok1 + !ok2 + !ok3;
    }
    return fails;
}
