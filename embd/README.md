# Grid Power Quality Dashboard (APFC + AHF)

Real-time light-theme dashboard for the STM32F401RE firmware.

```
python -m pip install numpy pyserial pyqtgraph PySide6
py dashboard.py --demo          # simulator, no hardware
py dashboard.py                 # auto-detect ST-LINK / COM35
py dashboard.py --port COM35
```

* **Common (always visible):** grid voltage V(t) and grid current I(t)
* **Power Factor Correction tab:** PF gauge, P/Q/S/D, capacitor bank AUTO/MANUAL control, PF and capacitance trends
* **Harmonic Mitigation tab:** THD before/after, AHF tracking, AHF vs target, corrected current, spectra, ADC diagnostics

`dashboard_matplotlib_old.py` is the previous matplotlib version, kept for reference.

## Firmware (`firmware/main.c`)

STM32F401RE, CMSIS only. The AHF uses selective harmonic control:

1. Every 100 ms the DFT gives a phasor per harmonic (h2–h15) for the load current (sensor I) and the injected current (sensor A).
2. Each harmonic has its own closed loop that drives the injected harmonic to `-I_h`.
3. The bridge + coupling-inductor gain is measured per harmonic on line, so amplitude and phase (inductor 1/h roll-off, ~90° lag, PWM delay) are corrected individually.
4. The output table is synthesised from those harmonics only.
5. The sample clock is phase-locked to the grid (exactly 200 samples per real cycle), and the PWM is re-synchronised to every sample.

Before flashing, set in `main.c`:

| Setting | What to check |
|---|---|
| `A_SIGN` | Sensor A must read +ve when the AHF pushes current in the direction sensor I reads +ve. If grid THD **rises** when AHF is ACTIVE, flip it. |
| `AHF_SENSE_GRID` | `0` if sensor I sits on the load side of the injection point, `1` if it measures the already-compensated grid current. |
| `AHF_INJ_RATIO` | Coupling-transformer ratio (PCC amps per sensor-A amp); `1` for direct injection. |
| `AHF_VDC`, `AHF_L_H`, `AHF_R_OHM` | Only a starting guess; the loop measures the real values. |
| `RELAY_SSR_MASK` | Bit set = that pin drives an active-HIGH SSR (default `0x4`: PB2). Clear = 5 V active-LOW relay module. |
| `CAP_RECLOSE_MS` | 10 s minimum OFF time before a capacitor re-closes. Needs a **bleeder resistor across every capacitor** (2.5 µF 820 kΩ, 10 µF 220 kΩ, 20 µF 100 kΩ, 1 W, ≥400 V), otherwise the cap is still charged when the relay closes and the contacts weld. |

Simulation of the control maths on a PC (no hardware): `firmware/test/run_sim.sh`.
