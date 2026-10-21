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
