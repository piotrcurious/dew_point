# Historical Versions Documentation

This directory contains the evolution of the Dew Point Monitor project. Each version represents a significant step in the development of the calibration and control logic.

## Version List

1.  **active.ino**: Initial implementation. Basic Magnus-Tetens dew point calculation and linear cooling control.
2.  **active2.ino**: Introduced contaminant adjustment factors (CO2, SO2, NO2) and early empirical data collection.
3.  **active3.ino**: Added non-linear gas response models and pressure correction.
4.  **active4.ino**: Implemented a basic quadratic least-squares fit for calibration.
5.  **active5.ino**: First iteration of the Monte Carlo search algorithm for contaminant detection.
6.  **active6.ino**: Refined Monte Carlo with chronological weighting and improved PWM logic.
7.  **active6_refactor.ino**: Structural cleanup, adding better sensor abstractions and improved cooling profiles.
8.  **active7_research.ino**: Exploratory version testing multi-pass Monte Carlo and advanced thermal mass modeling.
9.  **monte_carlo.ino / .md**: Standalone research and documentation for the Monte Carlo optimization engine.

---
*Note: The current production-ready version is `dew_point_monitor.ino` in the root directory.*
