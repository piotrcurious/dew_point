# ESP32 Dew Point Monitor Pro

A high-precision dew point monitoring and calibration system ported to the ESP32 platform.

## Features
- **Multitasking Core:** Uses FreeRTOS to separate real-time sensor control (Core 0) from heavy optimization (Core 1).
- **Hybrid Calibration:** Combines Monte Carlo global search with Nelder-Mead Simplex refinement for contaminant factor estimation (CO2, SO2, NO2).
- **Standalone Dashboard:** Embedded Three.js WebGL interface for real-time 3D data visualization. No internet required (AP mode compatible).
- **Advanced Physics:** Adsorption model with airflow factor correction linked to cooling intensity.
- **Hardware Safety:** PID thermal control with supply voltage droop compensation and battery monitoring.

## Project Structure
- `dew_point_monitor.ino`: Entry point and main FreeRTOS task orchestration.
- `Dashboard`: WebServer and WebSocket telemetry handler.
- `Optimization`: Nelder-Mead and Monte Carlo optimization engine.
- `Physics`: Magnus-Tetens dew point calculation and gas adsorption models.
- `Sensors`: ADC sampling, health checks, and EMA filtering.
- `ThermalControl`: PID loop and Peltier/Fan driver management.
- `config.h`: Centralized hardware and algorithm configuration.
- `three_js_bundle.h`: Minified and gzipped Three.js library for local hosting.

## Historical Archive
See the `old/` directory for the evolution of the project from the initial mock scripts to the final modular implementation.
