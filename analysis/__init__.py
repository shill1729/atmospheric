"""Repeatable calibration analysis for the atmospheric SDE/PDE forward model.

Modules:
  io       - load the real PM2.5 datasets into a standardized frame
  scales   - infer spatial/temporal/concentration/wind scales from real data
  forward_model - a numpy replica of the C++ PDE step, used to translate
                  those real-world scales into simulator config defaults
  report   - orchestrates io + scales + forward_model into one calibration report
"""
