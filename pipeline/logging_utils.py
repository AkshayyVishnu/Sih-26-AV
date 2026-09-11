"""
Central logging setup. Every pipeline stage logs through this so a single
run produces one chronological log file covering every decision made --
detections dropped, fusion fallbacks used, tracker assignments, predictor
choice, planner replans -- per the requirement to keep a decision trail,
not just final output.
"""
from __future__ import annotations

import logging
import os
from datetime import datetime

LOG_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "logs")


def get_logger(name: str = "pipeline") -> logging.Logger:
    """Returns a logger that writes to both console and a per-run log file
    under logs/. Call once per process; submodules should use
    logging.getLogger("pipeline.<stage>") to inherit this configuration.
    """
    os.makedirs(LOG_DIR, exist_ok=True)
    logger = logging.getLogger(name)
    if logger.handlers:
        # Already configured (e.g. re-imported) -- don't add duplicate handlers.
        return logger

    logger.setLevel(logging.DEBUG)

    run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = os.path.join(LOG_DIR, f"run_{run_id}.log")

    file_handler = logging.FileHandler(log_path, encoding="utf-8")
    file_handler.setLevel(logging.DEBUG)

    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.INFO)

    fmt = logging.Formatter(
        "%(asctime)s.%(msecs)03d | %(levelname)-7s | %(name)s | %(message)s",
        datefmt="%H:%M:%S",
    )
    file_handler.setFormatter(fmt)
    console_handler.setFormatter(fmt)

    logger.addHandler(file_handler)
    logger.addHandler(console_handler)

    logger.info("Log file for this run: %s", log_path)
    return logger
