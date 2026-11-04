"""Resolve locations of the trained model files shipped with this package.

Models live in ``models/`` in the source tree and are installed to
``share/fault_detector/models`` by ``colcon build``. Set the environment
variable ``FAULT_DETECTOR_MODEL_DIR`` to load models from somewhere else
(e.g. while iterating on newly trained models without rebuilding).
"""

import os

from ament_index_python.packages import get_package_share_directory


def model_dir() -> str:
    override = os.environ.get('FAULT_DETECTOR_MODEL_DIR')
    if override:
        return override
    return os.path.join(get_package_share_directory('fault_detector'), 'models')


def model_path(*parts: str) -> str:
    """Return the absolute path of a model file, e.g. model_path('IF', 'x.joblib')."""
    return os.path.join(model_dir(), *parts)
