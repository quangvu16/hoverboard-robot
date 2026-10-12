"""Where the AI car's data files live (Yahboom hardcoded /home/yahboom/...)."""
import os
import shutil

from ament_index_python.packages import get_package_share_directory

_SHARE = os.path.join(get_package_share_directory('hoverboard_ai_car'), 'data')
_USER = os.path.expanduser('~/.ros/hoverboard_ai_car')


def share_file(name):
    """Read-only data installed with the package (e.g. Haar cascades)."""
    return os.path.join(_SHARE, name)


def user_file(name):
    """A file the nodes WRITE (colour calibration): kept under ~/.ros so a
    rebuild does not wipe it, seeded from the installed default on first use."""
    path = os.path.join(_USER, name)
    if not os.path.exists(path):
        os.makedirs(_USER, exist_ok=True)
        if os.path.exists(share_file(name)):
            shutil.copy(share_file(name), path)
    return path
