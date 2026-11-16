"""QR mission matching and fixed report TF."""

# Spawned workers may import the source package before catkin's relay package.
# Keep generated msg/srv modules discoverable in both import orders.
from pkgutil import extend_path
__path__ = extend_path(__path__, __name__)
