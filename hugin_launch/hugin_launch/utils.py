from math import pi
from launch.actions import OpaqueFunction, Shutdown
from launch_ros.ros_adapters import get_ros_adapter


def boolean(arg: str):
    """Convert a string to a boolean value."""
    normed = arg.strip().lower()
    if normed == "true":
        return True
    elif normed == "false":
        return False
    else:
        raise ValueError(f"Invalid boolean string: {arg}. Expected 'true' or 'false'.")


def integer_list(arg: str) -> list[int]:
    """Convert a comma-separated string to a list of integers."""
    return [int(i) for i in arg.split(",")]


def float_list(arg: str) -> list[float]:
    """Convert a comma-separated string to a list of floats."""
    return [float(i) for i in arg.split(",")]


def string_list(arg: str) -> list[str]:
    """Convert a comma-separated string to a list of strings."""
    return [i.strip() for i in arg.split(",") if i.strip()]


def radians(deg: float) -> float:
    """Convert degrees to radians."""
    return deg / 180 * pi


def degrees(rad: float) -> float:
    """Convert radians to degrees."""
    return rad / pi * 180


def ShutdownFailure(reason: str) -> OpaqueFunction:
    """Create an OpaqueFunction that raises a RuntimeError on shutdown."""

    def throw_shutdown_error(context, reason: str):
        """Raise a RuntimeError with the given reason."""
        if get_ros_adapter(context)._ROSAdapter__is_running:
            raise RuntimeError(reason)

    return OpaqueFunction(
        function=lambda context: throw_shutdown_error(context, reason)
    )


def ShutdownClean(reason: str) -> OpaqueFunction:
    """Create an OpaqueFunction that shuts down the ROS adapter on shutdown."""

    def clean_shutdown(context):
        """Shutdown the ROS adapter cleanly."""
        if get_ros_adapter(context)._ROSAdapter__is_running:
            return [Shutdown(reason=reason)]

    return OpaqueFunction(function=clean_shutdown)
