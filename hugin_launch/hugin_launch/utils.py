from math import pi
from launch.actions import DeclareLaunchArgument


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


def radians(deg: float) -> float:
    """Convert degrees to radians."""
    return deg / 180 * pi


def degrees(rad: float) -> float:
    """Convert radians to degrees."""
    return rad / pi * 180


def DeclareLaunchArgumentWithNewDefault(
    base: DeclareLaunchArgument, default_value: str
) -> DeclareLaunchArgument:
    """Create DeclareLaunchArgument with new default value."""
    return DeclareLaunchArgument(
        base.name,
        default_value=default_value,
        description=base.description,
        condition=base.condition,
    )
