import random

def get_unique_name(name: str) -> str:
    """Returns the intrinsic yaml path of `vehicle_name`

    Parameters:
    ----------
    `name`: Base name

    Returns:
    ----------
    a string with a random number added to the end of `name`

    Examples:
    ----------
    >>> get_unique_name("rviz2")
    rviz2_1234
    """
    unique_id = random.randint(1000, 9999)
    return f"{name}_{unique_id}"
