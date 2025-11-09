import numpy as np
from scipy.interpolate import interp1d


def interp1(data_x, data_y, interp_x, method='linear', option=None):
    if not isinstance(data_x, list) or not isinstance(data_y, list):
        raise TypeError("data_x and data_y must be 1D lists")
    if len(data_x) != len(data_y):
        raise ValueError("data_x and data_y must have the same length")
    if not isinstance(interp_x, (list, float)):
        raise TypeError("interp_x must be a list or a single value")

    if method not in ['linear', 'pchip', 'spline']:
        raise ValueError("method should be 'linear', 'pchip', or 'spline'")

    # Determine if it is cyclic interpolation
    is_cyclic = False
    if option == 'cyclic':
        is_cyclic = True
        x_range = max(data_x) - min(data_x)

    # Create interpolation function
    f = interp1d(data_x, data_y, kind=method)

    # Perform interpolation for single value or list of values
    if isinstance(interp_x, float):
        if is_cyclic:
            result = f(interp_x % x_range)
        else:
            result = f(interp_x)
    else:
        result = f(interp_x)

    # Handle extrapolation option
    if option is not None and not is_cyclic:
        if option == 'extrap':
            pass  # No need to handle extrapolation
        elif isinstance(option, (int, float)):
            result = np.where(np.logical_or(interp_x < min(data_x), interp_x > max(data_x)), option, result)
        else:
            raise ValueError("option should be 'extrap', 'cyclic', or a specific number")

    # Handle cyclic interpolation
    if is_cyclic:
        result = np.where(interp_x < min(data_x), f((interp_x - min(data_x)) % x_range), result)
        result = np.where(interp_x > max(data_x), f((interp_x - max(data_x)) % x_range), result)

    return result