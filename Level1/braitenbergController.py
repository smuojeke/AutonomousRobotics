def braitenbergController(leftIntensity, rightIntensity):
    """Braitenberg controller with inhibitory connections between same-side sensors and motors."""

    # A parameter governing the conversion between light intensity values and motor speeds.
    # 30.0 is a rather arbitrary constant representing the top speed at maximum intensity.
    # 255 is the maximum image intensity.
    k = 30.0 / 255

    leftSpeed = k * (255 - leftIntensity)
    rightSpeed = k * (255 - rightIntensity)

    return leftSpeed, rightSpeed