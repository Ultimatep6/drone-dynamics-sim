def topLevel():
    """
    Decorator that attaches HDF5 group metadata to a dataclass.
    """

    def decorator(cls):
        # Attach metadata to the class itself
        setattr(cls, "__topLevel__", True)
        return cls  # IMPORTANT: return the class unchanged

    return decorator

def setpointConstraint():
    """
    Decorator that attaches setpoint constraint metadata to a dataclass.
    """

    def decorator(cls):
        # Attach metadata to the class itself
        setattr(cls, "__setpointConstraint__", True)
        return cls  # IMPORTANT: return the class unchanged

    return decorator

def stateConstraint():
    """
    Decorator that attaches state constraint metadata to a dataclass.
    """

    def decorator(cls):
        # Attach metadata to the class itself
        setattr(cls, "__stateConstraint__", True)
        return cls  # IMPORTANT: return the class unchanged

    return decorator