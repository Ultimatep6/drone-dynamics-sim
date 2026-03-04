def agentConfig():
    """
    Decorator that attaches HDF5 group metadata to a dataclass.
    """

    def decorator(cls):
        # Attach metadata to the class itself
        setattr(cls, "__topLevel__", True)
        return cls  # IMPORTANT: return the class unchanged

    return decorator