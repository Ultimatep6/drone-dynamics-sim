import numpy as np


def loggerGroup(group_path: str):
    """
    Decorator that attaches HDF5 group metadata to a dataclass.
    """

    def decorator(cls):
        # Attach metadata to the class itself
        setattr(cls, "__loggerGroup__", group_path)
        return cls  # IMPORTANT: return the class unchanged

    return decorator


def subsystem(name: str):
    def wrapper(cls):
        cls._subsystem_name = name

        def _validate_export_log(self, data: dict):
            """
            Validate that the exported log data matches the subsystem schema.

            Checks:
            - All schema fields are present in data
            - No extra fields are exported
            - Values match expected dtype
            - Arrays match expected shape (optional)
            """

            schema = self.logSchema  # {field_name: FieldSpec(dtype, unit)}

            # --- 1. Check for missing fields ---
            missing = set(schema.keys()) - set(data.keys())
            if missing:
                raise ValueError(
                    f"Subsystem '{self._subsystem_name}' missing fields in export_log: {missing}"
                )

            # --- 2. Check for unknown fields ---
            unknown = set(data.keys()) - set(schema.keys())
            if unknown:
                raise ValueError(
                    f"Subsystem '{self._subsystem_name}' exported unknown fields: {unknown}"
                )

            # --- 3. Check dtype compatibility ---
            for field, value in data.items():
                expected_dtype = schema[field]["dtype"]

                # Scalars
                if not isinstance(value, (np.generic, np.ndarray)):
                    # Convert Python scalars to numpy dtype for checking
                    try:
                        np_value = np.array(value, dtype=expected_dtype)
                    except Exception:
                        raise TypeError(
                            f"Subsystem '{self._subsystem_name}' field '{field}' "
                            f"expected dtype {expected_dtype}, got incompatible value {value!r}"
                        )
                    continue

                # NumPy scalar
                if isinstance(value, np.generic):
                    if value.dtype != expected_dtype:
                        raise TypeError(
                            f"Subsystem '{self._subsystem_name}' field '{field}' "
                            f"expected dtype {expected_dtype}, got {value.dtype}"
                        )
                    continue

                # NumPy array
                if isinstance(value, np.ndarray):
                    if value.dtype != expected_dtype:
                        raise TypeError(
                            f"Subsystem '{self._subsystem_name}' field '{field}' "
                            f"expected dtype {expected_dtype}, got {value.dtype}"
                        )
                    # Optional: enforce fixed shape
                    # expected_shape = schema[field].shape
                    # if expected_shape and value.shape != expected_shape:
                    #     raise ValueError(
                    #         f"Subsystem '{self._subsystem_name}' field '{field}' "
                    #         f"expected shape {expected_shape}, got {value.shape}"
                    #     )
                    continue

                raise TypeError(
                    f"Subsystem '{self._subsystem_name}' field '{field}' has unsupported type {type(value)}"
                )

        # 🔥 Attach the method to the class
        cls._validate_export_log = _validate_export_log

        return cls

    return wrapper
