from dataclasses import field
from typing import List

import h5py
import numpy as np


def create_schema(fields, dtypes, units):
    # Validate lengths
    if len({len(fields), len(dtypes), len(units)}) != 1:
        raise ValueError("fields, dtypes, and units must have the same length")

    # Validate strings
    if not all(isinstance(x, str) for lst in (fields, dtypes, units) for x in lst):
        raise ValueError("All fields, dtypes, and units must be strings")

    # Convert dtype strings → numpy dtypes
    dtype_map = {
        "float": np.float64,
        "double": np.float64,
        "int": np.int64,
        "bool": np.bool_,
        "str": h5py.string_dtype(encoding="utf-8"),
    }

    schema = {}
    for f, dt, u in zip(fields, dtypes, units):
        if dt not in dtype_map:
            try:
                dt = np.dtype(dt)
            except TypeError as e:
                raise e
        else:
            dt = dtype_map[dt]
        schema[f] = {"dtype": dt, "unit": u}
    return schema


def normalize_dtype(d):
    dt = np.dtype(d)
    if dt.kind in ("U", "S"):  # unicode or byte string
        return h5py.string_dtype(encoding="utf-8")
    return dt
