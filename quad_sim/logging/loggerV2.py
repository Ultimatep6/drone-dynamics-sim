from __future__ import annotations

from collections import defaultdict
from typing import Dict

import h5py
import numpy as np
from schema import And, Or, Schema, SchemaError, Use

from quad_sim.utils.logFuncs import normalize_dtype


class NCopterLogger:
    def __init__(self, filepath: str, chunk_size: int = 1024):
        self.filepath = filepath
        self.chunk_size = chunk_size
        self._file = h5py.File(filepath, "w")

        # Root group
        self._sim_group = self._file.create_group("simulation")
        self._drones_group = self._sim_group.create_group("drones")

        # registry[drone_id][subsystem_name] = subsystem_instance
        self.registry: Dict[str, Dict[str, Any]] = defaultdict(dict)

        # datasets[(drone_id, subsystem_name)][field_name] = h5py.Dataset
        self._datasets: Dict[tuple, Dict[str, h5py.Dataset]] = defaultdict(dict)

        # write_index[(drone_id, subsystem_name)] = int
        self._write_index: Dict[tuple, int] = defaultdict(int)

        # capacity[(drone_id, subsystem_name)] = int
        self._capacity: Dict[tuple, int] = defaultdict(int)

    # ---------- registration ----------

    def register_drone(self, drone: "Drone"):
        drone_id = drone.id
        if drone_id in self.registry:
            raise ValueError(f"Drone '{drone_id}' already registered")

        drone_group = self._drones_group.create_group(drone_id)

        subsystems = drone.get_subsystems()
        for subsystem in subsystems:
            name = subsystem._subsystem_name
            if name in self.registry[drone_id]:
                raise ValueError(
                    f"Subsystem '{name}' already registered for drone '{drone_id}'"
                )
            self.registry[drone_id][name] = subsystem
            # Create group for subsystem; datasets created lazily
            drone_group.create_group(name)

    # ---------- internal helpers ----------

    def _ensure_datasets(self, drone_id, subsystem_name, schema, example_data):
        key = (drone_id, subsystem_name)
        if self._datasets[key]:
            return

        subsystem_group = self._drones_group[drone_id][subsystem_name]
        ds_map = {}

        for field_name, spec in schema.items():
            value = example_data[field_name]

            # Determine per-step shape
            if isinstance(value, np.ndarray):
                field_shape = value.shape
            else:
                field_shape = ()  # scalar

            ds = subsystem_group.create_dataset(
                name=field_name,
                shape=(0,) + field_shape,
                maxshape=(None,) + field_shape,
                chunks=(self.chunk_size,) + field_shape,
                dtype=spec["dtype"],
            )
            ds.attrs["unit"] = spec["unit"]
            ds_map[field_name] = ds

        self._datasets[key] = ds_map
        self._capacity[key] = 0
        self._write_index[key] = 0

    def _ensure_capacity(self, key: tuple):
        idx = self._write_index[key]
        cap = self._capacity[key]
        if idx < cap:
            return
        # Need to grow by chunk_size
        new_cap = cap + self.chunk_size
        for ds in self._datasets[key].values():
            ds.resize((new_cap,) + ds.shape[1:])
        self._capacity[key] = new_cap

    # ---------- main logging step ----------

    def step(self):
        for drone_id, subsystems in self.registry.items():
            for subsystem_name, subsystem in subsystems.items():
                schema: Schema = subsystem.get_log_definition()
                self.passSchema(schema)
                key = (drone_id, subsystem_name)

                data = subsystem.export_log()

                subsystem._validate_export_log(data)

                # Lazily create datasets
                self._ensure_datasets(drone_id, subsystem_name, schema, data)

                # Ensure capacity for this write
                self._ensure_capacity(key)
                idx = self._write_index[key]

                # Write each field
                for field_name, value in data.items():
                    ds = self._datasets[key][field_name]
                    ds[idx] = value

                self._write_index[key] += 1

    # ---------- finalization ----------

    def finalize(self):
        # Truncate to final size
        for key, ds_map in self._datasets.items():
            final_size = self._write_index[key]
            for ds in ds_map.values():
                if ds.shape[0] != final_size:
                    ds.resize((final_size,) + ds.shape[1:])
        self._file.flush()
        self._file.close()

    # Data scheme for passing logging requests to the logger pre-runtime
    def passSchema(self, logSchema):
        schema = Schema(
            {
                str: {
                    "dtype": Use(normalize_dtype),
                    "unit": And(str, Use(lambda a: "N/A" if a == "" else a)),
                }
            }
        )
        try:
            schema.validate(logSchema)
        except SchemaError as e:
            raise e
        return logSchema
