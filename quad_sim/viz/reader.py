import h5py
import matplotlib.pyplot as plt
import numpy as np


def print_hdf5_contents(path):
    with h5py.File(path, "r") as f:
        print(f"\n=== FILE: {path} ===\n")
        _print_group(f, indent=0)


def _print_group(group, indent):
    prefix = "  " * indent

    for name, item in group.items():
        if isinstance(item, h5py.Group):
            print(f"{prefix}[GROUP] {name}/")
            _print_group(item, indent + 1)

        elif isinstance(item, h5py.Dataset):
            print(f"{prefix}[DATASET] {name}")
            print(f"{prefix}  shape: {item.shape}, dtype: {item.dtype}")

            try:
                data = item[...]
                print(f"{prefix}  values:\n{prefix}  {data}\n")
            except Exception as e:
                print(f"{prefix}  <ERROR reading dataset: {e}>\n")

        else:
            print(f"{prefix}[UNKNOWN] {name} ({type(item)})")


def plot_field(paths, drone, subsystem, field):
    """
    Plot a single dataset from multiple HDF5 logs.
    If all datasets have the same shape, plot them together:
        - Scalars: one line per database
        - Vectors: one subplot per component, each showing N lines
        - Matrices: flatten each matrix and plot each component in its own subplot
    """

    # Normalize input to list
    if isinstance(paths, str):
        paths = [paths]

    datasets = []
    shapes = []

    # Load all datasets
    for path in paths:
        with h5py.File(path, "r") as f:
            try:
                ds = f["simulation/drones"][drone][subsystem][field]
            except KeyError:
                print(f"[WARN] Path not found in {path}: {drone}/{subsystem}/{field}")
                continue

            data = ds[...]
            datasets.append((path, data))
            shapes.append(data.shape)

    if not datasets:
        print("No valid datasets found.")
        return

    # Check shape compatibility
    first_shape = shapes[0]
    for s in shapes:
        if s != first_shape:
            print("Datasets have incompatible shapes:")
            for (p, d), s in zip(datasets, shapes):
                print(f"  {p}: {s}")
            return

    print(f"Loaded {len(datasets)} datasets for {drone}/{subsystem}/{field}")
    print(f"  shape: {first_shape}")

    data_dim = len(first_shape)

    # --- Scalar time series (N,) ---
    if data_dim == 1:
        plt.figure()
        for path, data in datasets:
            plt.plot(data, label=path)
        plt.title(f"{field} (scalar)")
        plt.xlabel("timestep")
        plt.ylabel(field)
        plt.legend()
        plt.grid(True)
        plt.show()
        return

    # --- Vector time series (N, D) ---
    if data_dim == 2:
        N, D = first_shape
        fig, axes = plt.subplots(D, 1, figsize=(8, 3 * D), sharex=True)

        if D == 1:
            axes = [axes]

        for i in range(D):
            for path, data in datasets:
                axes[i].plot(data[:, i], label=path)
            axes[i].set_title(f"{field}[{i}]")
            axes[i].grid(True)
            axes[i].legend()

        axes[-1].set_xlabel("timestep")
        plt.tight_layout()
        plt.show()
        return

    # --- Matrix time series (N, A, B) ---
    if data_dim == 3:
        N, A, B = first_shape
        flat_dim = A * B

        fig, axes = plt.subplots(flat_dim, 1, figsize=(8, 3 * flat_dim), sharex=True)

        if flat_dim == 1:
            axes = [axes]

        for i in range(flat_dim):
            for path, data in datasets:
                flat = data.reshape(N, flat_dim)
                axes[i].plot(flat[:, i], alpha=0.7, label=path)
            axes[i].set_title(f"{field}[{i}] (flattened)")
            axes[i].grid(True)
            axes[i].legend()

        axes[-1].set_xlabel("timestep")
        plt.tight_layout()
        plt.show()
        return

    print(f"Unsupported dataset rank: {data_dim}")


def plot_comparison(
    paths: list, drone: str, subsystems: list, field_a: str, field_b: str
):
    """
    Plot two fields together (e.g., velocity vs velocity_setpoint).
    Supports multiple databases.
    """

    # Load both fields from each database
    def load_field(path, field, subsystem):
        with h5py.File(path, "r") as f:
            return f["simulation/drones"][drone][subsystem][field][...]

    datasets_a = [(path, load_field(path, field_a, subsystems[0])) for path in paths]
    datasets_b = [(path, load_field(path, field_b, subsystems[1])) for path in paths]

    # Check shape compatibility
    shape_a = datasets_a[0][1].shape
    shape_b = datasets_b[0][1].shape

    if shape_a[0] != shape_b[0]:
        print("Number of logs are incompatible")
        print(f"{field_a}: {shape_a}")
        print(f"{field_b}: {shape_b}")
        return

    data = datasets_a[0][1]
    ndim = data.ndim

    # --- Scalar ---
    if ndim == 1:
        plt.figure()
        for (p, da), (_, db) in zip(datasets_a, datasets_b):
            plt.plot(da, label=f"{p} - {field_a}")
            plt.plot(db, label=f"{p} - {field_b}")
        plt.title(f"{field_a} vs {field_b}")
        plt.xlabel("timestep")
        plt.grid(True)
        plt.legend()
        plt.show()
        return

    # --- Vector ---
    if ndim == 2:
        N, D = data.shape
        fig, axes = plt.subplots(D, 1, figsize=(8, 3 * D), sharex=True)

        if D == 1:
            axes = [axes]

        for i in range(D):
            for (p, da), (_, db) in zip(datasets_a, datasets_b):
                axes[i].plot(da[:, i], label=f"{p} - {field_a}")
                axes[i].plot(db[:, i], label=f"{p} - {field_b}")
            axes[i].set_title(f"Component {i}")
            axes[i].grid(True)
            axes[i].legend()

        axes[-1].set_xlabel("timestep")
        plt.tight_layout()
        plt.show()
        return

    # --- Matrix ---
    if ndim == 3:
        N, A, B = data.shape
        flat_dim = A * B
        fig, axes = plt.subplots(flat_dim, 1, figsize=(8, 3 * flat_dim), sharex=True)

        if flat_dim == 1:
            axes = [axes]

        for i in range(flat_dim):
            for (p, da), (_, db) in zip(datasets_a, datasets_b):
                fa = da.reshape(N, flat_dim)
                fb = db.reshape(N, flat_dim)
                axes[i].plot(fa[:, i], label=f"{p} - {field_a}")
                axes[i].plot(fb[:, i], label=f"{p} - {field_b}")
            axes[i].set_title(f"Matrix component {i}")
            axes[i].grid(True)
            axes[i].legend()

        axes[-1].set_xlabel("timestep")
        plt.tight_layout()
        plt.show()
        return
