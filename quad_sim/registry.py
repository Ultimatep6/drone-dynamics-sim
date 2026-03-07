"""
registry.py  –  Global class registry for the TOML builder
============================================================

This module lives at a **neutral** location so that both the ABCs
(``quad_sim.bases.*``) and the builder (``quad_sim.toml.builder``)
can import it without creating circular dependencies.

Concrete classes are registered automatically by ``__init_subclass__``
hooks in the ABCs.  The builder calls ``_resolve()`` at build-time to
look up a class by slot and name.
"""

from __future__ import annotations


# slot → {class_name: class}
_REGISTRY: dict[str, dict[str, type]] = {
    "drone":        {},
    "motor":        {},
    "dynamics":     {},
    "pilot":        {},
    "allocator":    {},
    "controller":   {},
    "integrator":   {},
    "environment":  {},
    "effect":       {},
    "constraint":   {},
    "constraints":  {},     # the ConstraintBase container
    "flight-mode":  {},
}


def _register(slot: str, cls: type) -> None:
    """
    Insert *cls* into the registry under *slot*.

    Called automatically by each ABC's ``__init_subclass__``.
    """
    if slot not in _REGISTRY:
        raise ValueError(
            f"Unknown slot '{slot}'. "
            f"Valid slots: {sorted(_REGISTRY.keys())}"
        )
    _REGISTRY[slot][cls.__name__] = cls


def _resolve(slot: str, name: str | None = None) -> type:
    """
    Look up a registered class by slot and (optional) name.

    If *name* is ``None``, returns the single registered class for that
    slot.  Raises if the slot is empty or ambiguous.
    """
    bucket = _REGISTRY[slot]
    if not bucket:
        raise RuntimeError(
            f"No classes registered for slot '{slot}'. "
            f"Did you import / define a concrete subclass?"
        )
    if name is not None:
        if name not in bucket:
            raise ValueError(
                f"'{name}' is not registered under '{slot}'. "
                f"Available: {list(bucket.keys())}"
            )
        return bucket[name]
    if len(bucket) == 1:
        return next(iter(bucket.values()))
    raise ValueError(
        f"Multiple classes registered for '{slot}': {list(bucket.keys())}. "
        f"Specify which one in [classes] of your TOML."
    )
