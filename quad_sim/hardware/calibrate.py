from abc import ABC, abstractmethod
from inputs import devices, get_gamepad
import time
import threading


class GamePadBase(ABC):
    def __init__(self):
        self.keys = {}  # ID: str -> tuple(min, max)

    def register(self) -> bool:
        """
        Registers the gamepad device to be used for controller input.
        This method should identify and connect to the appropriate gamepad hardware,
        ensuring it is ready for use in the simulation.

        :return: Boolean confirmation of successful registration.
        :rtype: bool
        """
        input("Please connect your gamepad and press Enter to start calibration...")
        gp = get_gamepad()
        if not gp:
            print("No gamepads detected. Please connect a gamepad and try again.")
            return False

        print(f"Gamepad detected: {devices.gamepads[0]}")
        input(
            "After pressing Enter, move the joysticks in all directions "
            "(especially the extremes) to help calibrate the input ranges. "
            "Ensure all buttons and triggers are also pressed to register "
            "their inputs.\nTO STOP CALIBRATION, WAIT 5s\n"
        )

        gamepad = devices.gamepads[0]
        last_event_time = time.time()
        last_values = {}       # track last state per code to suppress duplicates
        running = True

        def _read_events():
            nonlocal last_event_time, running
            while running:
                try:
                    events = gamepad.read()
                    for event in events:
                        if event.ev_type == "Sync":
                            continue

                        # Only process if the value actually changed
                        prev = last_values.get(event.code)
                        if prev == event.state:
                            continue
                        last_values[event.code] = event.state

                        print(f"  {event.ev_type:<12} {event.code:<20} {event.state}")

                        # Track min/max for axes
                        if event.code.startswith("ABS_"):
                            lo, hi = self.keys.get(event.code, (event.state, event.state))
                            self.keys[event.code] = (min(lo, event.state), max(hi, event.state))
                        else:
                            # Register buttons/keys with their value
                            self.keys.setdefault(event.code, (0, 1))

                        last_event_time = time.time()
                except Exception:
                    break

        reader = threading.Thread(target=_read_events, daemon=True)
        reader.start()

        # Main thread monitors for 5s of inactivity
        while True:
            time.sleep(0.5)
            if time.time() - last_event_time > 5:
                running = False
                break

        print("\n--- Calibration complete ---")
        print(f"{'Code':<20} {'Min':>10} {'Max':>10}")
        print("-" * 42)
        for code, (lo, hi) in sorted(self.keys.items()):
            print(f"{code:<20} {lo:>10} {hi:>10}")

        return True


if __name__ == "__main__":

    class TestPad(GamePadBase):
        pass

    gamepad = TestPad()
    gamepad.register()

    