"""
cli.py  —  Operator CLI for the restaurant delivery robot.

Runs in a background daemon thread so it doesn't block the control loop.
Reads commands from stdin and dispatches to the RobotStateMachine.

Commands:
    deliver <table_id>   — assign a delivery to the table with that AprilTag ID
    status               — print the current robot state
    stop                 — emergency stop (returns to IDLE immediately)
    help                 — list available commands
    quit / exit          — shut down the robot cleanly

Example session:
    Robot CLI ready.  Type 'help' for commands.
    > deliver 3
    Delivery to table 3 assigned.
    > status
    State: APPROACHING
    > stop
    Emergency stop executed.
"""

from __future__ import annotations

import logging
import threading
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from state_machine import RobotStateMachine

logger = logging.getLogger(__name__)

_HELP = """\
Commands:
  deliver <id>   assign delivery to the table with AprilTag ID <id>
  status         show current robot state
  stop           emergency stop — return to IDLE immediately
  help           show this message
  quit / exit    shut down the robot
"""


class DeliveryCLI:
    """
    Reads operator commands from stdin in a daemon thread and dispatches
    them to the RobotStateMachine.
    """

    def __init__(self, state_machine: RobotStateMachine) -> None:
        self._sm = state_machine

    def start(self) -> None:
        """Launch the CLI input loop in a background daemon thread."""
        t = threading.Thread(target=self._loop, name="CLI", daemon=True)
        t.start()

    def _loop(self) -> None:
        print("Robot CLI ready.  Type 'help' for commands.\n")
        while True:
            try:
                raw = input("> ").strip()
            except (EOFError, KeyboardInterrupt):
                break

            if not raw:
                continue

            parts = raw.lower().split()
            cmd   = parts[0]

            if cmd == "deliver":
                self._cmd_deliver(parts)
            elif cmd == "status":
                self._cmd_status()
            elif cmd == "stop":
                self._cmd_stop()
            elif cmd in ("help", "?"):
                print(_HELP)
            elif cmd in ("quit", "exit"):
                print("Shutting down …")
                self._sm.emergency_stop()
                self._sm.shutdown()
                break
            else:
                print(f"Unknown command: '{raw}'.  Type 'help' for a list.")

    def _cmd_deliver(self, parts: list[str]) -> None:
        if len(parts) < 2:
            print("Usage: deliver <table_id>")
            return
        try:
            table_id = int(parts[1])
        except ValueError:
            print(f"Invalid table ID: '{parts[1]}'.  Must be an integer.")
            return

        accepted = self._sm.assign_delivery(table_id)
        if accepted:
            print(f"Delivery to table {table_id} assigned.")
        else:
            print(
                f"Delivery to table {table_id} rejected — "
                f"robot is currently {self._sm.current_state.name}. "
                "Use 'stop' first to abort the current task."
            )

    def _cmd_status(self) -> None:
        print(f"State: {self._sm.current_state.name}")

    def _cmd_stop(self) -> None:
        self._sm.emergency_stop()
        print("Emergency stop executed.")
