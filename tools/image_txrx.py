"""
LoRa S-Band image transfer tool.
River Dowdy - Winter 2025

Dependencies: pip install pyserial Pillow matplotlib rich
"""
from rich.console import Console
from rich.text import Text

from _serial_bridge import pick_serial_port, open_serial, APPLE
from _transfer import cmd_send, cmd_receive

console = Console(highlight=False)

BANNER = r"""                    .                            .--------'  .--------'             .-.                                  .--------'
..-.     .-.        /                            (_)   /     (_)   /    /           (_) )-.                              (_)   /            /
   )   (    .-.   / .-.  .-._..  .-. .-.   .-.       /.-._.      /    /-.   .-.       /   \  .-.  .  .-.    .-.    .-.       /  .-.  . ---/---
  /     \ ./.-'_ / (    (   )  )/   )   )./.-'_     /(   )      /    /   |./.-'_     /     )(  |   )/   )  (   ) ./.-'_     / ./.-'_/ \  /
 (   .   )(__.'_/_.-`---'`-'  '/   /   ( (__.'   .-/._`-'    .-/.__.'    |(__.'   .-/  `--'  `-'-''/   (    `-/-'(__.'   .-/._(__.'/ ._)/
  `-' `-'                               `-'     (_/  `-     (_/  `-              (_/     `-._)          `--._/          (_/  `-   /             """


def _print_banner():
    for line in BANNER.split("\n"):
        console.print(Text(line))


def _menu_item(num, label):
    c = APPLE[(num - 1) % len(APPLE)]
    console.print(f"  [bold {c}]{num})[/]  {label}")


def main():
    _print_banner()

    port = pick_serial_port()
    ser = open_serial(port)
    if not ser:
        return

    console.print(f"  Connected: {port}")

    while True:
        console.print()
        _menu_item(1, "Send image")
        _menu_item(2, "Receive image")
        _menu_item(3, "Change serial port")
        _menu_item(4, "Quit")

        choice = input("\n  Select mode: ").strip()

        if choice == "1":
            try:
                cmd_send(ser)
            except Exception as e:
                console.print(f"  Error: {e}")
        elif choice == "2":
            cmd_receive(ser)
        elif choice == "3":
            ser.close()
            port = pick_serial_port()
            ser = open_serial(port)
            if not ser:
                return
            console.print(f"  Connected: {port}")
        elif choice == "4":
            break
        else:
            console.print("  Invalid choice.")

    ser.close()
    console.print("  Goodbye.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        console.print("\n  Stopped.")
