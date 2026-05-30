# RangeTestSoftware

LoRa range testing firmware for Raspberry Pi Pico2. Supports S-Band (SX1280) and UHF (RFM98PW).

## Build

```bash
cd build && cmake .. && ninja <target>
```

| Target | Description |
|--------|-------------|
| `RangeTestNodeA` | Range test — Transmitter |
| `RangeTestNodeB` | Range test — Receiver |
| `RangeTestObserver` | Range test — Observer |
| `ImageTransfer` | Image TX/RX over S-Band |
| `ManualRadioBringupTest` | Interactive radio debug |
| `UHFConstantFullPowerTx` | UHF stress test |

## Image Transfer

Flash `ImageTransfer.uf2` on both Picos.

```bash
cd tools
python3 -m venv .venv
source .venv/bin/activate
pip install pyserial Pillow matplotlib rich
python3 image_txrx.py
```

### Via Raspberry Pi

`ImageTransfer` also registers the RPi UART as a stdio transport, so the same
protocol works over the wired RPi link in addition to USB. Pins mirror the
samwise picubed board: `uart0` on GPIO31 (TX) / GPIO30 (RX), with a power-enable
line on GPIO29. Run `image_txrx.py` on the Pi and enter the UART device path
(e.g. `/dev/serial0`) at the port prompt.

Note: GPIO30/31 only exist on the RP2350B (picubed); a stock `pico2` (RP2350A)
exposes GPIO0-29 only.
