# RangeTestSoftware

LoRa range testing firmware for Raspberry Pi Pico2. Supports S-Band (SX1280, 2.4 GHz) and UHF (RFM98PW, 438 MHz).

## Build

```bash
cd build && cmake .. && make <target>
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
python3 tools/image_txrx.py receive --port /dev/tty.usbmodemXXXX

python3 tools/image_txrx.py send --port /dev/tty.usbmodemXXXX --image photo.jpg
```

Deps: `pip install pyserial Pillow`
