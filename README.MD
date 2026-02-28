# RangeTestSoftware

LoRa range testing firmware for Raspberry Pi Pico2. Supports S-Band (SX1280) and UHF (RFM98PW).

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
cd tools
python3 -m venv .venv
source .venv/bin/activate
pip install pyserial Pillow matplotlib rich
python3 image_txrx.py
```
