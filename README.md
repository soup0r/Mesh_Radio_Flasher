# Mesh Radio Flasher

A resilient, production-grade SWD flasher for mesh radio devices (nRF52840/nRF52833), running on Seeed XIAO ESP32C3.

## Features

- **SWD Programming**: Full read/write/erase capability for nRF52 series flash
- **Web Interface**: Modern UI for device management
- **Power Management**: Deep sleep with WiFi-based wake
- **Auto-Recovery**: Self-healing with watchdog and error logging

## Quick Start

1. Install PlatformIO:
```bash
pip install platformio
```

2. Build and upload:
```bash
pio run -t upload
```

3. Monitor output:
```bash
pio run -t monitor
```

## Documentation

See [docs/](docs/) for detailed documentation.

## License

MIT License
