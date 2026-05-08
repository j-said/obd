# OBD Rust

A Rust project for ESP microcontrollers using OBD protocol.

## Prerequisites

- Rust toolchain
- ESP-IDF
- USB cable for MCU programming

## Installation

### 1. Install Rust

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source $HOME/.cargo/env
```

### 2. Install ESP-IDF

```bash
git clone https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh
. ./export.sh
```

### 3. Install ESP Rust Toolchain

```bash
cargo install espup
espup install --targets esp32c3
```

### 4.Setup Project

```bash
git clone git@github.com:wroom-code-base/firmware.git
cd firmware
cargo add esp-idf-sys esp-idf-hal
```

## Building

```bash
cargo build --release
```

## Uploading to MCU

```bash
cargo espflash flash --release
```

For serial monitoring:
```bash
cargo espflash monitor
```

## Resources

- [ESP-IDF Documentation](https://docs.espressif.com/projects/esp-idf/)
- [esp-rs Community](https://github.com/esp-rs)
- [Rust Book](https://doc.rust-lang.org/book/)