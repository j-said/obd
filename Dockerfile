# syntax=docker/dockerfile:1
# ──────────────────────────────────────────────────────────────────────────────
# Stage 1 – Rust toolchain + ESP32C3 target
# ──────────────────────────────────────────────────────────────────────────────
FROM rust:1.86-slim-bookworm AS builder

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    git \
    pkg-config \
    libssl-dev \
    python3 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install the RISC-V bare-metal target
RUN rustup target add riscv32imc-unknown-none-elf \
    && rustup component add rust-src rustfmt clippy

# Install espflash for flashing
RUN cargo install espflash --locked

WORKDIR /firmware

# Copy dependency manifests first to cache layers
COPY Cargo.toml Cargo.lock rust-toolchain.toml ./

# Pre-fetch dependencies (dummy source so Cargo doesn't complain)
RUN mkdir -p src/bin && \
    echo '#![no_std]\n#![no_main]' > src/lib.rs && \
    echo '#![no_std]\n#![no_main]\nuse core::panic::PanicInfo;\n#[panic_handler]\nfn panic(_: &PanicInfo) -> ! { loop {} }' > src/bin/main.rs && \
    cargo fetch || true && \
    rm -rf src

# ──────────────────────────────────────────────────────────────────────────────
# Stage 2 – Development image (includes full source)
# ──────────────────────────────────────────────────────────────────────────────
FROM builder AS dev

COPY . .

# Build release binary
RUN cargo build --release

# Default command: open a shell for interactive development
CMD ["/bin/bash"]
