FROM balenalib/raspberrypi4-64-debian AS chef

WORKDIR /app

# Install dependancies
RUN apt-get update && apt-get install -y lld clang autoconf libtool pkg-config build-essential unzip wget

# install protoc
RUN wget https://github.com/protocolbuffers/protobuf/releases/download/v23.4/protoc-23.4-linux-aarch_64.zip
RUN unzip protoc-23.4-linux-aarch_64.zip
RUN cp -r /app/include/* /usr/local/include/.
RUN cp /app/bin/protoc /usr/local/bin/.

# Install rust
RUN curl https://sh.rustup.rs -sSf | bash -s -- -y
ENV PATH="/root/.cargo/bin:${PATH}"

RUN cargo install cargo-chef

# rust layer caching
FROM chef AS planner
COPY . .
RUN cargo chef prepare --recipe-path recipe.json

# rebuild dependencies if changed
FROM chef AS builder
# install deb because it doesn't chage often
RUN cargo install cargo-deb

# dependancies rebuild
COPY --from=planner /app/recipe.json recipe.json
RUN cargo chef cook --release --recipe-path recipe.json

# Now copy code
COPY . .

# Build
RUN cargo build --all --bins --release
RUN cargo deb --no-build

# Copy to exporter
FROM scratch AS export
COPY --from=builder /app/target/release/driver /
COPY --from=builder /app/target/release/foxglove_server /
COPY --from=builder /app/target/release/mcap_logger /
COPY --from=builder /app/target/debian/rplidar-zenoh-driver*.deb /
COPY --from=builder /app/target/debian/rplidar-zenoh-driver*.deb /rplidar-zenoh-driver.deb
