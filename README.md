# rplidar zenoh driver

## Install protoc on ubuntu

```bash
sudo apt update && sudo apt install autoconf automake libtool curl make g++ unzip -y
```

1. Download from [github releases](https://github.com/protocolbuffers/protobuf/releases)
2. extract `tar -xvf`
3. `./configure`
4. `make -j16`
5. `make -j16 check`
6. `sudo make install`
7. `sudo ldconfig`
