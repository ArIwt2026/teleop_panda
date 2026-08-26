# Network Interfaces & IP Configuration Guide

Simple reference guide for Panda Robot & PLC Ethernet interfaces, IP addresses, subnets, and setup commands.

---

## 1. Network Interfaces Overview

| Interface | Device / Adapter | Configured IP / Subnet | Purpose / Connection |
| :--- | :--- | :--- | :--- |
| **`enp4s0`** | Built-in Ethernet | `172.16.0.1 / 16` (`255.255.0.0`) | **Panda Robot Direct Connection** |
| **`enx00e04c685959`** | USB Ethernet Adapter | `192.168.0.222 / 24` (`255.255.255.0`) | **PLC Connection** |

---

## 2. Franka Panda Ethernet Configuration (`enp4s0`)

- **Interface Name**: `enp4s0`
- **Host System IP**: `172.16.0.1`
- **Subnet Mask**: `255.255.0.0` (CIDR: `/16`)
- **Panda Control Box IP**: `172.16.0.2`

### Command to configure `enp4s0` manually (if needed):
```bash
sudo ip addr add 172.16.0.1/16 dev enp4s0
sudo ip link set enp4s0 up
```

### Ping test to verify connection to Panda Robot:
```bash
ping -c 3 172.16.0.2
```

---

## 3. PLC Ethernet Adapter (`enx00e04c685959`)

- **Interface Name**: `enx00e04c685959`
- **Host System IP**: `192.168.0.222`
- **Subnet Mask**: `255.255.255.0` (CIDR: `/24`)

### Command to configure manually (if needed):
```bash
sudo ip addr add 192.168.0.222/24 dev enx00e04c685959
sudo ip link set enx00e04c685959 up
```

---

## 4. Network Setup via GUI (NetworkManager)

If setting up static IPs via Ubuntu Settings:

1. **Panda Connection (`enp4s0`)**:
   - **IPv4 Method**: Manual
   - **Address**: `172.16.0.1`
   - **Netmask**: `255.255.0.0` (or `/16`)
   - **Gateway**: (leave empty)

2. **PLC Connection (`enx00e04c685959`)**:
   - **IPv4 Method**: Manual
   - **Address**: `192.168.0.222`
   - **Netmask**: `255.255.255.0` (or `/24`)
   - **Gateway**: (leave empty)

---

## 5. Docker Network Access
When running Docker containers for ROS 2 communicating with the Panda robot (`172.16.0.2`) or PLC (`192.168.0.222`), run the container with host networking:

```bash
docker run --rm -it --gpus all \
  --net=host \
  -e ROS_DOMAIN_ID=42 \
  <your-docker-image>
```
