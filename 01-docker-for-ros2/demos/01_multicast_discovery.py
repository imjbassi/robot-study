"""A toy version of DDS discovery (SPDP) to show why ROS 2 needs --network host.

Real ROS 2 nodes announce themselves by sending UDP multicast packets to
239.255.0.1:7400 (for ROS_DOMAIN_ID=0). Anyone listening on that group learns
the node exists. This script does the same thing with plain sockets.

    python3 01_multicast_discovery.py listen
    python3 01_multicast_discovery.py announce --name my_node

Run `listen` on the host and `announce` in a container: with --network host the
announcements arrive; on the default bridge network they never do.
"""

import argparse
import json
import os
import socket
import struct
import time

GROUP = "239.255.0.1"  # DDS SPDP multicast group
BASE_PORT = 7400  # SPDP port = 7400 + 250 * domain_id


def discovery_port(domain_id):
    return BASE_PORT + 250 * domain_id


def listen(domain_id):
    port = discovery_port(domain_id)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", port))
    # Join the multicast group on all interfaces.
    mreq = struct.pack("4s4s", socket.inet_aton(GROUP), socket.inet_aton("0.0.0.0"))
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

    print(f"Listening for participants on {GROUP}:{port} (domain {domain_id}) ...")
    seen = set()
    while True:
        data, (addr, _) = sock.recvfrom(1024)
        try:
            msg = json.loads(data)
        except ValueError:
            continue  # probably a real DDS packet, not ours
        key = (msg.get("name"), addr)
        if key not in seen:
            seen.add(key)
            print(f"  discovered '{msg['name']}' from {addr} (host={msg['host']})")


def announce(domain_id, name, count, interval):
    port = discovery_port(domain_id)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    # TTL 1 = stay on the local network segment, like DDS does by default.
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 1)

    msg = json.dumps({"name": name, "host": socket.gethostname(), "pid": os.getpid()})
    print(f"Announcing '{name}' to {GROUP}:{port} every {interval}s ...")
    for i in range(count):
        sock.sendto(msg.encode(), (GROUP, port))
        print(f"  sent announcement {i + 1}/{count}")
        time.sleep(interval)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("mode", choices=["listen", "announce"])
    parser.add_argument("--domain", type=int, default=int(os.environ.get("ROS_DOMAIN_ID", 0)))
    parser.add_argument("--name", default=socket.gethostname())
    parser.add_argument("--count", type=int, default=10)
    parser.add_argument("--interval", type=float, default=1.0)
    args = parser.parse_args()

    if args.mode == "listen":
        listen(args.domain)
    else:
        announce(args.domain, args.name, args.count, args.interval)


if __name__ == "__main__":
    main()
