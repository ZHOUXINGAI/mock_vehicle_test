#!/usr/bin/env python3
"""Download one PX4 file through the read-only MAVROS FTP services."""

from __future__ import annotations

import argparse
from pathlib import Path

import rclpy
from mavros_msgs.srv import FileClose, FileOpen, FileRead
from rclpy.node import Node


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("remote_path")
    parser.add_argument("output_path", type=Path)
    parser.add_argument("--namespace", default="/mavros")
    parser.add_argument("--chunk-size", type=int, default=200)
    return parser.parse_args()


def call(node: Node, client, request, label: str):
    if not client.wait_for_service(timeout_sec=10.0):
        raise RuntimeError(f"service unavailable: {label}")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=15.0)
    if not future.done() or future.result() is None:
        raise RuntimeError(f"service timeout: {label}")
    return future.result()


def main() -> int:
    args = parse_args()
    if not 1 <= args.chunk_size <= 200:
        raise SystemExit("--chunk-size must be within [1, 200]")

    namespace = args.namespace.rstrip("/")
    rclpy.init()
    node = Node("px4_file_read_only_downloader")
    opened = False
    try:
        open_client = node.create_client(FileOpen, f"{namespace}/ftp/open")
        read_client = node.create_client(FileRead, f"{namespace}/ftp/read")
        close_client = node.create_client(FileClose, f"{namespace}/ftp/close")

        open_request = FileOpen.Request()
        open_request.file_path = args.remote_path
        open_request.mode = FileOpen.Request.MODE_READ
        open_response = call(node, open_client, open_request, "ftp/open")
        if not open_response.success:
            raise RuntimeError(f"ftp/open failed errno={open_response.r_errno}")
        opened = True
        expected_size = int(open_response.size)

        args.output_path.parent.mkdir(parents=True, exist_ok=True)
        offset = 0
        with args.output_path.open("wb") as output:
            while offset < expected_size:
                request = FileRead.Request()
                request.file_path = args.remote_path
                request.offset = offset
                request.size = min(args.chunk_size, expected_size - offset)
                response = call(node, read_client, request, "ftp/read")
                if not response.success:
                    raise RuntimeError(
                        f"ftp/read failed offset={offset} errno={response.r_errno}"
                    )
                block = bytes(response.data)
                if not block:
                    raise RuntimeError(f"unexpected EOF at offset={offset}")
                output.write(block)
                offset += len(block)
                if offset == expected_size or offset % 20000 < len(block):
                    print(f"downloaded={offset}/{expected_size}", flush=True)

        if offset != expected_size:
            raise RuntimeError(f"size mismatch: got={offset} expected={expected_size}")
        print(f"OK output={args.output_path} bytes={offset}", flush=True)
        return 0
    finally:
        if opened:
            close_request = FileClose.Request()
            close_request.file_path = args.remote_path
            try:
                close_response = call(node, close_client, close_request, "ftp/close")
                print(
                    f"ftp_close success={close_response.success} "
                    f"errno={close_response.r_errno}",
                    flush=True,
                )
            except Exception as exc:  # Close is best effort after a read failure.
                print(f"WARNING: ftp/close failed: {exc}", flush=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
