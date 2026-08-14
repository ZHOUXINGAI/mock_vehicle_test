#!/usr/bin/env python3
"""Download one PX4 ULog through MAVROS LOG_REQUEST_DATA services."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import rclpy
from mavros_msgs.msg import LogData
from mavros_msgs.srv import LogRequestData, LogRequestEnd
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log-id", type=int, required=True)
    parser.add_argument("--size", type=int, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--namespace", default="/mavros")
    parser.add_argument("--timeout-sec", type=float, default=180.0)
    return parser.parse_args()


class Downloader(Node):
    def __init__(self, namespace: str, log_id: int, size: int) -> None:
        super().__init__("px4_ulog_read_only_downloader")
        self.log_id = log_id
        self.size = size
        self.buffer = bytearray(size)
        self.received = bytearray(size)
        self.received_count = 0
        self.last_progress = time.monotonic()
        self.next_report = 0.1
        self.create_subscription(
            LogData,
            f"{namespace}/log_transfer/raw/log_data",
            self._data_callback,
            qos_profile_sensor_data,
        )
        self.request_client = self.create_client(
            LogRequestData, f"{namespace}/log_transfer/raw/log_request_data"
        )
        self.end_client = self.create_client(
            LogRequestEnd, f"{namespace}/log_transfer/raw/log_request_end"
        )

    def _data_callback(self, message: LogData) -> None:
        if int(message.id) != self.log_id or not message.data:
            return
        offset = int(message.offset)
        block = bytes(message.data)
        if offset < 0 or offset >= self.size:
            return
        block = block[: self.size - offset]
        for index, value in enumerate(block, start=offset):
            self.buffer[index] = value
            if not self.received[index]:
                self.received[index] = 1
                self.received_count += 1
        self.last_progress = time.monotonic()
        ratio = self.received_count / self.size
        if ratio >= self.next_report or self.received_count == self.size:
            print(
                f"received={self.received_count}/{self.size} ({ratio:.1%})",
                flush=True,
            )
            self.next_report += 0.1

    def request(self, offset: int, count: int) -> None:
        if not self.request_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("log_request_data service unavailable")
        request = LogRequestData.Request()
        request.id = self.log_id
        request.offset = offset
        request.count = count
        future = self.request_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done() or future.result() is None or not future.result().success:
            raise RuntimeError(
                f"log request rejected: id={self.log_id} offset={offset} count={count}"
            )
        print(f"requested offset={offset} count={count}", flush=True)

    def first_missing_span(self) -> tuple[int, int] | None:
        try:
            start = self.received.index(0)
        except ValueError:
            return None
        end = start
        while end < self.size and not self.received[end]:
            end += 1
        return start, end - start

    def end_transfer(self) -> None:
        if not self.end_client.wait_for_service(timeout_sec=5.0):
            print("WARNING: log_request_end service unavailable", flush=True)
            return
        future = self.end_client.call_async(LogRequestEnd.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        response = future.result() if future.done() else None
        print(
            f"log_request_end success={bool(response and response.success)}",
            flush=True,
        )


def main() -> int:
    args = parse_args()
    if not 0 <= args.log_id <= 65535:
        raise SystemExit("--log-id must be within [0, 65535]")
    if args.size <= 0:
        raise SystemExit("--size must be positive")
    if not 10.0 <= args.timeout_sec <= 600.0:
        raise SystemExit("--timeout-sec must be within [10, 600]")

    namespace = args.namespace.rstrip("/")
    rclpy.init()
    node = Downloader(namespace, args.log_id, args.size)
    deadline = time.monotonic() + args.timeout_sec
    retries = 0
    try:
        node.request(0, args.size)
        while rclpy.ok() and node.received_count < args.size:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if now >= deadline:
                break
            if now - node.last_progress >= 5.0:
                missing = node.first_missing_span()
                if missing is None:
                    break
                retries += 1
                if retries > 20:
                    break
                offset, count = missing
                node.request(offset, count)
                node.last_progress = time.monotonic()

        if node.received_count != args.size:
            missing = node.first_missing_span()
            raise RuntimeError(
                f"incomplete log: received={node.received_count}/{args.size} "
                f"first_missing={missing} retries={retries}"
            )
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_bytes(node.buffer)
        print(
            f"OK output={args.output} bytes={args.size} retries={retries}",
            flush=True,
        )
        return 0
    finally:
        node.end_transfer()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
