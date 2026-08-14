#!/usr/bin/python3
"""Convert an HTML report to DOCX after embedding every linked image."""

from __future__ import annotations

import argparse
import os
import socket
import subprocess
import tempfile
import time
import zipfile
from pathlib import Path
from urllib.parse import unquote, urlparse
from xml.etree import ElementTree

import uno
from com.sun.star.beans import PropertyValue


def property_value(name: str, value: object) -> PropertyValue:
    prop = PropertyValue()
    prop.Name = name
    prop.Value = value
    return prop


def free_tcp_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def connect_to_office(port: int):
    local_context = uno.getComponentContext()
    resolver = local_context.ServiceManager.createInstanceWithContext(
        "com.sun.star.bridge.UnoUrlResolver", local_context
    )
    address = f"uno:socket,host=127.0.0.1,port={port};urp;StarOffice.ComponentContext"
    deadline = time.monotonic() + 15.0
    while True:
        try:
            return resolver.resolve(address)
        except Exception:
            if time.monotonic() >= deadline:
                raise
            time.sleep(0.2)


def embed_external_docx_images(docx_path: Path) -> int:
    relationship_path = "word/_rels/document.xml.rels"
    document_path = "word/document.xml"
    relationship_namespace = "http://schemas.openxmlformats.org/package/2006/relationships"
    image_relationship = (
        "http://schemas.openxmlformats.org/officeDocument/2006/relationships/image"
    )

    with zipfile.ZipFile(docx_path, "r") as source_zip:
        relationship_xml = source_zip.read(relationship_path)
        document_xml = source_zip.read(document_path)
        relationship_root = ElementTree.fromstring(relationship_xml)
        images: list[tuple[str, bytes]] = []
        for relationship in relationship_root.findall(
            f"{{{relationship_namespace}}}Relationship"
        ):
            if relationship.get("Type") != image_relationship:
                continue
            if relationship.get("TargetMode") != "External":
                continue
            source_url = relationship.get("Target", "")
            parsed = urlparse(source_url)
            if parsed.scheme != "file":
                raise RuntimeError(f"Unsupported external image URL: {source_url}")
            image_path = Path(unquote(parsed.path))
            if not image_path.is_file():
                raise FileNotFoundError(image_path)
            media_name = f"word/media/report_image_{len(images) + 1}{image_path.suffix.lower()}"
            images.append((media_name, image_path.read_bytes()))
            relationship_id = relationship.get("Id")
            if not relationship_id:
                raise RuntimeError(f"Image relationship has no Id: {source_url}")
            link_attribute = f'r:link="{relationship_id}"'.encode()
            embed_attribute = f'r:embed="{relationship_id}"'.encode()
            if link_attribute not in document_xml:
                raise RuntimeError(
                    f"Document has no linked drawing for relationship {relationship_id}"
                )
            document_xml = document_xml.replace(link_attribute, embed_attribute)
            old_target = f'Target="{source_url}" TargetMode="External"'.encode()
            new_target = f'Target="{media_name.removeprefix("word/")}"'.encode()
            if old_target not in relationship_xml:
                raise RuntimeError(
                    f"Could not rewrite external relationship {relationship_id}"
                )
            relationship_xml = relationship_xml.replace(old_target, new_target, 1)

        if not images:
            return 0

        with tempfile.NamedTemporaryFile(
            prefix=f".{docx_path.name}.", suffix=".tmp", dir=docx_path.parent, delete=False
        ) as temporary_file:
            temporary_path = Path(temporary_file.name)

        try:
            with zipfile.ZipFile(temporary_path, "w") as output_zip:
                for info in source_zip.infolist():
                    payload = source_zip.read(info.filename)
                    if info.filename == relationship_path:
                        payload = relationship_xml
                    elif info.filename == document_path:
                        payload = document_xml
                    output_zip.writestr(info, payload)
                for media_name, payload in images:
                    output_zip.writestr(
                        media_name, payload, compress_type=zipfile.ZIP_DEFLATED
                    )
            os.replace(temporary_path, docx_path)
        finally:
            if temporary_path.exists():
                temporary_path.unlink()
    return len(images)


def convert_with_embedded_images(source: Path, output: Path) -> int:
    port = free_tcp_port()
    profile = Path(tempfile.mkdtemp(prefix="lo-report-profile-"))
    command = [
        "libreoffice",
        f"-env:UserInstallation={uno.systemPathToFileUrl(str(profile))}",
        "--headless",
        "--invisible",
        "--nologo",
        "--nodefault",
        "--nofirststartwizard",
        "--norestore",
        f"--accept=socket,host=127.0.0.1,port={port};urp;StarOffice.ComponentContext",
    ]
    office = subprocess.Popen(command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    document = None
    try:
        context = connect_to_office(port)
        service_manager = context.ServiceManager
        desktop = service_manager.createInstanceWithContext(
            "com.sun.star.frame.Desktop", context
        )
        document = desktop.loadComponentFromURL(
            uno.systemPathToFileUrl(str(source.resolve())),
            "_blank",
            0,
            (property_value("Hidden", True),),
        )
        if document is None:
            raise RuntimeError(f"LibreOffice could not load {source}")

        graphics = document.getGraphicObjects()
        embedded = 0
        for name in graphics.getElementNames():
            graphic_object = graphics.getByName(name)
            graphic = graphic_object.Graphic
            if graphic is None:
                raise RuntimeError(f"Could not load linked image object {name}")
            graphic_object.Graphic = graphic
            embedded += 1

        output.parent.mkdir(parents=True, exist_ok=True)
        document.storeAsURL(
            uno.systemPathToFileUrl(str(output.resolve())),
            (
                property_value("FilterName", "Office Open XML Text"),
                property_value("Overwrite", True),
            ),
        )
        return embedded
    finally:
        if document is not None:
            document.close(True)
        office.terminate()
        try:
            office.wait(timeout=5)
        except subprocess.TimeoutExpired:
            office.kill()
            office.wait(timeout=5)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    graphics_seen = convert_with_embedded_images(args.source, args.output)
    embedded = embed_external_docx_images(args.output)
    print(f"DOCX={args.output.resolve()}")
    print(f"graphics_seen={graphics_seen}")
    print(f"embedded_images={embedded}")


if __name__ == "__main__":
    main()
