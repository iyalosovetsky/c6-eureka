#!/usr/bin/env python3
"""Wrap an ESP32 app image (main.ino.bin) into a Zigbee OTA upgrade file.

Same layout as esp-zigbee-sdk tools/image_builder_tool: 56-byte OTA header,
then one "upgrade image" element (tag 0x0000).  Manufacturer and image type
default to the Arduino Zigbee library's addOTAClient() defaults.

Also writes c6-index.json next to the image: the zigbee2mqtt OTA index that
tools/z2m_ota.sh points z2m at (paths relative to the z2m data dir).

  python3 tools/make_ota.py build/main.ino.bin 0x01010400 out/c6-0x01010400.ota
"""
import hashlib
import json
import os
import struct
import sys

MANUFACTURER = 0x1001
IMAGE_TYPE = 0x1011


def main():
    src, version, dst = sys.argv[1], int(sys.argv[2], 0), sys.argv[3]
    data = open(src, "rb").read()
    element = struct.pack("<HI", 0x0000, len(data)) + data
    header_len = 56
    header = struct.pack(
        "<IHHHHHIH32sI",
        0x0BEEF11E,          # OTA upgrade file identifier
        0x0100,              # header version
        header_len,
        0x0000,              # field control: no optional fields
        MANUFACTURER,
        IMAGE_TYPE,
        version,
        0x0002,              # Zigbee stack version (Zigbee Pro)
        b"c6-eureka".ljust(32, b"\x00"),
        header_len + len(element),
    )
    image = header + element
    open(dst, "wb").write(image)
    print(f"{dst}: {len(image)} bytes, version 0x{version:08x}")

    name = os.path.basename(dst)
    index = [{
        "fileName": name,
        "url": f"ota/{name}",
        "fileVersion": version,
        "fileSize": len(image),
        "imageType": IMAGE_TYPE,
        "manufacturerCode": MANUFACTURER,
        "sha512": hashlib.sha512(image).hexdigest(),
        "otaHeaderString": "c6-eureka",
        "modelId": "ZBDimmable",
    }]
    index_path = os.path.join(os.path.dirname(dst), "c6-index.json")
    with open(index_path, "w") as f:
        json.dump(index, f, indent=1)
    print(f"{index_path}: points at ota/{name}")


if __name__ == "__main__":
    main()
