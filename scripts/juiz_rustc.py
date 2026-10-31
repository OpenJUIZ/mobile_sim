"""PyPI の juiz-cli の wheel から、その juiz をビルドした rustc の版を読んで出す

Rust のプラグインは juiz と同じ rustc でビルドしないと読み込めない(juiz docs/spec/app-package.md §12.1)。
配布用のプラグインは pip の juiz-cli で動かすので、その wheel の中のビルド情報
(juiz_sdk=…;rustc=rustc 1.99.1 (…))から rustc の版を読む。

使い方: python3 scripts/juiz_rustc.py 0.0.29   # => 1.99.1
"""
import io
import json
import re
import sys
import urllib.request
import zipfile


def main():
    if len(sys.argv) != 2:
        sys.exit("usage: juiz_rustc.py <juiz version>")
    version = sys.argv[1]
    with urllib.request.urlopen(f"https://pypi.org/pypi/juiz-cli/{version}/json", timeout=30) as r:
        urls = [
            u["url"]
            for u in json.load(r)["urls"]
            if "manylinux" in u["filename"] and "x86_64" in u["filename"]
        ]
    if not urls:
        sys.exit(f"no manylinux x86_64 wheel of juiz-cli {version}")
    with urllib.request.urlopen(urls[0], timeout=120) as r:
        wheel = zipfile.ZipFile(io.BytesIO(r.read()))
    pattern = re.compile(rb"juiz_sdk=([0-9A-Za-z.+-]+);rustc=rustc ([0-9]+\.[0-9]+\.[0-9]+)")
    for name in wheel.namelist():
        if name.endswith(".so"):
            m = pattern.search(wheel.read(name))
            if m:
                sdk = m.group(1).decode()
                if sdk != version:
                    sys.exit(f"{name} was built with juiz_sdk {sdk}, not {version}")
                print(m.group(2).decode())
                return
    sys.exit(f"no build info in the wheel of juiz-cli {version}")


if __name__ == "__main__":
    main()
