#!/usr/bin/env bash
# SPDX-License-Identifier: CC0-1.0
set -euo pipefail

build_dir=$1
ctest --test-dir "$build_dir" --timeout 120 --no-tests=error --output-junit test-results.xml

# Developer machines may skip unavailable kernel/policy features, but the CI
# runners must exercise every configured test, including namespace isolation.
python3 - "$build_dir/test-results.xml" <<'PY'
import sys
import xml.etree.ElementTree as ET

skipped = [test.attrib["name"] for test in ET.parse(sys.argv[1]).iter("testcase")
           if test.find("skipped") is not None]
if skipped:
    sys.exit("CI requires these skipped tests to run: " + ", ".join(skipped))
PY
