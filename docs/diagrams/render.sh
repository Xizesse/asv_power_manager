#!/usr/bin/env bash
# Render every docs/diagrams/*.dot to docs/img/<name>.svg and .png (needs graphviz: apt install graphviz)
set -euo pipefail
cd "$(dirname "$0")"
for f in *.dot; do
  n="${f%.dot}"
  dot -Tsvg "$f" -o "../img/$n.svg"
  dot -Tpng -Gdpi=150 "$f" -o "../img/$n.png"
  echo "rendered $n"
done
