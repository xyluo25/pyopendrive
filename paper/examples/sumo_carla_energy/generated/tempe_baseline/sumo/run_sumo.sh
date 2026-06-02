#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
sumo -c scenario.sumocfg
