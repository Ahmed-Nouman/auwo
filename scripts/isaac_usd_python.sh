#!/bin/bash
# Run a Python script with Isaac Sim's bundled USD (pxr) without starting Kit.
# ISAAC_SIM_PATH defaults to ~/isaac-sim-standalone-6.1.0.
ISAAC_SIM_PATH=${ISAAC_SIM_PATH:-$HOME/isaac-sim-standalone-6.1.0}
USD_LIBS=$(ls -d "$ISAAC_SIM_PATH"/extscache/omni.usd.libs-* 2>/dev/null | head -1)
if [ -z "$USD_LIBS" ]; then
  echo "omni.usd.libs not found under $ISAAC_SIM_PATH/extscache (set ISAAC_SIM_PATH)" >&2
  exit 1
fi
export PYTHONPATH="$USD_LIBS${PYTHONPATH:+:$PYTHONPATH}"
export LD_LIBRARY_PATH="$USD_LIBS/bin:$USD_LIBS/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec "$ISAAC_SIM_PATH/kit/python/bin/python3" "$@"
