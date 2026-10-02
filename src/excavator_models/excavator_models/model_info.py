#!/usr/bin/env python3
"""Print the active excavator model (or a given one) and its files.

    ros2 run excavator_models model_info            # AUWO_EXCAVATOR_MODEL or v2
    ros2 run excavator_models model_info v1
"""
import sys

from excavator_models.registry import ENV_VAR, MODELS, default_model, describe


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    names = args or [default_model()]
    print(f'default model: {default_model()}  (set {ENV_VAR} to change; available: '
          f'{", ".join(sorted(MODELS))})\n')
    for n in names:
        print(describe(n))
        print()
    return 0


if __name__ == '__main__':
    sys.exit(main())
