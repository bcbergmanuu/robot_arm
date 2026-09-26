"""Command line entry point: `uv run robotarm <command>`."""
import argparse
import sys

from robotarm import bus
from robotarm.analysis import steptest
from robotarm.master import gamepad, teleop
from robotarm.sim import server as sim_server
from robotarm.sim import tune
from robotarm.tools import gen_config


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="robotarm", description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    gen_config.register(subparsers)
    steptest.register(subparsers)
    tune.register(subparsers)
    sim_server.register(subparsers)
    bus.register(subparsers)
    gamepad.register(subparsers)
    teleop.register(subparsers)
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
