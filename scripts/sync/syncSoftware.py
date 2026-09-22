#!/usr/bin/env python3
"""Synchronize configured crawler software to one or more robot devices."""

import argparse
import logging
import os
from pathlib import Path
import shlex
import subprocess

import yaml


LOGGER = logging.getLogger("syncSoftware")
SYNC_FILE = "sync_config.yaml"
DEVICE_FILE = "device_list.yaml"
RSYNC_EXCLUDES = (
    "*.git/",
    "*.github/",
    ".vscode/",
    "coverage/",
    "html/",
    "*doc/",
    "*.png",
    "*.puml",
    "*.md",
    "*.dox"

)
REFERENCE_EXAMPLES = """
Examples:
    Sync one device using the default configuration:
        ./scripts/sync/syncSoftware.py -s remote -d ControlModule1

    Sync several devices:
        ./scripts/sync/syncSoftware.py -s remote -d ControlModule1,DevComputer2

    Use another configuration folder and show detailed logging:
        ./scripts/sync/syncSoftware.py -s remote -d ControlModule1 \\
            -c scenarios/dev -v

Configuration reference:
    The selected folder must contain sync_config.yaml and device_list.yaml.
    sync_config.yaml defines folders with folder_name, target, and
    supported_architectures, plus files with file_names, target, and optional
    hosts. File paths can use {host} or {device} to select a host-specific
    file automatically. A file entry with no host template or hosts list is
    synchronized to every selected host.
    device_list.yaml defines each device's type,
    architecture, and ros_workspace.
    Every sync ignores folders starting with templates and entries starting
    with test_.
"""


def read_yaml(path):
    """Read a YAML mapping and include the path in errors from the parser."""
    with path.open(encoding="utf-8") as config_file:
        data = yaml.safe_load(config_file)
    if not isinstance(data, dict):
        raise ValueError(f"Expected a YAML mapping in {path}")
    return data


def load_devices(config_dir):
    return read_yaml(config_dir / DEVICE_FILE).get("device_list", {})


def load_sync_folders(config_dir):
    folders = []
    for name, values in read_yaml(config_dir / SYNC_FILE).get("folders", {}).items():
        architectures = values.get("supported_architectures", values.get("supported_architecture", []))
        folders.append((name, values["folder_name"], values["target"], architectures))
    return folders


def load_sync_files(config_dir):
    files = []
    for name, values in read_yaml(config_dir / SYNC_FILE).get("files", {}).items():
        architectures = values.get("supported_architectures", values.get("supported_architecture", []))
        hosts = values.get("hosts", [])
        for file_name in values.get("file_names", []):
            files.append((name, file_name, values["target"], architectures, hosts))
    return files


def source_path(config_dir, folder_name):
    """Resolve paths relative to the repository containing the config folder."""
    configured_path = Path(folder_name).expanduser()
    if configured_path.is_absolute():
        return configured_path
    candidates = (
        config_dir.parent / configured_path,
        config_dir.parent.parent / configured_path,
        config_dir / configured_path,
    )
    return next((candidate for candidate in candidates if candidate.exists()), candidates[0])


def expand_host_value(value, device_name):
    """Expand host placeholders in configured source and target paths."""
    return value.replace("{host}", device_name).replace("{device}", device_name)


def remote_path(target):
    """Return a destination path and bootstrap path for a configured target."""
    if target.startswith("/"):
        return target, shlex.quote(target)
    if target.startswith("~/"):
        relative_target = target[2:]
        return target, f"$HOME/{shlex.quote(relative_target)}"
    return f"~/{target}", f"$HOME/{shlex.quote(target)}"


def rsync_folder(source, target, device_name):
    destination, bootstrap_target = remote_path(target)
    command = [
        "rsync",
        "-iartq",
        f"--rsync-path=mkdir -p {bootstrap_target} && rsync -iartq",
        f"{source}/",
    ]
    command.extend(f"--exclude={pattern}" for pattern in RSYNC_EXCLUDES)
    command.append(f"robot@{device_name}:{destination}/")
    LOGGER.debug("Synchronizing %s to %s", source, device_name)
    result = subprocess.run(command, check=False, text=True, capture_output=True)
    if result.stdout:
        LOGGER.info("%s", result.stdout.rstrip())
    if result.stderr:
        LOGGER.warning("%s", result.stderr.rstrip())
    return result.returncode == 0


def rsync_file(source, target, device_name):
    destination, bootstrap_target = remote_path(target)
    command = [
        "rsync",
        "-iartq",
        f"--rsync-path=mkdir -p {bootstrap_target} && rsync",
        str(source),
        f"robot@{device_name}:{destination}/",
    ]
    #LOGGER.debug("Synchronizing %s to %s", source, device_name)
    result = subprocess.run(command, check=False, text=True, capture_output=True)
    if result.stdout:
        LOGGER.debug("%s", result.stdout.rstrip())
    if result.stderr:
        LOGGER.warning("%s", result.stderr.rstrip())
    return result.returncode == 0


def sync_remote(config_dir, device_name):
    LOGGER.info("Sync started to %s", device_name)
    devices = load_devices(config_dir)
    if device_name not in devices:
        LOGGER.error("Device '%s' is not in %s", device_name, config_dir / DEVICE_FILE)
        return False

    device = devices[device_name]
    architecture = device["architecture"]
    success = True
    for name, folder_name, target, architectures in load_sync_folders(config_dir):
        if architecture not in architectures:
            LOGGER.info("Skipping %s for architecture %s", name, architecture)
            continue
        source = source_path(config_dir, folder_name)
        if not source.is_dir():
            LOGGER.error("Configured source folder does not exist: %s", source)
            success = False
            continue
        success = rsync_folder(source, target, device_name) and success
    for name, file_name, target, architectures, hosts in load_sync_files(config_dir):
        if hosts and device_name not in hosts:
            LOGGER.info("Skipping %s for device %s", name, device_name)
            continue
        if architecture not in architectures:
            LOGGER.info("Skipping %s for architecture %s", name, architecture)
            continue
        source = source_path(config_dir, expand_host_value(file_name, device_name))
        expanded_target = expand_host_value(target, device_name)
        if not source.is_file():
            LOGGER.error("Configured source file does not exist: %s", source)
            success = False
            continue
        success = rsync_file(source, expanded_target, device_name) and success
    LOGGER.info("Sync completed to %s", device_name)
    return success


def parse_args():
    parser = argparse.ArgumentParser(
        description=__doc__,
        epilog=REFERENCE_EXAMPLES,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("-s", "--syncmode", default="all", help="Sync mode (remote is supported)")
    parser.add_argument("-d", "--devices", default="", help="DeviceName1,DeviceName2,...")
    parser.add_argument(
        "-c",
        "--config_dir",
        default=os.path.expanduser("~/ros2_ws/src/crawler_app/robot_config"),
        help="Folder containing sync_config.yaml and device_list.yaml",
    )
    parser.add_argument(
        "--quiet",
        dest="quiet",
        action="store_true",
        default=False,
        help="Show only errors and warnings",
    )
    parser.add_argument(
        "--no-quiet",
        dest="quiet",
        action="store_false",
        help="Show sync progress and informational messages (default)",
    )
    parser.add_argument("-v", "--verbose", action="store_true", help="Enable detailed logging")
    return parser.parse_args()


def main():
    options = parse_args()
    if options.verbose:
        log_level = logging.DEBUG
    elif options.quiet:
        log_level = logging.ERROR
    else:
        log_level = logging.INFO
    logging.basicConfig(level=log_level, format="%(levelname)s: %(message)s")
    if options.syncmode != "remote":
        LOGGER.error("Sync mode '%s' is not supported", options.syncmode)
        return 1

    config_dir = Path(options.config_dir).expanduser().resolve()
    devices = [device.strip() for device in options.devices.split(",") if device.strip()]
    if not devices:
        LOGGER.error("At least one device is required with --devices")
        return 1
    success = True
    for device in devices:
        success = sync_remote(config_dir, device) and success
    return 0 if success else 1


if __name__ == "__main__":
    raise SystemExit(main())
