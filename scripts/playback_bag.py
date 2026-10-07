#!/usr/bin/env python3

import argparse
import os
import subprocess
import sys

from ament_index_python.packages import get_package_share_directory
import yaml


def load_excluded_topics(scenario_name):
    if os.path.sep in scenario_name or scenario_name in {'.', '..'}:
        raise ValueError(f"Invalid playback scenario name: '{scenario_name}'")

    deployment_map_path = os.path.join(
        get_package_share_directory('crawler_app'),
        'config',
        'playback_scenarios',
        scenario_name,
        'deployment_map.yaml',
    )
    if not os.path.isfile(deployment_map_path):
        raise ValueError(f"Playback deployment map does not exist: {deployment_map_path}")

    with open(deployment_map_path, 'r', encoding='utf-8') as config_file:
        deployment_map = yaml.safe_load(config_file) or {}

    excluded_topics = deployment_map.get('exclude_topics', [])
    if not isinstance(excluded_topics, list) or any(
            not isinstance(topic, str) or not topic for topic in excluded_topics):
        raise ValueError("'exclude_topics' must be a list of non-empty topic names")

    return excluded_topics


def main():
    parser = argparse.ArgumentParser(
        description='Play a ROS 2 bag using the selected playback scenario')
    parser.add_argument('scenario', help='Name under config/playback_scenarios')
    parser.add_argument('bag_path', help='Path to the bag directory')
    args, bag_arguments = parser.parse_known_args()

    try:
        excluded_topics = load_excluded_topics(args.scenario)
    except (OSError, ValueError, yaml.YAMLError) as error:
        parser.error(str(error))

    command = ['ros2', 'bag', 'play', args.bag_path, '--clock']
    if excluded_topics:
        command.extend(['--exclude-topics', *excluded_topics])
    command.extend(bag_arguments)

    print(f"Playback scenario: {args.scenario}")
    print(f"Excluded bag topics: {excluded_topics or '(none)'}")
    try:
        return subprocess.call(command)
    except FileNotFoundError:
        print("Unable to run 'ros2'; source the ROS 2 workspace first", file=sys.stderr)
        return 127


if __name__ == '__main__':
    sys.exit(main())