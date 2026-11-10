#!/usr/bin/env python3
"""
Shared Ignition Gazebo entity inspection and removal helpers.
Queries active models (via direct `/world/<world>/scene/info` first, falling
back to `ign model --list`) before invoking `/world/<world>/remove`, so existing
test objects are reliably removed without triggering Ignition `not found` errors.
"""

import re
import subprocess
import threading

DEFAULT_TEST_OBJECTS = (
    'presented_object',
    'red_object',
    'green_object',
    'blue_object',
    'yellow_object',
    'red_box',
    'green_box',
    'blue_box',
    'yellow_box',
)

GUI_PRIMITIVE_PREFIXES = ('box', 'cylinder', 'sphere', 'unit_box', 'unit_cylinder')

PERMANENT_MODELS = frozenset({
    'ground_plane',
    'home_docking_pad',
    'red_pillar',
    'green_pillar',
    'white_pillar',
    'colour_detection_robot',
})

_REMOVE_LOCK = threading.Lock()


def _query_scene_info(world_name: str, timeout: float = 5.0):
    """Query top-level model names in a known Gazebo world via scene/info RPC."""
    try:
        res = subprocess.run(
            [
                'ign', 'service',
                '-s', f'/world/{world_name}/scene/info',
                '--reqtype', 'ignition.msgs.Empty',
                '--reptype', 'ignition.msgs.Scene',
                '--timeout', '2000',
                '--req', '',
            ],
            capture_output=True,
            text=True,
            timeout=timeout,
        )
    except (subprocess.SubprocessError, OSError) as exc:
        return False, set(), f'scene/info exception: {exc}'

    if res.returncode != 0 or 'model {' not in res.stdout:
        return False, set(), f'scene/info rc={res.returncode}'

    models = set(re.findall(r'^model \{\r?\n  name: "([^"]+)"', res.stdout, flags=re.MULTILINE))
    return True, models, ''


def get_active_world_and_models(preferred_world: str = '', timeout: float = 5.0):
    """Return (query_ok, world_name, existing_model_names, diagnostic_msg)."""
    diag = ''
    if preferred_world:
        ok, models, diag = _query_scene_info(preferred_world, timeout=timeout)
        if ok:
            return True, preferred_world, models, ''

    try:
        res = subprocess.run(
            ['ign', 'model', '--list'],
            capture_output=True,
            text=True,
            timeout=timeout,
        )
    except (subprocess.SubprocessError, OSError) as exc:
        return False, preferred_world or None, set(), f'{diag}; ign model --list failed: {exc}'.strip('; ')

    world_name = None
    world_match = re.search(r'Requesting state for world \[([^\]]+)\]', res.stdout)
    if world_match:
        world_name = world_match.group(1)

    models = set()
    in_models_section = False
    for line in res.stdout.splitlines():
        if 'Available models:' in line:
            in_models_section = True
            continue
        if in_models_section:
            stripped = line.strip()
            if stripped.startswith('- '):
                models.add(stripped[2:].strip())

    if world_name or in_models_section:
        return True, world_name or preferred_world or None, models, ''

    return False, preferred_world or None, set(), f'{diag}; ign model --list rc={res.returncode}'.strip('; ')


def _match_target_models(existing_models, candidates, include_gui_primitives: bool):
    """Select removable model names matching candidates or GUI-suffixed names."""
    prefixes = list(candidates)
    if include_gui_primitives:
        prefixes.extend(GUI_PRIMITIVE_PREFIXES)

    matched = []
    for model in sorted(existing_models):
        if model in PERMANENT_MODELS:
            continue
        if model in candidates or any(model == p or model.startswith(f'{p}_') for p in prefixes):
            matched.append(model)
    return matched


def _invoke_remove(world: str, model_name: str, timeout: float = 3.0) -> bool:
    """Invoke Gazebo /world/<world>/remove for one model."""
    try:
        res = subprocess.run(
            [
                'ign', 'service',
                '-s', f'/world/{world}/remove',
                '--reqtype', 'ignition.msgs.Entity',
                '--reptype', 'ignition.msgs.Boolean',
                '--timeout', '1500',
                '--req', f'name: "{model_name}", type: MODEL',
            ],
            capture_output=True,
            text=True,
            timeout=timeout,
        )
        return 'data: true' in res.stdout
    except (subprocess.SubprocessError, OSError):
        return False


def remove_existing_models(
    target_names=None,
    preferred_world: str = '',
    fallback_remove: bool = False,
):
    """Remove matching models currently present in Gazebo.

    Returns (removed_pairs, warning_msg) where removed_pairs is [(model, world), ...].
    If fallback_remove=True and Gazebo model querying fails completely, attempts a
    direct single remove of 'presented_object' on preferred_world so cleanup still runs.
    """
    include_gui = target_names is None
    candidates = tuple(target_names) if target_names is not None else DEFAULT_TEST_OBJECTS
    removed = []

    with _REMOVE_LOCK:
        query_ok, active_world, existing_models, diag = get_active_world_and_models(
            preferred_world=preferred_world
        )
        world = active_world or preferred_world

        if not query_ok:
            if fallback_remove and world:
                if _invoke_remove(world, 'presented_object'):
                    removed.append(('presented_object', world))
                    return removed, f'Model query failed ({diag}), but fallback remove succeeded.'
            return removed, diag

        if not world:
            return removed, 'Could not determine active Gazebo world.'

        for obj in _match_target_models(existing_models, candidates, include_gui):
            if _invoke_remove(world, obj):
                removed.append((obj, world))

    return removed, ''
