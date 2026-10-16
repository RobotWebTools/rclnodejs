# Copyright (c) 2026 RobotWebTools Contributors. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

import json


MAX_SAFE_INTEGER = (1 << 53) - 1


def _preserve_large_integers(value):
    if isinstance(value, int) and not -MAX_SAFE_INTEGER <= value <= MAX_SAFE_INTEGER:
        return str(value)
    if isinstance(value, dict):
        return {key: _preserve_large_integers(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_preserve_large_integers(item) for item in value]
    return value


def dumps_for_javascript(value):
    """Encode unsafe integers as decimal strings before JavaScript can round them."""
    return json.dumps(_preserve_large_integers(value))
