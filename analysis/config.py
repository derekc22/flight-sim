import json
from pathlib import Path


def load_json(path, required_fields):
    with Path(path).open(encoding="utf-8") as file:
        data = json.load(file)

    for field_path in required_fields:
        value = data
        for field in field_path.split("."):
            if not isinstance(value, dict) or field not in value:
                raise ValueError(f"Error: key '{field_path}' is not present in JSON")
            value = value[field]

    return data
