import toml
from datetime import datetime

with open("pyproject.toml", "r") as f:
    config = toml.load(f)

now = datetime.now().strftime("%Y.%m.%d.%H%M")
config["tool"]["poetry"]["version"] = now

with open("pyproject.toml", "w") as f:
    toml.dump(config, f)

print(f"Version updated to {now}")
