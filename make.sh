python update_version.py
poetry build -f wheel
poetry config repositories.sdpi http://18.218.85.44:8080/
poetry publish -r sdpi