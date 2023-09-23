#!/bin/bash
find src/* | xargs clang-format -i
find tests/* | xargs clang-format -i
find example/* | xargs clang-format -i
python3 -m isort .
python3 -m black --required-version 22.6.0 .
python3 -m flake8 .
