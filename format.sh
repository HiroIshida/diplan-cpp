#!/bin/bash
find src/* | xargs clang-format -i
find tests/* | xargs clang-format -i
find example/* | xargs clang-format -i
