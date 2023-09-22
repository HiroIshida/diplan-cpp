#!/bin/bash
find cpp/* | xargs clang-format -i
find tests/* | xargs clang-format -i
