#!/bin/bash
find src include -regex '.*\.\(cpp\|hpp\|cc\|cxx\|h\)' -exec clang-format-10 -style=file -i {} \;
