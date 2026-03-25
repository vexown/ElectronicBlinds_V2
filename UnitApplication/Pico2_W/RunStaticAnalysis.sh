#!/bin/bash
#===============================================================================
# RunStaticAnalysis.sh
#===============================================================================
# Description:
#   This script runs static code analysis on the Moduri project using Cppcheck.
#
# What is Cppcheck?
#   Cppcheck is an open-source static analysis tool for C/C++ code that detects
#   bugs, undefined behavior, and dangerous coding patterns without executing
#   the code. It's particularly good at finding memory leaks, buffer overflows,
#   and other common programming errors.
#
# Options Used:
#   --enable=all           : Enable all available checks (style, performance, 
#                            portability, information, warning, etc.)
#   --inconclusive         : Report even when the analysis is inconclusive
#                            (might increase false positives)
#   --force                : Force checking all configurations 
#                            (preprocessor conditionals)
#   --std=c11              : Use C11 standard for analysis
#   --template="{...}"     : Format the output messages (for better readability)
#   --suppress=*:/path/*   : Ignore warnings from third-party code
#   --suppress=missingInclude : Don't warn about missing includes that might 
#                               be system headers
#   --project=...          : Use compile_commands.json for accurate include paths
#
# Output:
#   All warnings and errors are saved to StaticAnalysis_output.txt
#
#===============================================================================

cppcheck --enable=all --inconclusive --force --std=c11 \
--template="{file}:{line}: {severity}: {message} [{id}]" \
--suppress=*:/home/blankmcu/pico/pico-sdk/* \
--suppress=*:/home/blankmcu/FreeRTOS_Kernel/* \
--suppress=missingInclude \
--project=build/compile_commands.json 2> StaticAnalysis_output.txt