#!/bin/bash

py_files=$(find ros2_ws/src -name '*.py')
js_files=$(find web_interface/src -type f \( -name '*.js' -o -name '*.jsx' \))

py_count=$(echo "$py_files" | wc -l)
py_lines=$(cat $py_files 2>/dev/null | wc -l)

js_count=$(echo "$js_files" | wc -l)
js_lines=$(cat $js_files 2>/dev/null | wc -l)

total_files=$((py_count + js_count))
total_lines=$((py_lines + js_lines))

echo "Python files: $py_count, lines: $py_lines"
echo "JS/JSX files: $js_count, lines: $js_lines"
echo "Total files: $total_files, total lines: $total_lines"
