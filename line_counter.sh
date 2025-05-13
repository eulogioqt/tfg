#!/bin/bash

# Buscar archivos
py_files=$(find ros2_ws/src -name '*.py')
cpp_files=$(find ros2_ws/src -type f \( -name '*.cpp' -o -name '*.hpp' -o -name '*.h' \))
js_files=$(find web_interface/src -type f \( -name '*.js' -o -name '*.jsx' \))

# Contar archivos
py_count=$(echo "$py_files" | wc -l)
cpp_count=$(echo "$cpp_files" | wc -l)
js_count=$(echo "$js_files" | wc -l)

# Contar líneas (si hay archivos)
py_lines=$(cat $py_files 2>/dev/null | wc -l)
cpp_lines=$(cat $cpp_files 2>/dev/null | wc -l)
js_lines=$(cat $js_files 2>/dev/null | wc -l)

# Totales
total_files=$((py_count + cpp_count + js_count))
total_lines=$((py_lines + cpp_lines + js_lines))

# Mostrar resultados
echo "Python files: $py_count, lines: $py_lines"
echo "C++ files:    $cpp_count, lines: $cpp_lines"
echo "JS/JSX files: $js_count, lines: $js_lines"
echo ""
echo "Total files:  $total_files, total lines: $total_lines"
