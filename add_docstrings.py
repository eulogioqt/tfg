import ast
from pathlib import Path

MODULE_TEMPLATE = '"""TODO: Add module documentation."""'
CLASS_TEMPLATE = '"""TODO: Describe class."""'


def func_template(node: ast.AST) -> str:
    params = []
    if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
        for arg in node.args.args:
            if arg.arg != 'self':
                params.append(f'    {arg.arg} (:obj:`Any`): TODO.')
        if node.args.vararg:
            params.append(f'    *{node.args.vararg.arg} (:obj:`Any`): TODO.')
        for kw in node.args.kwonlyargs:
            params.append(f'    {kw.arg} (:obj:`Any`): TODO.')
        if node.args.kwarg:
            params.append(f'    **{node.args.kwarg.arg} (:obj:`Any`): TODO.')
    params_str = "\n".join(params)
    body = "\nArgs:\n" + params_str if params else ""
    return f'"""TODO: Describe {node.name}.{body}\n"""'


def add_docstrings_to_file(path: Path) -> bool:
    text = path.read_text()
    try:
        module = ast.parse(text)
    except SyntaxError:
        return False

    lines = text.splitlines()
    insertions = []
    if ast.get_docstring(module) is None:
        insertions.append((0, MODULE_TEMPLATE))

    for node in ast.walk(module):
        if isinstance(node, ast.ClassDef):
            if ast.get_docstring(node) is None and node.body:
                indent = ' ' * node.col_offset
                line = node.body[0].lineno - 1
                insertions.append((line, indent + CLASS_TEMPLATE))
        elif isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            if ast.get_docstring(node) is None and node.body:
                indent = ' ' * node.col_offset
                line = node.body[0].lineno - 1
                doc = func_template(node)
                insertions.append((line, indent + doc))

    if not insertions:
        return False

    for idx, doc in sorted(insertions, reverse=True):
        lines.insert(idx, doc)

    path.write_text('\n'.join(lines) + '\n')
    return True


def main():
    changed = False
    for file in Path('ros2_ws/src').rglob('*.py'):
        if add_docstrings_to_file(file):
            print(f'Updated {file}')
            changed = True
    if not changed:
        print('No changes made')


if __name__ == '__main__':
    main()
