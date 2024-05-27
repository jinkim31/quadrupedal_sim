

import os

package_name = 'package'
dir = 'meshes'
targets = {}
root = os.path.join(os.getcwd(), dir)
print("ROOT", root)
for path, subdirs, files in os.walk(root):
    for name in files:
        abs_path = os.path.join(path, name)
        rel_path = os.path.relpath(abs_path, os.getcwd())
        print(rel_path)
        targets.append(rel_path)
print(os.path.join('share', package_name, dir), targets)
