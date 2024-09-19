import sys
import os
from pathlib import Path


cur_file = os.path.abspath(__file__)
cur_dir = os.path.dirname(cur_file)
cur_dir_path = Path(cur_dir)

print(f'File {cur_file}')
print(f'Dir {cur_dir}')

parent_dir1 = os.path.join(cur_dir, '..')
# parent_dir2 = cur_dir.parent
#
print(parent_dir1)

parent_dir2 = cur_dir_path.parent.absolute()
print(parent_dir2)

parent_dir3 = parent_dir2.parent.absolute()
print(parent_dir3)
