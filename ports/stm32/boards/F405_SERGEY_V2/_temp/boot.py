import os
from flashbdev import bdev

try:
    os.mount(bdev, "/flash")
except OSError:
    import uos
    uos.VfsLfs2.mkfs(bdev)
    os.mount(bdev, "/flash")

os.chdir("/flash")
print("Mounted external flash at /flash")

