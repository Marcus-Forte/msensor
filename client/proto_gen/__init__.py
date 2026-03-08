import sys
import os

# Allow generated gRPC stubs to use bare imports (e.g. `import lidar_pb2`)
# by adding this package's directory to sys.path.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
