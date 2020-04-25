import numpy
from stl import mesh, Mode
import argparse

def main():
  parser = argparse.ArgumentParser("STL rectifier (mm -> m)")
  parser.add_argument("input_path", help="input stl file path")
  parser.add_argument("output_path", help="output stl file path")
  parser.add_argument("scale", type=float, default=1e-3, help="the ratio output / input")
  args = parser.parse_args()

  stl = mesh.Mesh.from_file(args.input_path)

  for p in stl.points:
    p *= args.scale

  stl.save(args.output_path, mode=Mode.ASCII)


if __name__ == "__main__":
  main()
