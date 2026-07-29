import os

files = [
    "BUILD.bazel",
    "README.md",
    "config/quadrotor_sim.yaml",
    "src/quadrotor_se3_controller.cc",
    "src/quadrotor_sim.cc",
    "src/quadrotor_visualizer.cc",
    "systems/se3_controller.cc",
    "systems/se3_controller.h"
]

for f in files:
    with open(f, "r") as file:
        content = file.read()
    if "<<<<<<< HEAD" in content:
        print(f"--- {f} ---")
        blocks = content.split('<<<<<<< HEAD\n')
        for i, block in enumerate(blocks[1:]):
            parts = block.split('=======\n')
            if len(parts) < 2: continue
            head = parts[0]
            master = parts[1].split('>>>>>>>')[0]
            print(f"Conflict {i+1}:")
            print("[HEAD (OURS)]\n" + head)
            print("[MASTER]\n" + master)
            print("-" * 40)
