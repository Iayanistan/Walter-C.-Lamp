# Walter C. Lamp

A FAIRINO FR3 robot cell that runs ASTM D4607 (iodine number) steps. It pairs a deterministic procedure path with a GR00T VLA that detects and recovers from errors.

| Folder | What it is |
| --- | --- |
| `controls/` | ROS 2 Jazzy workspace (`walter_ws`), FR3 Python helpers (`fr3_python/walter_fr3`), Isaac Sim scene (`sim/isaac`), GR00T fine-tune/serve scripts (`vla`), procedure builder skeleton, docs |
| `vision_poc/` | Computer vision proof of concept: vessel detector (YOLO, `models/vessels.pt`) and spill detection |

Start with `controls/README.md`.
