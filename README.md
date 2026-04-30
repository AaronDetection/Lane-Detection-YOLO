# Lane Detection — YOLOv8 & Visual Servoing

Real-time lane separator detection for an autonomous Tetrix robot, integrated into a visual servoing loop for trajectory control.

***********************
Find the Dataset .zip here: [Tetrix +800 images on Roboflow](https://app.roboflow.com/blueberriesaaron/tetrix-t1/models)
**********************

---

## What this project does

Detects lane separators and block obstacles from a live camera feed, computes the midpoint coordinate between detected separators, and sends that signal to the robot's steering system via UART.

| Class | Bounding Box |
|---|---|
| Lane separator | Purple |
| Block obstacle | Yellow |

The hard constraint: detection had to hit >95% precision without dropping FPS below real-time threshold. That ruled out heavier architectures and shaped every configuration decision.

---

## Stack

- **Model:** YOLOv8-Small (Ultralytics)
- **Annotation:** Roboflow (800+ images, manual)
- **Integration:** UART serial communication to Tetrix controller
- **Training:** Google Colab

---

## Key results

| Metric | Value |
|---|---|
| Precision | >0.90 |
| Recall | ~0.85 |
| mAP@50 | 0.95 |
| mAP@50-95 | 0.70 |

The mAP@50-95 score is what enables the spatial calculation — bounding box localization is precise enough to compute reliable midpoint coordinates for trajectory control. Without that accuracy, visual servoing drifts.

---

## Why YOLOv8-Small over Medium/Large

Latency. The robot's camera feed needed sub-100ms inference per frame. Small hits the precision target while keeping the pipeline fast enough for live control. Bigger model = more accurate boxes, slower feedback loop = worse steering.

---

## Dataset

[Tetrix T1 - v6 on Roboflow](https://app.roboflow.com/blueberriesaaron/tetrix-t1/models) — 800+ manually captured images, custom dataset
