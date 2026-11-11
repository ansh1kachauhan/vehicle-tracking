# Classical Vehicle Motion & Multi-Object Tracker

A modular, real-time classical computer vision pipeline for multi-vehicle detection, tracking, velocity estimation, and re-identification without deep learning.

---

## 📌 Highlights

- **Pure Classical Computer Vision**: Implements an end-to-end detection and tracking pipeline using traditional mathematical and geometric vision methods—no neural networks or GPU acceleration required.
- **Adaptive Foreground Segmentation**: Gaussian Mixture-based background subtraction (MOG2) with shadow suppression and morphological noise reduction.
- **Robust Feature Tracking**: Multi-scale pyramidal Lucas-Kanade (LK) optical flow coupled with forward-backward error validation.
- **RANSAC Motion Estimation**: Outlier-resilient affine transformation (`estimateAffinePartial2D`) and median displacement tracking to maintain stable vehicle bounding boxes.
- **Re-Identification Across Occlusions**: Feature descriptor matching (ORB / SIFT) with Lowe's ratio test and RANSAC homography verification to recover lost tracks.
- **Rich Visual Diagnostics**: Color-coded tracking IDs, active feature point overlays, historical motion trails, direction vectors, and real-time speed estimation (`px/frame`).

---

## 🏗️ Architecture & Pipeline Overview

```
                      [ Input Video Frame ]
                                │
                 ┌──────────────┴──────────────┐
                 ▼                             ▼
        [ Grayscale + Blur ]           [ MOG2 Background Subtraction ]
                 │                             │
                 │                     [ Road ROI Masking ]
                 │                             │
                 │                     [ Morphological Filtering ]
                 │                             │
                 │                     [ Contour Detection & Area Filtering ]
                 │                             │
                 │                     [ Detected Vehicle Bounding Boxes ]
                 │                             │
                 └──────────────┬──────────────┘
                                ▼
              [ Lucas-Kanade Optical Flow (t-1 -> t) ]
                                │
              [ Forward-Backward Error Check (fb_err < 1.0) ]
                                │
              [ RANSAC Affine Estimation & Box Displacement ]
                                │
              [ Periodic Association (IoU Matching) ]
                                │
         ┌──────────────────────┴──────────────────────┐
         ▼                                             ▼
  [ Active Tracks ]                             [ Unmatched Detections ]
   • Refreshed features                          • Feature extraction
   • Trail & speed update                        • Re-ID matching vs. Lost Tracks
                                                   (ORB/SIFT + RANSAC Homography)
                                                 • Spawn new track if no match
```

---

## 📁 Repository Structure

```text
classical tracking/
├── vehicle_tracker.py       # Main pipeline implementation and CLI runner
├── traffic.mp4              # Sample input traffic surveillance video
├── tracked.mp4              # Generated output tracked video (ORB)
├── tracked_sift.mp4         # Generated output tracked video (SIFT)
└── README.md                # Project documentation
```

---

## ⚙️ Installation & Setup

### Prerequisites
- Python 3.8 or higher
- Recommended: Virtual environment (`venv` or `conda`)

### 1. Clone or Open the Repository
```bash
cd "C:\Users\c31an\Desktop\classical tracking"
```

### 2. Set Up Virtual Environment (Optional but recommended)
```bash
python -m venv venv

# Windows
venv\Scripts\activate

# Linux / macOS
source venv/bin/activate
```

### 3. Install Dependencies
```bash
pip install opencv-python opencv-contrib-python numpy imageio
```

> **Note**: `opencv-contrib-python` is required if using SIFT descriptors and advanced OpenCV tracking modules.

---

## 🚀 Usage

### 1. Basic Tracking (Default: ORB)
Processes `traffic.mp4` and outputs annotations to `tracked.mp4`:
```bash
python vehicle_tracker.py --input traffic.mp4 --output tracked.mp4
```

### 2. High-Accuracy Tracking with SIFT
```bash
python vehicle_tracker.py --input traffic.mp4 --output tracked_sift.mp4 --feature sift
```

### 3. Live Preview Window
To visualize tracking in real-time as frames are processed:
```bash
python vehicle_tracker.py --input traffic.mp4 --show
```
*(Press `Esc` during playback to terminate early).*

### 4. Process Specific Segment & Generate Animated GIF
```bash
python vehicle_tracker.py --input traffic.mp4 --start 2.0 --duration 10.0 --gif demo.gif --gif-fps 10
```

---

## 🔧 Command-Line Arguments

| Argument | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `--input` | `str` | `traffic.mp4` | Path to the source video file |
| `--output` | `str` | `tracked.mp4` | Destination path for the rendered output video |
| `--feature` | `str` | `orb` | Feature detector type: `orb` (fast) or `sift` (robust) |
| `--width` | `int` | `960` | Target frame width for processing and aspect-ratio scaling |
| `--start` | `float` | `0.0` | Video start timestamp in seconds |
| `--duration` | `float` | `0.0` | Duration to process in seconds (`0` = full video) |
| `--warmup` | `int` | `60` | Number of initial frames to initialize background model |
| `--min-area` | `int` | `700` | Minimum contour area threshold for vehicle detection |
| `--show` | `flag` | `False` | Display interactive preview window (`cv2.imshow`) |
| `--gif` | `str` | `None` | Optional file path to export an animated GIF preview |
| `--gif-fps` | `int` | `8` | Frame rate for output GIF |
| `--gif-width` | `int` | `480` | Width of the output GIF |

---

## 🔬 Technical Deep-Dive

### 1. Motion Segmentation & Region of Interest (ROI)
- **Background Subtraction**: Uses `cv2.createBackgroundSubtractorMOG2` with `detectShadows=True`. Shadow pixels (gray value `127`) are explicitly filtered out to avoid merging shadow footprints into vehicle silhouettes.
- **Geometric ROI**: Constrains detection to the drivable roadway polygon to prevent false detections from roadside trees, clouds, or sky.
- **Morphological Clean-up**: Elliptical morphological opening ($3\times3$) removes speckle noise, followed by closing ($11\times11$) and dilation to fuse separated body parts into unified vehicle blobs.

### 2. Optical Flow & Trajectory Smoothing
- **Pyramidal Lucas-Kanade**: Tracks feature points between frame $t-1$ and frame $t$ across 3 pyramid levels.
- **Forward-Backward Consistency Validation**: Checks bidirectional optical flow ($p_0 \rightarrow p_1 \rightarrow p_0'$). Points with Euclidean distance $\lVert p_0 - p_0' \rVert \ge 1.0\text{ px}$ are dropped as occlusions or track failures.
- **Affine RANSAC**: Computes robust partial affine transformation (`cv2.estimateAffinePartial2D`) between inlier keypoints, filtering out dynamic feature deviations.
- **Displacement**: Updates bounding box coordinates using the median coordinate shift of confirmed inliers.
- **Speed & Motion Trail**: Tracks the center point over the last 40 frames and computes an exponential moving average (EMA) speed:
  $$\text{speed}_t = 0.8 \cdot \text{speed}_{t-1} + 0.2 \cdot \Delta d$$

### 3. Re-Identification (Re-ID) & Occlusion Recovery
When vehicles are occluded or briefly lost:
- Trackers transition to a `lost_tracks` buffer for up to 25 frames (`max_lost=25`).
- Unmatched new detections are matched against lost tracks using feature descriptors (`BFMatcher` + Lowe's ratio test with $0.75$).
- Candidate matches are validated using **RANSAC Homography** (`cv2.findHomography`). If $\ge 12$ geometric inliers match, the vehicle track is revived with its original unique ID and trajectory history.

---

## 📈 Performance Comparison: ORB vs. SIFT

| Metric | ORB (`--feature orb`) | SIFT (`--feature sift`) |
| :--- | :--- | :--- |
| **Speed / FPS** | High (~Real-time) | Moderate |
| **Descriptor Distance** | Hamming Distance | $L_2$ Euclidean Norm |
| **Scale Invariance** | Pyramidal (good) | Scale-space octave extrema (superior) |
| **Occlusion Re-ID** | Solid for distinct textures | Superior under complex viewpoints/rotations |
| **Recommended Use** | Real-time surveillance | Detailed offline video analysis |
