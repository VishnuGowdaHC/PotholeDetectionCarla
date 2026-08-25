# Multi-Sensor IoT-Based Pothole Detection System

Real-time road anomaly detection and mapping, built and validated entirely as a CARLA simulation. It is ready for integration with hardware components.

Built for the Unisys Innovation Program, Season 17.

## What I Built

- **Event-triggered sensor fusion pipeline.** A TOF depth sensor and an IMU accelerometer run always-on as low-power triggers. The global-shutter camera only wakes up when one of them flags an anomaly, so the system avoids continuous video processing entirely.
- **Two parallel detection pipelines off the same trigger data:**
  - Visual pipeline: TOF anomaly detection triggers camera capture, an on-device TFLite MobileNetV2 model classifies the frame, the hit gets GPS-tagged and pushed to the cloud.
  - Road Quality Index pipeline: IMU Z-axis spikes get logged and GPS-tagged independently, then clustered into a heatmap. This runs even without camera confirmation.
- **Distance-corrected GPS tagging.** The TOF sensor detects a dip before the wheel reaches it, so I projected the pothole's true location forward using vehicle heading instead of logging the sensor's own position.
- **CARLA simulation harness (Town02)** to validate detection logic before touching hardware: synchronous-mode sensor polling, a live telemetry dashboard (canvas-rendered heatmap of IMU, TOF, and fused hits plotted against the actual road network), scripted prop scattering for obstacle testing, and keyboard vehicle control.
- **Edge inference design.** MobileNetV2 trained via transfer learning (frozen base, then fine-tuning the last 30 layers), quantized to TFLite INT8 for on-device classification with no cloud dependency at inference time.

## Metrics

### Cost

Total component cost: **₹15,650**.

A continuous-video-tracking architecture (camera always on, model always running) needs a GPU-class edge board to keep up with real-time inference. A Jetson Nano setup with adequate heat dissipation and a camera suited to continuous capture runs about ₹30,000, versus the ₹6,200 Raspberry Pi 4 this design uses. Swapping that one component in raises total system cost to roughly ₹39,450. The system was architected to reduce continuous camera compute and power costs by ~60%, targeting deployment on a Raspberry Pi 4. This is achieved because it only spins up the camera and the classification model on a trigger instead of running both continuously, which is the actual source of the compute and power savings.

### Model Training

Transfer-learned MobileNetV2, fine-tuned after epoch 10. Final: **~84% train accuracy, ~80% val accuracy**.

## Tech Stack

Python, TFLite INT8 MobileNetV2, CARLA (Town02).

## Recognition

Selected for the Unisys Innovation Program, Season 17, out of 4,487 project submissions nationally. Advanced past Round 1, eliminated in Round 2 at approximately rank 32, **roughly the top 0.7% of all submissions**.
