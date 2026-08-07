# Understanding Inference Runtime Terms for FoundationPose

This article explains the basic deployment terms behind FoundationPose, ONNX, TensorRT, CUDA, and model inference in plain language.

## The Main Idea

The robot needs to run a neural network that looks at camera data and predicts something useful.

In this project, FoundationPose predicts where an object is in 3D. After that, the grasp pipeline can use the object pose to generate possible grasp candidates.

## Training vs Inference

Training is when a model learns from a large dataset. This is usually slow, expensive, and needs a lot of compute.

Inference is when an already-trained model is used to make predictions.

For this project, the goal is inference. The project does not need to train FoundationPose.

## Model Files

A trained model is stored in files that contain learned numeric values called weights.

Common model file formats include:

- `.pt` or `.pth` for PyTorch models,
- `.onnx` for ONNX models,
- `.plan` or `.engine` for TensorRT-optimized models.

These files can be large and may have license restrictions. They should not be committed to the repository unless they are intentionally reviewed for public release.

## PyTorch

PyTorch is a popular deep learning framework.

It is useful for research because models are easy to build, modify, and debug. But raw PyTorch is often not the fastest option when a robot needs stable real-time performance.

Many models start in PyTorch during research and are later exported to another format for deployment.

## ONNX

ONNX is a portable model format.

Think of ONNX as a neutral export format. A model can be trained in PyTorch, exported to ONNX, and then loaded by other inference tools.

The flow often looks like this:

```text
PyTorch model
-> ONNX file
-> deployment runtime
```

ONNX itself does not automatically make a model fast. It makes the model easier to move into tools that can optimize it.

## TensorRT

TensorRT is NVIDIA's tool for making neural networks run fast on NVIDIA GPUs.

TensorRT takes an ONNX model and builds an optimized version for a specific NVIDIA GPU. This optimized version usually runs faster and more predictably than the original research model.

The flow looks like this:

```text
ONNX file
-> TensorRT optimization
-> TensorRT engine file
-> fast inference on NVIDIA GPU
```

## TensorRT Engine Files

A TensorRT engine file is the optimized model file created by TensorRT.

It often has a `.plan` or `.engine` extension.

For FoundationPose, the optimized files are typically:

```text
refine_trt_engine.plan
score_trt_engine.plan
```

A TensorRT engine is not as portable as an ONNX file. It is built for a specific GPU, CUDA version, TensorRT version, and model input shape range.

## CUDA

CUDA is NVIDIA's GPU computing system.

If a model says it needs CUDA, it means it expects an NVIDIA GPU and the matching NVIDIA software stack.

This matters because pose estimation and grasp inference can be too slow on CPU for real-time robot use.

## Why FoundationPose Has Two Models

FoundationPose uses two main model parts:

- `refine_model`,
- `score_model`.

The refine model improves possible object poses.

The score model chooses the best pose from those possibilities.

That is why FoundationPose setup usually involves two ONNX files:

```text
refine_model.onnx
score_model.onnx
```

Both need to be converted into TensorRT engine files for fast NVIDIA GPU inference:

```text
refine_model.onnx
-> refine_trt_engine.plan

score_model.onnx
-> score_trt_engine.plan
```

## How This Fits the Project

FoundationPose should answer one question:

```text
Where is the object?
```

The grasp pipeline should answer the next question:

```text
Given the object pose, where can the robot grasp it?
```

Keeping those responsibilities separate is important.

It means the grasp-candidate part can be developed and tested before the full FoundationPose GPU setup is ready.

## Recommended Development Order

Start with a simple fake object pose:

```text
fake object pose
-> grasp candidates
-> RViz visualization
```

Then add more realism:

```text
depth or point-cloud filtering
-> better candidate scoring
-> rosbag replay demo
```

Then add FoundationPose:

```text
RGB-D camera data
-> FoundationPose
-> real object pose
-> grasp candidates
-> RViz visualization
```

## Main Things That Can Go Wrong

Common problems include:

- the ONNX model exists, but the TensorRT engine was not built;
- the TensorRT engine was built for a different GPU or software version;
- CUDA is not available inside the container;
- GPU memory is not enough during engine conversion;
- RGB and depth images are not aligned;
- camera calibration is wrong;
- the CAD mesh scale does not match the real object;
- the segmentation mask or object detection is poor;
- model files are accidentally committed to git.

## Short Summary

Use ONNX as the portable model format.

Use TensorRT engine files for fast NVIDIA GPU inference.

Use FoundationPose to estimate the object pose.

Use this package to turn the object pose into grasp candidates and visualizations.
