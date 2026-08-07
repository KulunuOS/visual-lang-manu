# Inference Runtime Background

This note explains the deployment terms that appear in the FoundationPose and Isaac ROS documentation.

## Model Training vs Model Inference

Training is the process of learning model weights from data. It is expensive, slow, and usually needs large datasets and GPUs.

Inference is using an already trained model to make predictions. In this project, inference means taking RGB-D sensor input and producing something useful for the robot, such as an object pose or grasp candidates.

For the first demo, the goal is inference only. The project should not train FoundationPose.

## Model Weights

Model weights are the learned numeric parameters of a neural network. They are stored in files such as:

- `.pt` or `.pth` for PyTorch,
- `.onnx` for ONNX,
- `.engine` or `.plan` for TensorRT runtime engines.

Weights and runtime artifacts can be large. Do not commit model files unless they are intentionally reviewed for public release and their license allows redistribution.

## PyTorch

PyTorch is a deep learning framework commonly used for research code.

It is convenient for development because models are easy to write, debug, and modify. The tradeoff is that raw PyTorch inference is often not the fastest deployment option, especially in robotics systems that need stable frame rates.

Many model repositories start in PyTorch, then export to another format for deployment.

## ONNX

ONNX means Open Neural Network Exchange.

Think of ONNX as a portable model format. It describes the model graph and weights in a framework-neutral way so the model can be loaded by different runtimes.

ONNX is useful because it can act as a bridge:

```text
PyTorch or training framework
  -> exported ONNX model
  -> optimized runtime such as ONNX Runtime or TensorRT
```

ONNX itself is not automatically the fastest possible runtime. It is a standard interchange format that makes optimization and deployment easier.

## ONNX Runtime

ONNX Runtime is a runtime that can execute ONNX models.

It can run on CPU, CUDA GPU, or other providers depending on what is installed. For many models, ONNX Runtime is faster and easier to deploy than raw PyTorch. It is often a good middle ground for projects that need portability.

For FoundationPose in Isaac ROS, NVIDIA's intended path is TensorRT rather than plain ONNX Runtime.

## TensorRT

TensorRT is NVIDIA's high-performance inference optimizer and runtime.

It takes a trained model, usually through ONNX, and builds an optimized version for a specific NVIDIA GPU and precision mode. TensorRT can apply optimizations such as:

- layer fusion,
- kernel selection,
- memory planning,
- lower-precision math such as FP16 when supported,
- shape-specific optimization.

The result is usually much faster and more predictable than running the same model through a research framework.

The tradeoff is portability. A TensorRT engine is tied to the target hardware, TensorRT version, CUDA version, and model input shapes more tightly than ONNX is.

## TensorRT Engine or Plan File

A TensorRT engine file, often named `.engine` or `.plan`, is the optimized runtime artifact produced by TensorRT.

Typical flow:

```text
refine_model.onnx
  -> trtexec
  -> refine_trt_engine.plan

score_model.onnx
  -> trtexec
  -> score_trt_engine.plan
```

The `.onnx` files are portable model descriptions. The `.plan` files are optimized binaries for running inference efficiently on the target NVIDIA platform.

Do not assume a `.plan` file built on one machine will work on every other machine.

## CUDA

CUDA is NVIDIA's GPU computing platform.

When documentation says a package needs CUDA, it means the model or runtime expects an NVIDIA GPU and the matching NVIDIA software stack. CPU-only machines cannot use CUDA acceleration.

For robotics perception, CUDA matters because RGB-D pose estimation and grasp inference can be too slow on CPU for real-time operation.

## FP32, FP16, and INT8

These terms describe numeric precision:

- FP32: 32-bit floating point. Most accurate, usually slower and more memory-heavy.
- FP16: 16-bit floating point. Faster and lower memory on supported GPUs, but can sometimes reduce accuracy.
- INT8: 8-bit integer. Fastest and smallest, but usually needs calibration and can reduce accuracy if not handled carefully.

TensorRT often uses FP16 for speed when the model supports it. Some models or TensorRT versions may require FP32 for accuracy.

## Why FoundationPose Has Two Models

FoundationPose uses two main neural network stages:

- refine model: improves pose hypotheses,
- score model: ranks pose hypotheses and selects the best one.

That is why the Isaac ROS quickstart downloads and converts two ONNX files:

- `refine_model.onnx`,
- `score_model.onnx`.

Both must be available as TensorRT engine plans for the optimized Isaac ROS runtime.

## How FoundationPose Fits This Project

For this repository, FoundationPose should be treated as a pose provider:

```text
RGB image
depth image
camera info
segmentation mask or detection
CAD mesh or reference captures
  -> FoundationPose
  -> object 6-DoF pose
```

Then this package can generate grasp candidates:

```text
object 6-DoF pose
CAD-frame grasp library
depth or point cloud
  -> grasp candidate node
  -> ranked grasp candidates
  -> RViz visualization
```

This separation matters. The grasp candidate node can be developed and tested with a simple pose-topic stub before the full FoundationPose GPU stack is installed.

## Practical Development Path

Recommended order:

1. Build a pose-topic stub that publishes a fixed or bag-derived object pose.
2. Build grasp candidate transformation and RViz visualization.
3. Add depth or point-cloud filtering.
4. Install Isaac ROS FoundationPose in the dev container.
5. Download FoundationPose ONNX files outside git.
6. Convert ONNX files to TensorRT `.plan` files on the target NVIDIA machine.
7. Wire FoundationPose pose output into the grasp candidate node.
8. Measure latency and tune the pipeline.

## Common Failure Modes

- ONNX model exists, but TensorRT engine was not built.
- TensorRT engine was built for a different hardware or software stack.
- CUDA is not available inside the container.
- GPU memory is insufficient during engine conversion.
- RGB and depth images are not aligned.
- Camera calibration is wrong.
- The object mesh scale does not match the physical object.
- The segmentation mask or bounding box is poor, causing bad pose initialization.
- Model files are placed in git by mistake.

## Rule of Thumb

Use ONNX as the portable model format.

Use TensorRT `.plan` files for fast NVIDIA deployment.

Use ROS 2 topics to keep heavyweight inference separate from grasp-candidate logic.
