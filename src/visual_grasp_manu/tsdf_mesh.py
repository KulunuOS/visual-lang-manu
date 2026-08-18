from __future__ import annotations

import argparse
import copy
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import yaml

from visual_grasp_manu.mask_generation import ensure_binary_mask, read_rgb_image
from visual_grasp_manu.scan_dataset import (
    DEPTH_EXTENSIONS,
    MASK_EXTENSIONS,
    RGB_EXTENSIONS,
    format_report,
    read_yaml_file,
    validate_scan_dataset,
)


@dataclass(frozen=True)
class TsdfMeshResult:
    dataset_path: Path
    mesh_path: Path
    frames_integrated: int
    vertices: int
    triangles: int


@dataclass(frozen=True)
class FusionFrame:
    stem: str
    rgb_path: Path
    depth_path: Path
    mask_path: Path
    pose_path: Path


def generate_tsdf_mesh(
    dataset_path: Path | str,
    *,
    output_path: Path | str | None = None,
    voxel_length: float = 0.004,
    sdf_trunc: float = 0.02,
    depth_trunc: float = 1.5,
    frame_stride: int = 1,
    max_frames: int = 0,
    min_mask_pixels: int = 100,
    keep_largest_component: bool = False,
    frame_list_path: Path | str | None = None,
) -> TsdfMeshResult:
    try:
        import open3d as o3d
    except ImportError as exc:
        raise RuntimeError(
            "Open3D is required for TSDF fusion. Install it in the active "
            "environment, for example: python -m pip install --user open3d"
        ) from exc

    path = Path(dataset_path)
    report = validate_scan_dataset(path, stage="masks")
    if not report.ok:
        raise ValueError(format_report(report))

    metadata_errors: list[str] = []
    intrinsics_data = read_yaml_file(path / "camera_intrinsics.yaml", metadata_errors, required=True)
    if metadata_errors:
        raise ValueError("\n".join(metadata_errors))

    depth_scale = float(intrinsics_data["depth_scale"])
    intrinsic = o3d.camera.PinholeCameraIntrinsic(
        int(intrinsics_data["width"]),
        int(intrinsics_data["height"]),
        float(intrinsics_data["fx"]),
        float(intrinsics_data["fy"]),
        float(intrinsics_data["cx"]),
        float(intrinsics_data["cy"]),
    )

    frames = collect_fusion_frames(path)
    if frame_list_path is not None:
        accepted_stems = read_frame_list(frame_list_path)
        frames = [frame for frame in frames if frame.stem in accepted_stems]
    frame_stride = max(1, frame_stride)
    frames = frames[::frame_stride]
    if max_frames > 0:
        frames = frames[:max_frames]
    if not frames:
        raise ValueError(f"No matching rgb/depth/mask/camera_pose frames found in: {path}")

    volume = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=voxel_length,
        sdf_trunc=sdf_trunc,
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8,
    )

    frames_integrated = 0
    for frame in frames:
        rgb = read_rgb_image(frame.rgb_path)
        depth = read_depth(frame.depth_path)
        mask = read_mask(frame.mask_path)
        if int(np.count_nonzero(mask)) < min_mask_pixels:
            continue

        masked_depth = apply_mask_to_depth(depth, mask)
        masked_rgb = apply_mask_to_rgb(rgb, mask)
        color_o3d = o3d.geometry.Image(np.ascontiguousarray(masked_rgb))
        depth_o3d = o3d.geometry.Image(np.ascontiguousarray(masked_depth))
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color_o3d,
            depth_o3d,
            depth_scale=1.0 / depth_scale,
            depth_trunc=depth_trunc,
            convert_rgb_to_intensity=False,
        )
        camera_to_world = read_pose_matrix(frame.pose_path)
        world_to_camera = np.linalg.inv(camera_to_world)
        volume.integrate(rgbd, intrinsic, world_to_camera)
        frames_integrated += 1

    if frames_integrated == 0:
        raise ValueError("No frames were integrated. Check masks, min_mask_pixels, and depth validity.")

    mesh = volume.extract_triangle_mesh()
    if keep_largest_component:
        mesh = keep_largest_triangle_component(mesh)
    mesh.compute_vertex_normals()
    output = Path(output_path) if output_path is not None else path / "mesh" / "object.ply"
    output.parent.mkdir(parents=True, exist_ok=True)
    if not o3d.io.write_triangle_mesh(str(output), mesh):
        raise ValueError(f"Could not write mesh: {output}")

    result = TsdfMeshResult(
        dataset_path=path,
        mesh_path=output,
        frames_integrated=frames_integrated,
        vertices=len(mesh.vertices),
        triangles=len(mesh.triangles),
    )
    write_mesh_metadata(
        path,
        result,
        voxel_length,
        sdf_trunc,
        depth_trunc,
        frame_stride,
        max_frames,
        keep_largest_component,
        frame_list_path,
    )
    return result


def keep_largest_triangle_component(mesh):
    labels, counts, _ = mesh.cluster_connected_triangles()
    counts = np.asarray(counts)
    if len(counts) <= 1:
        return mesh

    labels = np.asarray(labels)
    largest_label = int(np.argmax(counts))
    remove_mask = labels != largest_label
    cleaned = copy.deepcopy(mesh)
    cleaned.remove_triangles_by_mask(remove_mask.tolist())
    cleaned.remove_unreferenced_vertices()
    return cleaned


def collect_fusion_frames(dataset_path: Path | str) -> list[FusionFrame]:
    path = Path(dataset_path)
    rgb_files = collect_by_stem(path / "rgb", RGB_EXTENSIONS)
    depth_files = collect_by_stem(path / "depth", DEPTH_EXTENSIONS)
    mask_files = collect_by_stem(path / "masks", MASK_EXTENSIONS)
    pose_files = collect_by_stem(path / "camera_poses", {".txt"})

    frames: list[FusionFrame] = []
    for stem, rgb_path in sorted(rgb_files.items()):
        depth_path = depth_files.get(stem)
        mask_path = mask_files.get(stem)
        pose_path = pose_files.get(stem)
        if depth_path is None or mask_path is None or pose_path is None:
            continue
        frames.append(FusionFrame(stem, rgb_path, depth_path, mask_path, pose_path))
    return frames


def read_frame_list(path: Path | str) -> set[str]:
    frame_list_path = Path(path)
    if not frame_list_path.is_file():
        raise ValueError(f"Frame list does not exist: {frame_list_path}")
    stems = {
        line.strip()
        for line in frame_list_path.read_text(encoding="utf-8").splitlines()
        if line.strip() and not line.strip().startswith("#")
    }
    if not stems:
        raise ValueError(f"Frame list is empty: {frame_list_path}")
    return stems


def collect_by_stem(directory: Path, extensions: set[str]) -> dict[str, Path]:
    if not directory.is_dir():
        return {}
    return {
        path.stem: path
        for path in sorted(directory.iterdir())
        if path.is_file() and path.suffix.lower() in extensions
    }


def read_depth(path: Path) -> np.ndarray:
    if path.suffix.lower() == ".npy":
        return np.load(path)
    depth = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if depth is None:
        raise ValueError(f"Could not read depth image: {path}")
    return depth


def read_mask(path: Path) -> np.ndarray:
    if path.suffix.lower() == ".npy":
        return ensure_binary_mask(np.load(path))
    mask = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if mask is None:
        raise ValueError(f"Could not read mask image: {path}")
    return ensure_binary_mask(mask)


def read_pose_matrix(path: Path) -> np.ndarray:
    matrix = np.loadtxt(path, dtype=np.float64)
    if matrix.shape != (4, 4):
        raise ValueError(f"Camera pose must be a 4x4 matrix: {path}")
    return matrix


def apply_mask_to_depth(depth: np.ndarray, mask: np.ndarray) -> np.ndarray:
    if depth.shape[:2] != mask.shape[:2]:
        raise ValueError(f"Depth shape {depth.shape[:2]} does not match mask shape {mask.shape[:2]}")
    masked = np.asarray(depth).copy()
    masked[ensure_binary_mask(mask) == 0] = 0
    return masked


def apply_mask_to_rgb(rgb: np.ndarray, mask: np.ndarray) -> np.ndarray:
    if rgb.shape[:2] != mask.shape[:2]:
        raise ValueError(f"RGB shape {rgb.shape[:2]} does not match mask shape {mask.shape[:2]}")
    masked = np.asarray(rgb).copy()
    masked[ensure_binary_mask(mask) == 0] = 0
    return masked


def write_mesh_metadata(
    dataset_path: Path,
    result: TsdfMeshResult,
    voxel_length: float,
    sdf_trunc: float,
    depth_trunc: float,
    frame_stride: int,
    max_frames: int,
    keep_largest_component: bool,
    frame_list_path: Path | str | None,
) -> None:
    data: dict[str, Any] = {
        "mesh_path": str(result.mesh_path.relative_to(dataset_path)),
        "backend": "open3d_tsdf",
        "frames_integrated": result.frames_integrated,
        "vertices": result.vertices,
        "triangles": result.triangles,
        "voxel_length": voxel_length,
        "sdf_trunc": sdf_trunc,
        "depth_trunc": depth_trunc,
        "frame_stride": frame_stride,
        "max_frames": max_frames,
        "keep_largest_component": keep_largest_component,
        "frame_list": str(frame_list_path) if frame_list_path is not None else None,
    }
    (dataset_path / "mesh_metadata.yaml").write_text(
        yaml.safe_dump(data, sort_keys=False),
        encoding="utf-8",
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Fuse masked multi-view RGB-D scan frames into an object mesh using Open3D TSDF."
    )
    parser.add_argument("dataset_path", type=Path, help="Scan dataset directory.")
    parser.add_argument("--output", type=Path, default=None, help="Output mesh path. Defaults to mesh/object.ply.")
    parser.add_argument("--voxel-length", type=float, default=0.004, help="TSDF voxel length in meters.")
    parser.add_argument("--sdf-trunc", type=float, default=0.02, help="TSDF truncation distance in meters.")
    parser.add_argument("--depth-trunc", type=float, default=1.5, help="Maximum depth in meters.")
    parser.add_argument("--frame-stride", type=int, default=1, help="Use every Nth masked frame.")
    parser.add_argument("--max-frames", type=int, default=0, help="Maximum frames to integrate. 0 means all.")
    parser.add_argument("--min-mask-pixels", type=int, default=100, help="Skip frames with fewer mask pixels.")
    parser.add_argument(
        "--frame-list",
        type=Path,
        default=None,
        help="Optional newline-delimited frame stems to fuse, e.g. accepted_frames.txt.",
    )
    parser.add_argument(
        "--keep-largest-component",
        action="store_true",
        help="Remove disconnected mesh islands and keep only the largest triangle component.",
    )
    args = parser.parse_args(argv)

    result = generate_tsdf_mesh(
        args.dataset_path,
        output_path=args.output,
        voxel_length=args.voxel_length,
        sdf_trunc=args.sdf_trunc,
        depth_trunc=args.depth_trunc,
        frame_stride=args.frame_stride,
        max_frames=args.max_frames,
        min_mask_pixels=args.min_mask_pixels,
        keep_largest_component=args.keep_largest_component,
        frame_list_path=args.frame_list,
    )
    print(f"Scan dataset: {result.dataset_path}")
    print(f"Frames integrated: {result.frames_integrated}")
    print(f"Vertices: {result.vertices}")
    print(f"Triangles: {result.triangles}")
    print(f"Mesh: {result.mesh_path}")
    print("Status: OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
