from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class MeshStats:
    mesh_path: Path
    vertices: int
    triangles: int
    min_bound: tuple[float, float, float]
    max_bound: tuple[float, float, float]
    extent: tuple[float, float, float]


def resolve_mesh_path(path: Path | str) -> Path:
    mesh_path = Path(path)
    if mesh_path.is_dir():
        mesh_path = mesh_path / "mesh" / "object.ply"
    if not mesh_path.is_file():
        raise ValueError(f"Mesh file does not exist: {mesh_path}")
    return mesh_path


def load_mesh(mesh_path: Path | str):
    try:
        import open3d as o3d
    except ImportError as exc:
        raise RuntimeError(
            "Open3D is required for mesh visualization. Install it in the active "
            "environment, for example: python -m pip install --user open3d"
        ) from exc

    path = resolve_mesh_path(mesh_path)
    mesh = o3d.io.read_triangle_mesh(str(path))
    if mesh.is_empty():
        raise ValueError(f"Open3D loaded an empty mesh: {path}")
    if not mesh.has_vertex_normals():
        mesh.compute_vertex_normals()
    return path, mesh


def mesh_stats(mesh_path: Path | str) -> MeshStats:
    path, mesh = load_mesh(mesh_path)
    bbox = mesh.get_axis_aligned_bounding_box()
    return MeshStats(
        mesh_path=path,
        vertices=len(mesh.vertices),
        triangles=len(mesh.triangles),
        min_bound=tuple(float(value) for value in bbox.min_bound),
        max_bound=tuple(float(value) for value in bbox.max_bound),
        extent=tuple(float(value) for value in bbox.get_extent()),
    )


def visualize_mesh(
    mesh_path: Path | str,
    *,
    show_axes: bool = True,
    show_bbox: bool = True,
    window_name: str = "visual_grasp_manu mesh",
) -> MeshStats:
    try:
        import open3d as o3d
    except ImportError as exc:
        raise RuntimeError(
            "Open3D is required for mesh visualization. Install it in the active "
            "environment, for example: python -m pip install --user open3d"
        ) from exc

    path, mesh = load_mesh(mesh_path)
    bbox = mesh.get_axis_aligned_bounding_box()
    extent = bbox.get_extent()
    scale = float(max(np.max(extent), 0.05))

    geometries = [mesh]
    if show_bbox:
        bbox.color = (1.0, 0.7, 0.0)
        geometries.append(bbox)
    if show_axes:
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=scale * 0.35,
            origin=bbox.get_center() - np.asarray([scale * 0.55, scale * 0.55, scale * 0.55]),
        )
        geometries.append(frame)

    o3d.visualization.draw_geometries(
        geometries,
        window_name=window_name,
        mesh_show_back_face=True,
    )
    return mesh_stats(path)


def format_stats(stats: MeshStats) -> str:
    lines = [
        f"Mesh: {stats.mesh_path}",
        f"Vertices: {stats.vertices}",
        f"Triangles: {stats.triangles}",
        f"Min bound: {format_tuple(stats.min_bound)}",
        f"Max bound: {format_tuple(stats.max_bound)}",
        f"Extent: {format_tuple(stats.extent)}",
    ]
    return "\n".join(lines)


def format_tuple(values: tuple[float, float, float]) -> str:
    return "(" + ", ".join(f"{value:.6g}" for value in values) + ")"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Visualize a generated object mesh with Open3D."
    )
    parser.add_argument(
        "mesh_or_dataset_path",
        type=Path,
        help="Path to mesh/object.ply, another mesh file, or a scan dataset directory.",
    )
    parser.add_argument("--no-view", action="store_true", help="Print mesh stats without opening a viewer.")
    parser.add_argument("--no-axes", action="store_true", help="Hide the coordinate frame helper.")
    parser.add_argument("--no-bbox", action="store_true", help="Hide the axis-aligned bounding box helper.")
    parser.add_argument("--window-name", default="visual_grasp_manu mesh")
    args = parser.parse_args(argv)

    if args.no_view:
        stats = mesh_stats(args.mesh_or_dataset_path)
    else:
        stats = visualize_mesh(
            args.mesh_or_dataset_path,
            show_axes=not args.no_axes,
            show_bbox=not args.no_bbox,
            window_name=args.window_name,
        )
    print(format_stats(stats))
    print("Status: OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
