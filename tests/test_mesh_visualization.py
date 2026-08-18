from pathlib import Path

import numpy as np
import open3d as o3d

from visual_grasp_manu.mesh_visualization import (
    format_stats,
    mesh_stats,
    resolve_mesh_path,
)


def write_box_mesh(path: Path) -> None:
    mesh = o3d.geometry.TriangleMesh.create_box(width=0.1, height=0.2, depth=0.3)
    mesh.compute_vertex_normals()
    assert o3d.io.write_triangle_mesh(str(path), mesh)


def test_resolve_mesh_path_accepts_dataset_directory(tmp_path: Path):
    mesh_dir = tmp_path / "scan" / "mesh"
    mesh_dir.mkdir(parents=True)
    mesh_path = mesh_dir / "object.ply"
    mesh_path.write_text("ply\n", encoding="utf-8")

    assert resolve_mesh_path(tmp_path / "scan") == mesh_path


def test_mesh_stats_reports_geometry(tmp_path: Path):
    mesh_path = tmp_path / "object.ply"
    write_box_mesh(mesh_path)

    stats = mesh_stats(mesh_path)

    assert stats.vertices == 8
    assert stats.triangles == 12
    np.testing.assert_allclose(stats.extent, (0.1, 0.2, 0.3), atol=1e-6)


def test_format_stats_contains_mesh_path(tmp_path: Path):
    mesh_path = tmp_path / "object.ply"
    write_box_mesh(mesh_path)

    text = format_stats(mesh_stats(mesh_path))

    assert f"Mesh: {mesh_path}" in text
    assert "Vertices: 8" in text
