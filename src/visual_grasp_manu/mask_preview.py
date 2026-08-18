from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from visual_grasp_manu.mask_generation import (
    create_mask_overlay,
    ensure_binary_mask,
    read_rgb_image,
)
from visual_grasp_manu.scan_dataset import MASK_EXTENSIONS, RGB_EXTENSIONS


@dataclass(frozen=True)
class PreviewFrame:
    stem: str
    rgb_path: Path
    mask_path: Path
    overlay_path: Path | None


@dataclass(frozen=True)
class MaskPreviewResult:
    dataset_path: Path
    output_path: Path
    frames_rendered: int


def collect_preview_frames(dataset_path: Path | str) -> list[PreviewFrame]:
    path = Path(dataset_path)
    rgb_files = collect_by_stem(path / "rgb", RGB_EXTENSIONS)
    mask_files = collect_by_stem(path / "masks", MASK_EXTENSIONS)
    overlay_files = collect_by_stem(path / "mask_overlays", RGB_EXTENSIONS)

    frames: list[PreviewFrame] = []
    for stem, rgb_path in sorted(rgb_files.items()):
        mask_path = mask_files.get(stem)
        if mask_path is None:
            continue
        frames.append(
            PreviewFrame(
                stem=stem,
                rgb_path=rgb_path,
                mask_path=mask_path,
                overlay_path=overlay_files.get(stem),
            )
        )
    return frames


def collect_by_stem(directory: Path, extensions: set[str]) -> dict[str, Path]:
    if not directory.is_dir():
        return {}
    return {
        path.stem: path
        for path in sorted(directory.iterdir())
        if path.is_file() and path.suffix.lower() in extensions
    }


def create_mask_contact_sheet(
    dataset_path: Path | str,
    *,
    output_path: Path | str | None = None,
    columns: int = 1,
    max_frames: int = 0,
    tile_width: int = 320,
) -> MaskPreviewResult:
    path = Path(dataset_path)
    frames = collect_preview_frames(path)
    if max_frames > 0:
        frames = frames[:max_frames]
    if not frames:
        raise ValueError(f"No matching rgb/ and masks/ frames found in: {path}")

    output = Path(output_path) if output_path is not None else path / "mask_preview" / "contact_sheet.png"
    output.parent.mkdir(parents=True, exist_ok=True)

    frame_tiles = [render_frame_tile(frame, tile_width=tile_width) for frame in frames]
    sheet = assemble_grid(frame_tiles, columns=max(1, columns))
    if not cv2.imwrite(str(output), sheet):
        raise ValueError(f"Could not write preview image: {output}")

    return MaskPreviewResult(path, output, len(frames))


def render_frame_tile(frame: PreviewFrame, *, tile_width: int) -> np.ndarray:
    rgb = read_rgb_image(frame.rgb_path)
    mask = read_mask(frame.mask_path)
    overlay = read_overlay(frame.overlay_path) if frame.overlay_path is not None else create_mask_overlay(rgb, mask)

    rgb_panel = resize_panel(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), tile_width)
    mask_panel = resize_panel(colorize_mask(mask), tile_width)
    overlay_panel = resize_panel(cv2.cvtColor(overlay, cv2.COLOR_RGB2BGR), tile_width)

    panel_height = max(rgb_panel.shape[0], mask_panel.shape[0], overlay_panel.shape[0])
    panels = [
        add_panel_label(pad_to_height(rgb_panel, panel_height), "RGB"),
        add_panel_label(pad_to_height(mask_panel, panel_height), "Mask"),
        add_panel_label(pad_to_height(overlay_panel, panel_height), "Overlay"),
    ]
    body = cv2.hconcat(panels)
    header = np.full((32, body.shape[1], 3), 24, dtype=np.uint8)
    cv2.putText(
        header,
        f"Frame {frame.stem}",
        (10, 22),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (235, 235, 235),
        1,
        cv2.LINE_AA,
    )
    return cv2.vconcat([header, body])


def read_mask(path: Path) -> np.ndarray:
    if path.suffix.lower() == ".npy":
        return ensure_binary_mask(np.load(path))
    mask = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if mask is None:
        raise ValueError(f"Could not read mask image: {path}")
    return ensure_binary_mask(mask)


def read_overlay(path: Path) -> np.ndarray:
    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"Could not read overlay image: {path}")
    return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)


def colorize_mask(mask: np.ndarray) -> np.ndarray:
    binary = ensure_binary_mask(mask)
    color = np.zeros((*binary.shape, 3), dtype=np.uint8)
    color[:, :, 1] = binary
    return color


def resize_panel(image: np.ndarray, tile_width: int) -> np.ndarray:
    height, width = image.shape[:2]
    if width <= 0 or height <= 0:
        raise ValueError("Cannot resize an empty image")
    tile_height = max(1, int(round(tile_width * height / width)))
    return cv2.resize(image, (tile_width, tile_height), interpolation=cv2.INTER_AREA)


def pad_to_height(image: np.ndarray, target_height: int) -> np.ndarray:
    pad = target_height - image.shape[0]
    if pad <= 0:
        return image
    return cv2.copyMakeBorder(image, 0, pad, 0, 0, cv2.BORDER_CONSTANT, value=(0, 0, 0))


def add_panel_label(image: np.ndarray, label: str) -> np.ndarray:
    labeled = image.copy()
    cv2.rectangle(labeled, (0, 0), (86, 24), (0, 0, 0), thickness=-1)
    cv2.putText(
        labeled,
        label,
        (8, 17),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        (245, 245, 245),
        1,
        cv2.LINE_AA,
    )
    return labeled


def assemble_grid(tiles: list[np.ndarray], *, columns: int) -> np.ndarray:
    max_height = max(tile.shape[0] for tile in tiles)
    max_width = max(tile.shape[1] for tile in tiles)
    padded_tiles = [pad_tile(tile, max_width, max_height) for tile in tiles]

    rows: list[np.ndarray] = []
    blank = np.full((max_height, max_width, 3), 18, dtype=np.uint8)
    for start in range(0, len(padded_tiles), columns):
        row_tiles = padded_tiles[start : start + columns]
        row_tiles.extend(blank.copy() for _ in range(columns - len(row_tiles)))
        rows.append(cv2.hconcat(row_tiles))
    return cv2.vconcat(rows)


def pad_tile(tile: np.ndarray, target_width: int, target_height: int) -> np.ndarray:
    bottom = target_height - tile.shape[0]
    right = target_width - tile.shape[1]
    return cv2.copyMakeBorder(
        tile,
        0,
        max(0, bottom),
        0,
        max(0, right),
        cv2.BORDER_CONSTANT,
        value=(18, 18, 18),
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Create a contact-sheet preview of scan dataset RGB images, masks, and overlays."
    )
    parser.add_argument("dataset_path", type=Path, help="Scan dataset directory.")
    parser.add_argument("--output", type=Path, default=None, help="Output PNG path.")
    parser.add_argument("--columns", type=int, default=1, help="Number of frame tiles per row.")
    parser.add_argument("--max-frames", type=int, default=0, help="Limit frames. 0 means all frames.")
    parser.add_argument("--tile-width", type=int, default=320, help="Width of each RGB/mask/overlay panel.")
    args = parser.parse_args(argv)

    result = create_mask_contact_sheet(
        args.dataset_path,
        output_path=args.output,
        columns=args.columns,
        max_frames=args.max_frames,
        tile_width=args.tile_width,
    )
    print(f"Scan dataset: {result.dataset_path}")
    print(f"Frames rendered: {result.frames_rendered}")
    print(f"Preview: {result.output_path}")
    print("Status: OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
