from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Protocol

import cv2
import numpy as np
import yaml

from visual_grasp_manu.scan_dataset import (
    RGB_EXTENSIONS,
    format_report,
    read_yaml_file,
    validate_scan_dataset,
)


@dataclass(frozen=True)
class MaskPrediction:
    mask: np.ndarray
    box_xyxy: tuple[float, float, float, float] | None
    score: float
    phrase: str


@dataclass(frozen=True)
class MaskGenerationResult:
    dataset_path: Path
    frames_processed: int
    masks_written: int
    annotations_path: Path


class MaskBackend(Protocol):
    name: str

    def predict(self, image_path: Path, prompt: str) -> MaskPrediction:
        ...


class BoxStubMaskBackend:
    name = "box_stub"

    def predict(self, image_path: Path, prompt: str) -> MaskPrediction:
        image = read_rgb_image(image_path)
        height, width = image.shape[:2]
        x0 = int(width * 0.25)
        y0 = int(height * 0.25)
        x1 = int(width * 0.75)
        y1 = int(height * 0.75)
        mask = np.zeros((height, width), dtype=np.uint8)
        mask[y0:y1, x0:x1] = 255
        return MaskPrediction(
            mask=mask,
            box_xyxy=(float(x0), float(y0), float(x1), float(y1)),
            score=1.0,
            phrase=prompt,
        )


class HsvColorMaskBackend:
    name = "hsv_color"

    def __init__(
        self,
        *,
        lower_hsv: tuple[int, int, int] = (85, 45, 25),
        upper_hsv: tuple[int, int, int] = (135, 255, 255),
        min_area: int = 50,
    ) -> None:
        self.lower_hsv = lower_hsv
        self.upper_hsv = upper_hsv
        self.min_area = min_area

    def predict(self, image_path: Path, prompt: str) -> MaskPrediction:
        image = read_rgb_image(image_path)
        hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(
            hsv,
            np.asarray(self.lower_hsv, dtype=np.uint8),
            np.asarray(self.upper_hsv, dtype=np.uint8),
        )
        mask = clean_mask(mask)
        mask = keep_largest_component(mask, min_area=self.min_area)
        box = mask_box_xyxy(mask)
        return MaskPrediction(
            mask=mask,
            box_xyxy=box,
            score=float(np.count_nonzero(mask)),
            phrase=prompt,
        )


class GroundedSam2MaskBackend:
    name = "grounding_dino_sam2"

    def __init__(
        self,
        *,
        grounding_config: Path,
        grounding_checkpoint: Path,
        sam2_config: str,
        sam2_checkpoint: Path,
        device: str,
        grounding_device: str | None,
        box_threshold: float,
        text_threshold: float,
        multimask_output: bool,
    ) -> None:
        try:
            from groundingdino.util.inference import load_image, load_model, predict
        except ImportError as exc:
            raise RuntimeError(
                "Grounding DINO is not installed. Install it in the active "
                "environment or rerun with --backend box_stub for a file-contract smoke test."
            ) from exc

        try:
            from sam2.build_sam import build_sam2
            from sam2.sam2_image_predictor import SAM2ImagePredictor
        except ImportError as exc:
            raise RuntimeError(
                "SAM2 is not installed. Install it in the active environment or "
                "rerun with --backend box_stub for a file-contract smoke test."
            ) from exc

        self.load_image = load_image
        self.predict_boxes = predict
        self.box_threshold = box_threshold
        self.text_threshold = text_threshold
        self.multimask_output = multimask_output
        self.grounding_device = grounding_device or device

        self.grounding_model = load_model(
            str(grounding_config),
            str(grounding_checkpoint),
            device=self.grounding_device,
        )
        sam2_model = build_sam2(
            sam2_config,
            str(sam2_checkpoint),
            device=device,
        )
        self.sam2_predictor = SAM2ImagePredictor(sam2_model)

    def predict(self, image_path: Path, prompt: str) -> MaskPrediction:
        image_source, image_tensor = self.load_image(str(image_path))
        boxes, logits, phrases = self.predict_boxes(
            model=self.grounding_model,
            image=image_tensor,
            caption=prompt,
            box_threshold=self.box_threshold,
            text_threshold=self.text_threshold,
            device=self.grounding_device,
        )
        if len(boxes) == 0:
            return empty_prediction(image_source, prompt)

        best_index = int(np.asarray(logits.detach().cpu()).argmax())
        best_box = np.asarray(boxes[best_index].detach().cpu(), dtype=np.float32)
        score = float(np.asarray(logits[best_index].detach().cpu()))
        phrase = str(phrases[best_index]) if phrases else prompt
        box_xyxy = normalized_cxcywh_to_xyxy(best_box, image_source.shape[1], image_source.shape[0])

        self.sam2_predictor.set_image(image_source)
        masks, mask_scores, _ = self.sam2_predictor.predict(
            box=np.asarray(box_xyxy, dtype=np.float32),
            multimask_output=self.multimask_output,
        )
        mask_index = int(np.asarray(mask_scores).argmax()) if len(mask_scores) else 0
        mask = np.asarray(masks[mask_index], dtype=np.uint8) * 255
        return MaskPrediction(
            mask=mask,
            box_xyxy=tuple(float(value) for value in box_xyxy),
            score=score,
            phrase=phrase,
        )


def empty_prediction(image_source: np.ndarray, prompt: str) -> MaskPrediction:
    height, width = image_source.shape[:2]
    return MaskPrediction(
        mask=np.zeros((height, width), dtype=np.uint8),
        box_xyxy=None,
        score=0.0,
        phrase=prompt,
    )


def normalized_cxcywh_to_xyxy(
    box: np.ndarray,
    width: int,
    height: int,
) -> tuple[float, float, float, float]:
    cx, cy, box_width, box_height = [float(value) for value in box]
    x0 = (cx - box_width / 2.0) * width
    y0 = (cy - box_height / 2.0) * height
    x1 = (cx + box_width / 2.0) * width
    y1 = (cy + box_height / 2.0) * height
    return (
        clamp(x0, 0.0, float(width - 1)),
        clamp(y0, 0.0, float(height - 1)),
        clamp(x1, 0.0, float(width - 1)),
        clamp(y1, 0.0, float(height - 1)),
    )


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def generate_masks(
    dataset_path: Path | str,
    *,
    backend: MaskBackend,
    prompt: str | None = None,
    limit: int = 0,
    overwrite: bool = False,
    save_overlays: bool = False,
) -> MaskGenerationResult:
    path = Path(dataset_path)
    capture_report = validate_scan_dataset(path, stage="capture")
    if not capture_report.ok:
        raise ValueError(format_report(capture_report))

    metadata_errors: list[str] = []
    metadata = read_yaml_file(path / "metadata.yaml", metadata_errors, required=True)
    if metadata_errors:
        raise ValueError("\n".join(metadata_errors))

    object_prompt = prompt or str(metadata.get("object_prompt", "")).strip()
    if not object_prompt:
        raise ValueError("A text prompt is required. Set metadata.yaml object_prompt or pass --prompt.")

    all_rgb_frames = collect_rgb_frames(path / "rgb")
    rgb_frames = all_rgb_frames
    if limit > 0:
        rgb_frames = rgb_frames[:limit]

    masks_dir = path / "masks"
    masks_dir.mkdir(parents=True, exist_ok=True)
    annotations_dir = path / "mask_annotations"
    annotations_dir.mkdir(parents=True, exist_ok=True)
    overlays_dir = path / "mask_overlays"
    if save_overlays:
        overlays_dir.mkdir(parents=True, exist_ok=True)

    frame_annotations: list[dict[str, Any]] = []
    masks_written = 0
    for rgb_path in rgb_frames:
        mask_path = masks_dir / f"{rgb_path.stem}.png"
        if mask_path.exists() and not overwrite:
            frame_annotations.append(existing_annotation(rgb_path, mask_path, backend.name))
            continue

        prediction = backend.predict(rgb_path, object_prompt)
        mask = ensure_binary_mask(prediction.mask)
        cv2.imwrite(str(mask_path), mask)
        masks_written += 1

        annotation = {
            "frame": rgb_path.stem,
            "rgb_path": str(rgb_path.relative_to(path)),
            "mask_path": str(mask_path.relative_to(path)),
            "prompt": object_prompt,
            "backend": backend.name,
            "score": prediction.score,
            "phrase": prediction.phrase,
            "box_xyxy": list(prediction.box_xyxy) if prediction.box_xyxy is not None else None,
        }
        write_json(annotations_dir / f"{rgb_path.stem}.json", annotation)
        frame_annotations.append(annotation)

        if save_overlays:
            overlay = create_mask_overlay(read_rgb_image(rgb_path), mask)
            cv2.imwrite(
                str(overlays_dir / f"{rgb_path.stem}.png"),
                cv2.cvtColor(overlay, cv2.COLOR_RGB2BGR),
            )

    annotations = {
        "dataset_id": path.name,
        "prompt": object_prompt,
        "backend": backend.name,
        "frames_processed": len(rgb_frames),
        "total_rgb_frames": len(all_rgb_frames),
        "partial_run": len(rgb_frames) != len(all_rgb_frames),
        "masks_written": masks_written,
        "frames": frame_annotations,
    }
    annotations_path = annotations_dir / "summary.json"
    write_json(annotations_path, annotations)
    write_mask_metadata(path, annotations)

    if len(rgb_frames) == len(all_rgb_frames):
        masks_report = validate_scan_dataset(path, stage="masks")
        if not masks_report.ok:
            raise ValueError(format_report(masks_report))

    return MaskGenerationResult(
        dataset_path=path,
        frames_processed=len(rgb_frames),
        masks_written=masks_written,
        annotations_path=annotations_path,
    )


def collect_rgb_frames(rgb_dir: Path) -> list[Path]:
    return [
        path
        for path in sorted(rgb_dir.iterdir())
        if path.is_file() and path.suffix.lower() in RGB_EXTENSIONS
    ]


def existing_annotation(rgb_path: Path, mask_path: Path, backend_name: str) -> dict[str, Any]:
    return {
        "frame": rgb_path.stem,
        "rgb_path": str(Path("rgb") / rgb_path.name),
        "mask_path": str(Path("masks") / mask_path.name),
        "prompt": None,
        "backend": backend_name,
        "score": None,
        "phrase": None,
        "box_xyxy": None,
        "skipped_existing": True,
    }


def ensure_binary_mask(mask: np.ndarray) -> np.ndarray:
    if mask.ndim == 3:
        mask = mask[:, :, 0]
    return np.where(mask > 0, 255, 0).astype(np.uint8)


def clean_mask(mask: np.ndarray) -> np.ndarray:
    binary = ensure_binary_mask(mask)
    kernel = np.ones((5, 5), dtype=np.uint8)
    opened = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
    return cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel)


def keep_largest_component(mask: np.ndarray, *, min_area: int) -> np.ndarray:
    binary = ensure_binary_mask(mask)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(binary, connectivity=8)
    if count <= 1:
        return binary
    areas = stats[1:, cv2.CC_STAT_AREA]
    largest_index = int(np.argmax(areas)) + 1
    if int(stats[largest_index, cv2.CC_STAT_AREA]) < min_area:
        return np.zeros_like(binary)
    return np.where(labels == largest_index, 255, 0).astype(np.uint8)


def mask_box_xyxy(mask: np.ndarray) -> tuple[float, float, float, float] | None:
    ys, xs = np.nonzero(ensure_binary_mask(mask))
    if len(xs) == 0 or len(ys) == 0:
        return None
    return (float(xs.min()), float(ys.min()), float(xs.max()), float(ys.max()))


def create_mask_overlay(image: np.ndarray, mask: np.ndarray) -> np.ndarray:
    overlay = image.copy()
    green = np.zeros_like(overlay)
    green[:, :, 1] = 255
    alpha = (ensure_binary_mask(mask) > 0)[:, :, None].astype(np.float32) * 0.45
    return (overlay * (1.0 - alpha) + green * alpha).astype(np.uint8)


def read_rgb_image(path: Path) -> np.ndarray:
    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"Could not read RGB image: {path}")
    return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)


def write_json(path: Path, data: dict[str, Any]) -> None:
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def write_mask_metadata(dataset_path: Path, annotations: dict[str, Any]) -> None:
    mask_metadata = {
        "backend": annotations["backend"],
        "prompt": annotations["prompt"],
        "frames_processed": annotations["frames_processed"],
        "total_rgb_frames": annotations["total_rgb_frames"],
        "partial_run": annotations["partial_run"],
        "masks_written": annotations["masks_written"],
        "annotation_summary": "mask_annotations/summary.json",
    }
    (dataset_path / "masks_metadata.yaml").write_text(
        yaml.safe_dump(mask_metadata, sort_keys=False),
        encoding="utf-8",
    )


def build_backend(args: argparse.Namespace) -> MaskBackend:
    if args.backend == "box_stub":
        return BoxStubMaskBackend()
    if args.backend == "hsv_color":
        return HsvColorMaskBackend(
            lower_hsv=parse_hsv_triplet(args.hsv_lower),
            upper_hsv=parse_hsv_triplet(args.hsv_upper),
            min_area=args.hsv_min_area,
        )

    required_paths = {
        "--grounding-config": args.grounding_config,
        "--grounding-checkpoint": args.grounding_checkpoint,
        "--sam2-checkpoint": args.sam2_checkpoint,
    }
    missing = [name for name, value in required_paths.items() if value is None]
    if missing:
        raise ValueError(
            "The grounded SAM2 backend requires: " + ", ".join(missing)
        )
    return GroundedSam2MaskBackend(
        grounding_config=args.grounding_config,
        grounding_checkpoint=args.grounding_checkpoint,
        sam2_config=args.sam2_config,
        sam2_checkpoint=args.sam2_checkpoint,
        device=args.device,
        grounding_device=args.grounding_device,
        box_threshold=args.box_threshold,
        text_threshold=args.text_threshold,
        multimask_output=args.multimask_output,
    )


def parse_hsv_triplet(value: str) -> tuple[int, int, int]:
    parts = [part.strip() for part in value.split(",")]
    if len(parts) != 3:
        raise ValueError(f"HSV triplet must have three comma-separated integers: {value}")
    parsed = tuple(int(part) for part in parts)
    if any(part < 0 or part > 255 for part in parsed):
        raise ValueError(f"HSV values must be in [0, 255]: {value}")
    return parsed


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Generate object masks for a scan dataset using Grounding DINO + SAM2."
    )
    parser.add_argument("dataset_path", type=Path, help="Scan dataset directory.")
    parser.add_argument("--prompt", default=None, help="Text prompt. Defaults to metadata.yaml object_prompt.")
    parser.add_argument(
        "--backend",
        choices=["grounding_dino_sam2", "box_stub", "hsv_color"],
        default="grounding_dino_sam2",
        help=(
            "Mask backend. box_stub is for file-contract smoke tests only; "
            "hsv_color is a color-threshold local test backend."
        ),
    )
    parser.add_argument("--device", default="cuda", help="Device for SAM2 and default Grounding DINO inference.")
    parser.add_argument(
        "--grounding-device",
        default=None,
        help="Optional separate device for Grounding DINO, e.g. cpu on 6 GB VRAM systems.",
    )
    parser.add_argument("--grounding-config", type=Path, default=None)
    parser.add_argument("--grounding-checkpoint", type=Path, default=None)
    parser.add_argument(
        "--sam2-config",
        default="configs/sam2.1/sam2.1_hiera_t.yaml",
        help="SAM2 config path or package config name used by build_sam2.",
    )
    parser.add_argument("--sam2-checkpoint", type=Path, default=None)
    parser.add_argument("--box-threshold", type=float, default=0.30)
    parser.add_argument("--text-threshold", type=float, default=0.25)
    parser.add_argument("--multimask-output", action="store_true")
    parser.add_argument("--hsv-lower", default="85,45,25", help="Lower HSV threshold for hsv_color.")
    parser.add_argument("--hsv-upper", default="135,255,255", help="Upper HSV threshold for hsv_color.")
    parser.add_argument("--hsv-min-area", type=int, default=50, help="Minimum component area for hsv_color.")
    parser.add_argument("--limit", type=int, default=0, help="Limit frames for smoke tests. 0 means all frames.")
    parser.add_argument("--overwrite", action="store_true", help="Overwrite existing masks.")
    parser.add_argument("--save-overlays", action="store_true", help="Write mask_overlays/*.png for review.")
    args = parser.parse_args(argv)

    backend = build_backend(args)
    result = generate_masks(
        args.dataset_path,
        backend=backend,
        prompt=args.prompt,
        limit=args.limit,
        overwrite=args.overwrite,
        save_overlays=args.save_overlays,
    )
    print(f"Scan dataset: {result.dataset_path}")
    print(f"Frames processed: {result.frames_processed}")
    print(f"Masks written: {result.masks_written}")
    print(f"Annotations: {result.annotations_path}")
    print("Status: OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
