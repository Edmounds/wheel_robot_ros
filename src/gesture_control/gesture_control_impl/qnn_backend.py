from __future__ import annotations

from pathlib import Path
from typing import List

from .gesture_classifier import HandState, Landmark, classify_landmarks


class QualcommHandBackend:
    """MediaPipe Hand QNN backend adapted from the Qualcomm sample code."""

    def __init__(
        self,
        model_dir: str,
        assets_dir: str,
        detection_model: str,
        landmark_model: str,
        anchors_file: str,
        min_hand_score: float,
    ) -> None:
        self._model_dir = Path(model_dir).expanduser()
        self._assets_dir = Path(assets_dir).expanduser()
        self._detection_model = self._model_dir / detection_model
        self._landmark_model = self._model_dir / landmark_model
        self._anchors_file = self._assets_dir / anchors_file
        self._min_hand_score = min_hand_score

        self._cv2 = None
        self._np = None
        self._torch = None
        self._aidlite = None
        self._resize_pad = None
        self._denormalize_detections = None
        self._post_process = None
        self._anchors = None
        self._detector = None
        self._landmarker = None

    def start(self) -> None:
        self._check_files()
        self._import_runtime()

        self._post_process = _PostMediaPipeHand(self._cv2, self._np, self._torch)
        self._anchors = self._torch.tensor(
            self._np.load(str(self._anchors_file)),
            dtype=self._torch.float32,
            device="cpu",
        )
        self._detector = _QnnModel(
            aidlite=self._aidlite,
            model_path=self._detection_model,
            input_shapes=[[1, 3, 256, 256]],
            output_shapes=[[1, 2944, 18], [1, 2944, 1]],
        )
        self._landmarker = _QnnModel(
            aidlite=self._aidlite,
            model_path=self._landmark_model,
            input_shapes=[[1, 3, 256, 256]],
            output_shapes=[[1], [1], [1, 21, 3]],
        )

    def detect(self, bgr_frame) -> List[HandState]:
        if self._detector is None or self._landmarker is None:
            raise RuntimeError("QualcommHandBackend.start() was not called")

        frame = self._np.ascontiguousarray(bgr_frame[:, :, ::-1])
        resized, _, scale, pad = self._resize_pad(frame)

        model_input = (resized / 255).astype(self._np.float32)
        model_input = self._np.transpose(model_input, (2, 0, 1))
        model_input = model_input[self._np.newaxis, ...]

        detection_outputs = self._detector.invoke(model_input)
        detections = self._post_process._tensors_to_detections(
            self._torch.from_numpy(detection_outputs[0]),
            self._torch.from_numpy(detection_outputs[1]),
            self._anchors,
        )

        filtered_detections = []
        num_coords = 18
        for detection in detections:
            hands = self._post_process._weighted_non_max_suppression(detection)
            hands = (
                self._torch.stack(hands)
                if len(hands) > 0
                else self._torch.zeros((0, num_coords + 1))
            )
            filtered_detections.append(hands)

        if not filtered_detections:
            return []

        hand_detections = self._denormalize_detections(filtered_detections[0], scale, pad)
        if len(hand_detections) == 0:
            return []

        xc, yc, roi_scale, theta = self._post_process.detection2roi(hand_detections)
        roi_images, affine, _ = self._post_process.extract_roi(frame, xc, yc, theta, roi_scale)
        if roi_images.size()[0] == 0:
            return []

        frame_height, frame_width = frame.shape[:2]
        hands: List[HandState] = []
        for roi_index in range(roi_images.size()[0]):
            flags, normalized_landmarks = self._landmarker.invoke_landmarks(
                roi_images[roi_index : roi_index + 1].numpy()
            )
            landmarks = self._post_process.denormalize_landmarks(
                self._torch.from_numpy(normalized_landmarks),
                affine[roi_index : roi_index + 1],
            )

            score = float(flags[0])
            if score < self._min_hand_score:
                continue

            hand_landmarks = [
                Landmark(
                    x=float(point[0]) / float(frame_width),
                    y=float(point[1]) / float(frame_height),
                    z=float(point[2]),
                )
                for point in landmarks[0]
            ]
            wrist = hand_landmarks[0]
            hands.append(
                HandState(
                    gesture=classify_landmarks(hand_landmarks),
                    wrist_x=wrist.x,
                    wrist_y=wrist.y,
                    score=score,
                )
            )

        return hands

    def close(self) -> None:
        for model in (self._detector, self._landmarker):
            if model is not None:
                model.close()

    def _check_files(self) -> None:
        missing = [
            str(path)
            for path in (self._detection_model, self._landmark_model, self._anchors_file)
            if not path.exists()
        ]
        if missing:
            raise RuntimeError("missing Qualcomm hand model assets: " + ", ".join(missing))

    def _import_runtime(self) -> None:
        import importlib
        import sys

        if str(self._assets_dir) not in sys.path:
            sys.path.insert(0, str(self._assets_dir))

        try:
            self._cv2 = importlib.import_module("cv2")
            self._np = importlib.import_module("numpy")
            self._torch = importlib.import_module("torch")
            self._aidlite = importlib.import_module("aidlite")
            blazebase = importlib.import_module("blazebase")
        except ImportError as exc:
            raise RuntimeError(
                "Qualcomm QNN hand backend requires cv2, numpy, torch, aidlite, "
                "and the gesture_control Qualcomm hand assets"
            ) from exc

        self._resize_pad = blazebase.resize_pad
        self._denormalize_detections = blazebase.denormalize_detections


class _QnnModel:
    def __init__(self, aidlite, model_path: Path, input_shapes, output_shapes) -> None:
        self._aidlite = aidlite
        self._output_shapes = output_shapes
        self._model = aidlite.Model.create_instance(str(model_path))
        if self._model is None:
            raise RuntimeError(f"failed to create QNN model: {model_path}")

        config = aidlite.Config.create_instance()
        if config is None:
            raise RuntimeError("failed to create aidlite config")

        config.implement_type = aidlite.ImplementType.TYPE_LOCAL
        config.framework_type = aidlite.FrameworkType.TYPE_QNN
        config.accelerate_type = aidlite.AccelerateType.TYPE_DSP
        config.is_quantify_model = 1

        self._interpreter = aidlite.InterpreterBuilder.build_interpretper_from_model_and_config(
            self._model,
            config,
        )
        if self._interpreter is None:
            raise RuntimeError(f"failed to build QNN interpreter: {model_path}")

        self._model.set_model_properties(
            input_shapes,
            aidlite.DataType.TYPE_FLOAT32,
            output_shapes,
            aidlite.DataType.TYPE_FLOAT32,
        )

        if self._interpreter.init() != 0:
            raise RuntimeError(f"failed to init QNN interpreter: {model_path}")
        if self._interpreter.load_model() != 0:
            raise RuntimeError(f"failed to load QNN model: {model_path}")

    def invoke(self, model_input):
        result = self._interpreter.set_input_tensor(0, model_input.data)
        if result != 0:
            raise RuntimeError("interpreter set_input_tensor() failed")
        result = self._interpreter.invoke()
        if result != 0:
            raise RuntimeError("interpreter invoke() failed")
        return [
            self._interpreter.get_output_tensor(index).reshape(shape).copy()
            for index, shape in enumerate(self._output_shapes)
        ]

    def invoke_landmarks(self, model_input):
        outputs = self.invoke(model_input)
        return outputs[0].reshape(1).copy(), outputs[2].reshape(1, 21, 3).copy()

    def close(self) -> None:
        if self._interpreter is None:
            return
        destroy = getattr(self._interpreter, "destory", None)
        if destroy is None:
            destroy = getattr(self._interpreter, "destroy", None)
        if destroy is not None:
            destroy()


class _PostMediaPipeHand:
    def __init__(self, cv2, np, torch) -> None:
        self._cv2 = cv2
        self._np = np
        self._torch = torch
        self.kp1 = 0
        self.kp2 = 2
        self.theta0 = 1.5707963267948966
        self.dscale = 2.6
        self.dy = -0.5
        self.x_scale = 256.0
        self.y_scale = 256.0
        self.h_scale = 256.0
        self.w_scale = 256.0
        self.num_keypoints = 7
        self.num_classes = 1
        self.num_anchors = 2944
        self.num_coords = 18
        self.min_score_thresh = 0.75
        self.score_clipping_thresh = 100.0
        self.min_suppression_threshold = 0.3
        self.resolution = 256

    def detection2roi(self, detection):
        xc = (detection[:, 1] + detection[:, 3]) / 2
        yc = (detection[:, 0] + detection[:, 2]) / 2
        scale = detection[:, 3] - detection[:, 1]
        yc += self.dy * scale
        scale *= self.dscale
        x0 = detection[:, 4 + 2 * self.kp1]
        y0 = detection[:, 4 + 2 * self.kp1 + 1]
        x1 = detection[:, 4 + 2 * self.kp2]
        y1 = detection[:, 4 + 2 * self.kp2 + 1]
        theta = self._torch.atan2(y0 - y1, x0 - x1) - self.theta0
        return xc, yc, scale, theta

    def _decode_boxes(self, raw_boxes, anchors):
        boxes = self._torch.zeros_like(raw_boxes)
        x_center = raw_boxes[..., 0] / self.x_scale * anchors[:, 2] + anchors[:, 0]
        y_center = raw_boxes[..., 1] / self.y_scale * anchors[:, 3] + anchors[:, 1]
        width = raw_boxes[..., 2] / self.w_scale * anchors[:, 2]
        height = raw_boxes[..., 3] / self.h_scale * anchors[:, 3]

        boxes[..., 0] = y_center - height / 2.0
        boxes[..., 1] = x_center - width / 2.0
        boxes[..., 2] = y_center + height / 2.0
        boxes[..., 3] = x_center + width / 2.0

        for keypoint in range(self.num_keypoints):
            offset = 4 + keypoint * 2
            keypoint_x = raw_boxes[..., offset] / self.x_scale * anchors[:, 2] + anchors[:, 0]
            keypoint_y = (
                raw_boxes[..., offset + 1] / self.y_scale * anchors[:, 3] + anchors[:, 1]
            )
            boxes[..., offset] = keypoint_x
            boxes[..., offset + 1] = keypoint_y
        return boxes

    def _tensors_to_detections(self, raw_box_tensor, raw_score_tensor, anchors):
        detection_boxes = self._decode_boxes(raw_box_tensor, anchors)
        raw_score_tensor = raw_score_tensor.clamp(
            -self.score_clipping_thresh,
            self.score_clipping_thresh,
        )
        detection_scores = raw_score_tensor.sigmoid().squeeze(dim=-1)
        mask = detection_scores >= self.min_score_thresh

        output_detections = []
        for index in range(raw_box_tensor.shape[0]):
            boxes = detection_boxes[index, mask[index]]
            scores = detection_scores[index, mask[index]].unsqueeze(dim=-1)
            output_detections.append(self._torch.cat((boxes, scores), dim=-1))
        return output_detections

    def extract_roi(self, frame, xc, yc, theta, scale):
        points = self._torch.tensor(
            [[-1, -1, 1, 1], [-1, 1, -1, 1]],
            device=scale.device,
        ).view(1, 2, 4)
        points = points * scale.view(-1, 1, 1) / 2
        theta = theta.view(-1, 1, 1)
        rotation = self._torch.cat(
            (
                self._torch.cat((self._torch.cos(theta), -self._torch.sin(theta)), 2),
                self._torch.cat((self._torch.sin(theta), self._torch.cos(theta)), 2),
            ),
            1,
        )
        center = self._torch.cat((xc.view(-1, 1, 1), yc.view(-1, 1, 1)), 1)
        points = rotation @ points + center

        res = self.resolution
        points1 = self._np.array([[0, 0, res - 1], [0, res - 1, 0]], dtype=self._np.float32).T
        affines = []
        images = []
        for index in range(points.shape[0]):
            pts = points[index, :, :3].detach().numpy().T
            matrix = self._cv2.getAffineTransform(pts, points1)
            image = self._cv2.warpAffine(frame, matrix, (res, res))
            images.append(self._torch.tensor(image, device=scale.device))
            affine = self._cv2.invertAffineTransform(matrix).astype("float32")
            affines.append(self._torch.tensor(affine, device=scale.device))

        if images:
            images = self._torch.stack(images).permute(0, 3, 1, 2).float() / 255.0
            affines = self._torch.stack(affines)
        else:
            images = self._torch.zeros((0, 3, res, res), device=scale.device)
            affines = self._torch.zeros((0, 2, 3), device=scale.device)
        return images, affines, points

    def denormalize_landmarks(self, landmarks, affines):
        landmarks[:, :, :2] *= self.resolution
        for index in range(len(landmarks)):
            landmark = landmarks[index]
            affine = affines[index]
            landmarks[index, :, :2] = (affine[:, :2] @ landmark[:, :2].T + affine[:, 2:]).T
        return landmarks

    def intersect(self, box_a, box_b):
        count_a = box_a.size(0)
        count_b = box_b.size(0)
        max_xy = self._torch.min(
            box_a[:, 2:].unsqueeze(1).expand(count_a, count_b, 2),
            box_b[:, 2:].unsqueeze(0).expand(count_a, count_b, 2),
        )
        min_xy = self._torch.max(
            box_a[:, :2].unsqueeze(1).expand(count_a, count_b, 2),
            box_b[:, :2].unsqueeze(0).expand(count_a, count_b, 2),
        )
        intersection = self._torch.clamp((max_xy - min_xy), min=0)
        return intersection[:, :, 0] * intersection[:, :, 1]

    def jaccard(self, box_a, box_b):
        intersection = self.intersect(box_a, box_b)
        area_a = ((box_a[:, 2] - box_a[:, 0]) * (box_a[:, 3] - box_a[:, 1])).unsqueeze(
            1
        ).expand_as(intersection)
        area_b = ((box_b[:, 2] - box_b[:, 0]) * (box_b[:, 3] - box_b[:, 1])).unsqueeze(
            0
        ).expand_as(intersection)
        union = area_a + area_b - intersection
        return intersection / union

    def overlap_similarity(self, box, other_boxes):
        return self.jaccard(box.unsqueeze(0), other_boxes).squeeze(0)

    def _weighted_non_max_suppression(self, detections):
        if len(detections) == 0:
            return []

        output_detections = []
        remaining = self._torch.argsort(detections[:, self.num_coords], descending=True)

        while len(remaining) > 0:
            detection = detections[remaining[0]]
            first_box = detection[:4]
            other_boxes = detections[remaining, :4]
            ious = self.overlap_similarity(first_box, other_boxes)
            mask = ious > self.min_suppression_threshold
            overlapping = remaining[mask]
            remaining = remaining[~mask]

            weighted_detection = detection.clone()
            if len(overlapping) > 1:
                coordinates = detections[overlapping, : self.num_coords]
                scores = detections[overlapping, self.num_coords : self.num_coords + 1]
                total_score = scores.sum()
                weighted = (coordinates * scores).sum(dim=0) / total_score
                weighted_detection[: self.num_coords] = weighted
                weighted_detection[self.num_coords] = total_score / len(overlapping)
            output_detections.append(weighted_detection)

        return output_detections
