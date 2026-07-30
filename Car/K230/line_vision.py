"""基于多条水平 ROI 的快速黑线识别。

算法特点：
1. QVGA 灰度图，避免 RGB 转换开销；
2. Otsu 阈值低频更新，兼顾光照变化与速度；
3. 跟踪后缩小搜索窗口，降低背景误识别；
4. 多点拟合输出横向误差、航向误差和曲率；
5. 无效时立即输出 0，不沿用旧控制量。
"""

import math

import vision_config as cfg

RAD_TO_DEG = 57.295779513


def _clamp(value, minimum, maximum):
    if value < minimum:
        return minimum
    if value > maximum:
        return maximum
    return value


def _limit_step(target, current, max_step):
    delta = target - current
    if delta > max_step:
        return current + max_step
    if delta < -max_step:
        return current - max_step
    return target


class VisionResult:
    def __init__(self):
        self.valid = 0
        self.lost_count = 0
        self.confidence = 0
        self.lateral_error_mm = 0
        self.heading_error_deg = 0
        self.curvature = 0

        # 本地调试字段，不进入 LXS1 数据区。
        self.raw_valid_strips = 0
        self.threshold = cfg.FIXED_THRESHOLD


class LineDetector:
    def __init__(self):
        count = len(cfg.STRIP_CENTER_Y)
        self.count = count
        self.x_points = [0.0] * count
        self.y_points = [0.0] * count
        self.point_valid = [False] * count
        self.last_x = [float(cfg.OPTICAL_CENTER_X)] * count
        self.last_width = [18.0] * count

        self.threshold = float(cfg.FIXED_THRESHOLD)
        self.thresholds = [(0, int(self.threshold))]
        self.frame_index = 0
        self.lost_count = 0
        self.valid_streak = 0
        self.have_filter = False
        self.filtered_lateral = 0.0
        self.filtered_heading = 0.0
        self.filtered_curvature = 0.0
        self.result = VisionResult()

        top = min(cfg.STRIP_CENTER_Y) - cfg.STRIP_HEIGHT // 2
        bottom = max(cfg.STRIP_CENTER_Y) + cfg.STRIP_HEIGHT // 2
        top = max(0, top)
        bottom = min(cfg.IMAGE_HEIGHT, bottom)
        self.histogram_roi = (0, top, cfg.IMAGE_WIDTH, bottom - top)

    def _update_threshold(self, image):
        if not cfg.ADAPTIVE_THRESHOLD:
            self.threshold = float(cfg.FIXED_THRESHOLD)
            self.thresholds = [(0, int(self.threshold))]
            return

        if self.frame_index % cfg.THRESHOLD_UPDATE_FRAMES:
            return

        try:
            histogram = image.get_histogram(roi=self.histogram_roi, bins=64)
            raw = histogram.get_threshold().value() + cfg.THRESHOLD_OFFSET
            raw = _clamp(raw, cfg.THRESHOLD_MIN, cfg.THRESHOLD_MAX)
            alpha = cfg.THRESHOLD_FILTER_ALPHA
            self.threshold += alpha * (raw - self.threshold)
            self.thresholds = [(0, int(self.threshold))]
        except Exception:
            # 部分旧固件不支持 bins 参数，失败时保持上一次阈值。
            pass

    def _search_roi(self, strip_index):
        center_y = cfg.STRIP_CENTER_Y[strip_index]
        y0 = center_y - cfg.STRIP_HEIGHT // 2
        y0 = _clamp(y0, 0, cfg.IMAGE_HEIGHT - cfg.STRIP_HEIGHT)

        if self.valid_streak > 0 or self.lost_count < 3:
            half = cfg.TRACK_SEARCH_HALF_WIDTH
            half += min(self.lost_count, 3) * cfg.LOST_EXPAND_PIXELS
            expected_x = int(self.last_x[strip_index])
            x0 = max(0, expected_x - half)
            x1 = min(cfg.IMAGE_WIDTH, expected_x + half)
            if x1 - x0 < 32:
                x0 = max(0, min(x0, cfg.IMAGE_WIDTH - 32))
                x1 = x0 + 32
        else:
            x0 = 0
            x1 = cfg.IMAGE_WIDTH

        return (x0, int(y0), int(x1 - x0), cfg.STRIP_HEIGHT)

    def _best_blob(self, blobs, strip_index):
        best = None
        best_score = -2147483647
        expected_x = self.last_x[strip_index]
        expected_width = self.last_width[strip_index]
        tracking = self.valid_streak > 0 or self.lost_count < 3

        for blob in blobs:
            width = blob.w()
            height = blob.h()
            if width < cfg.MIN_LINE_WIDTH or width > cfg.MAX_LINE_WIDTH:
                continue
            if height > cfg.MAX_LINE_HEIGHT:
                continue

            score = blob.pixels() * 4
            if tracking:
                score -= abs(blob.cx() - expected_x) * cfg.POSITION_PENALTY
                score -= abs(width - expected_width) * cfg.WIDTH_PENALTY

            if score > best_score:
                best_score = score
                best = blob

        return best

    def _find_points(self, image):
        valid_count = 0
        density_sum = 0.0

        for index in range(self.count):
            roi = self._search_roi(index)
            try:
                blobs = image.find_blobs(
                    self.thresholds,
                    roi=roi,
                    x_stride=cfg.X_STRIDE,
                    y_stride=cfg.Y_STRIDE,
                    area_threshold=cfg.MIN_BLOB_AREA,
                    pixels_threshold=cfg.MIN_BLOB_PIXELS,
                    merge=True,
                    margin=cfg.MERGE_MARGIN,
                )
            except Exception:
                blobs = ()

            blob = self._best_blob(blobs, index)
            if blob is None:
                self.point_valid[index] = False
                continue

            x = float(blob.cx())
            y = float(blob.cy())
            width = float(blob.w())
            self.x_points[index] = x
            self.y_points[index] = y
            self.point_valid[index] = True
            self.last_x[index] = x
            self.last_width[index] = width
            valid_count += 1

            roi_area = max(1, roi[2] * roi[3])
            density_sum += min(1.0, blob.pixels() / (roi_area * 0.12))

            if cfg.DEBUG_DRAW:
                try:
                    image.draw_rectangle(blob.rect(), color=255, thickness=1)
                    image.draw_cross(int(x), int(y), color=255, size=4)
                except Exception:
                    pass

        density_quality = density_sum / valid_count if valid_count else 0.0
        return valid_count, density_quality

    def _geometry(self, valid_count):
        # 对 x = intercept + slope * forward_pixel 做最小二乘拟合。
        sum_s = 0.0
        sum_x = 0.0
        sum_ss = 0.0
        sum_sx = 0.0

        for index in range(self.count):
            if not self.point_valid[index]:
                continue
            s = cfg.IMAGE_HEIGHT - self.y_points[index]
            x = self.x_points[index]
            sum_s += s
            sum_x += x
            sum_ss += s * s
            sum_sx += s * x

        denominator = valid_count * sum_ss - sum_s * sum_s
        if abs(denominator) < 0.001:
            return 0.0, 0.0, 0.0, 999.0

        slope = (valid_count * sum_sx - sum_s * sum_x) / denominator
        intercept = (sum_x - slope * sum_s) / valid_count

        control_s = cfg.IMAGE_HEIGHT - cfg.CONTROL_Y
        line_x = intercept + slope * control_s
        lateral = (line_x - cfg.OPTICAL_CENTER_X) * cfg.LATERAL_MM_PER_PIXEL

        tangent = slope * cfg.LATERAL_MM_PER_PIXEL / cfg.FORWARD_MM_PER_PIXEL
        heading = math.atan(tangent) * RAD_TO_DEG

        residual_sum = 0.0
        first = -1
        second = -1
        penultimate = -1
        last = -1

        for index in range(self.count):
            if not self.point_valid[index]:
                continue
            s = cfg.IMAGE_HEIGHT - self.y_points[index]
            residual = self.x_points[index] - (intercept + slope * s)
            residual_sum += residual * residual
            if first < 0:
                first = index
            elif second < 0:
                second = index
            penultimate = last
            last = index

        rms = math.sqrt(residual_sum / valid_count)
        curvature = 0.0

        # 近端和远端切线方向的变化量 / 前向距离。
        if first >= 0 and second >= 0 and penultimate >= 0 and last >= 0:
            s0 = cfg.IMAGE_HEIGHT - self.y_points[first]
            s1 = cfg.IMAGE_HEIGHT - self.y_points[second]
            s2 = cfg.IMAGE_HEIGHT - self.y_points[penultimate]
            s3 = cfg.IMAGE_HEIGHT - self.y_points[last]

            ds_near = s1 - s0
            ds_far = s3 - s2
            if abs(ds_near) > 0.1 and abs(ds_far) > 0.1:
                slope_near = (self.x_points[second] - self.x_points[first]) / ds_near
                slope_far = (self.x_points[last] - self.x_points[penultimate]) / ds_far
                angle_near = math.atan(
                    slope_near * cfg.LATERAL_MM_PER_PIXEL / cfg.FORWARD_MM_PER_PIXEL
                )
                angle_far = math.atan(
                    slope_far * cfg.LATERAL_MM_PER_PIXEL / cfg.FORWARD_MM_PER_PIXEL
                )
                mid_near = 0.5 * (s0 + s1)
                mid_far = 0.5 * (s2 + s3)
                distance_mm = (mid_far - mid_near) * cfg.FORWARD_MM_PER_PIXEL
                if abs(distance_mm) > 1.0:
                    # 输出单位为 0.001 / m。
                    curvature = (angle_far - angle_near) / (distance_mm / 1000.0)
                    curvature *= 1000.0

        return lateral, heading, curvature, rms

    def _publish_valid(self, confidence, lateral, heading, curvature):
        if not self.have_filter:
            self.filtered_lateral = lateral
            self.filtered_heading = heading
            self.filtered_curvature = curvature
            self.have_filter = True
        else:
            lateral = _limit_step(
                lateral, self.filtered_lateral, cfg.MAX_LATERAL_STEP_MM
            )
            heading = _limit_step(
                heading, self.filtered_heading, cfg.MAX_HEADING_STEP_DEG
            )
            curvature = _limit_step(
                curvature, self.filtered_curvature, cfg.MAX_CURVATURE_STEP
            )
            self.filtered_lateral += cfg.LATERAL_FILTER_ALPHA * (
                lateral - self.filtered_lateral
            )
            self.filtered_heading += cfg.HEADING_FILTER_ALPHA * (
                heading - self.filtered_heading
            )
            self.filtered_curvature += cfg.CURVATURE_FILTER_ALPHA * (
                curvature - self.filtered_curvature
            )

        self.valid_streak += 1
        self.lost_count = 0

        result = self.result
        result.valid = 1 if self.valid_streak >= cfg.VALID_CONFIRM_FRAMES else 0
        result.lost_count = 0
        result.confidence = int(confidence) if result.valid else 0
        if result.valid:
            result.lateral_error_mm = int(round(self.filtered_lateral))
            result.heading_error_deg = int(round(self.filtered_heading))
            result.curvature = int(round(self.filtered_curvature))
        else:
            result.lateral_error_mm = 0
            result.heading_error_deg = 0
            result.curvature = 0

    def _publish_invalid(self):
        self.valid_streak = 0
        self.lost_count = min(255, self.lost_count + 1)

        result = self.result
        result.valid = 0
        result.lost_count = self.lost_count
        result.confidence = 0
        result.lateral_error_mm = 0
        result.heading_error_deg = 0
        result.curvature = 0

    def process(self, image):
        self.frame_index += 1
        self._update_threshold(image)
        valid_count, density_quality = self._find_points(image)

        result = self.result
        result.raw_valid_strips = valid_count
        result.threshold = int(self.threshold)

        if valid_count < cfg.MIN_VALID_STRIPS:
            self._publish_invalid()
            return result

        lateral, heading, curvature, rms = self._geometry(valid_count)
        coverage = valid_count / self.count
        residual_quality = _clamp(1.0 - rms / 34.0, 0.0, 1.0)
        confidence = int(
            coverage * 650.0
            + residual_quality * 250.0
            + density_quality * 100.0
        )
        confidence = int(_clamp(confidence, 0, 1000))

        if confidence < cfg.MIN_CONFIDENCE:
            self._publish_invalid()
        else:
            self._publish_valid(confidence, lateral, heading, curvature)

        return result

    def draw_debug_overlay(
        self,
        image,
        result,
        fps=0.0,
        tx_error=0,
        scale_x=1.0,
        scale_y=1.0,
    ):
        """把 320x240 识别坐标按比例绘制到独立 OSD 图层。"""
        try:
            text_scale = max(1, int(round(min(scale_x, scale_y))))

            # 采样带、相机光轴和识别中心线。
            for center_y in cfg.STRIP_CENTER_Y:
                y0 = int(
                    (center_y - cfg.STRIP_HEIGHT // 2) * scale_y
                )
                image.draw_rectangle(
                    0,
                    y0,
                    int(cfg.IMAGE_WIDTH * scale_x),
                    int(cfg.STRIP_HEIGHT * scale_y),
                    color=(120, 170, 255),
                    thickness=1,
                )

            image.draw_line(
                int(cfg.OPTICAL_CENTER_X * scale_x),
                int(42 * scale_y),
                int(cfg.OPTICAL_CENTER_X * scale_x),
                int((cfg.IMAGE_HEIGHT - 1) * scale_y),
                color=(255, 210, 0),
                thickness=1,
            )

            previous = -1
            for index in range(self.count):
                if not self.point_valid[index]:
                    continue
                x = int(self.x_points[index] * scale_x)
                y = int(self.y_points[index] * scale_y)
                image.draw_cross(
                    x,
                    y,
                    color=(0, 255, 80),
                    size=5 * text_scale,
                    thickness=2,
                )
                if previous >= 0:
                    image.draw_line(
                        int(self.x_points[previous] * scale_x),
                        int(self.y_points[previous] * scale_y),
                        x,
                        y,
                        color=(0, 255, 80),
                        thickness=2,
                    )
                previous = index

            # 黑底白字，避免白色场地使字符不可读。
            image.draw_rectangle(
                0,
                0,
                int(cfg.IMAGE_WIDTH * scale_x),
                int(40 * scale_y),
                color=(0, 0, 0),
                thickness=1,
                fill=True,
            )
            state = "OK" if result.valid else "LOST"
            image.draw_string(
                int(2 * scale_x),
                int(1 * scale_y),
                "%s C:%d T:%d N:%d"
                % (
                    state,
                    result.confidence,
                    result.threshold,
                    result.raw_valid_strips,
                ),
                color=(255, 255, 255),
                scale=text_scale,
            )
            image.draw_string(
                int(2 * scale_x),
                int(13 * scale_y),
                "LAT:%+d H:%+d K:%+d"
                % (
                    result.lateral_error_mm,
                    result.heading_error_deg,
                    result.curvature,
                ),
                color=(255, 255, 255),
                scale=text_scale,
            )
            image.draw_string(
                int(2 * scale_x),
                int(25 * scale_y),
                "FPS:%.1f TXE:%d LOST:%d"
                % (fps, tx_error, result.lost_count),
                color=(255, 255, 255),
                scale=text_scale,
            )
            return True
        except Exception:
            return False
