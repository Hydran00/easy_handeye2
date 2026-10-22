#!/usr/bin/env python
import cv2
import numpy as np
import rclpy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation as R_convert
from sensor_msgs.msg import Image


class ChessboardTracker(Node):
    """
    Detects a plain chessboard or a ChArUco board, publishes camera_link -> marker_frame on TF
    and shows a live preview with the detected corners and board axes overlaid.
    Chessboard: origin on the first inner corner, x along the columns, y along the rows.
    ChArUco: OpenCV board frame (origin on the board corner, x along squares_x, y along squares_y).
    z points into the board in both cases.
    """

    def __init__(self):
        super().__init__("chessboard_tracker")

        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_link", "camera_link")
        self.declare_parameter("camera_optical_frame", "camera_color_optical_frame")
        self.declare_parameter("marker_frame", "chessboard")
        self.declare_parameter("board_type", "charuco")  # 'chessboard' or 'charuco'
        # number of INNER corners (squares - 1) along each direction
        self.declare_parameter("chessboard_cols", 9)
        self.declare_parameter("chessboard_rows", 6)
        self.declare_parameter("square_size", 0.030)  # meters
        # ChArUco only: number of SQUARES along each direction, marker side and dictionary
        self.declare_parameter("squares_x", 5)
        self.declare_parameter("squares_y", 5)
        self.declare_parameter("marker_size", 0.022)  # meters
        self.declare_parameter("aruco_dictionary", "DICT_4X4_50")
        # boards generated with OpenCV < 4.6 (or calib.io) and an even number of rows need the legacy layout
        self.declare_parameter("legacy_pattern", False)
        self.declare_parameter("marker_corner_refinement", "NONE")
        # detection runs on a downscaled image for speed, corners are refined at full resolution
        self.declare_parameter("detection_scale", 0.5)
        self.declare_parameter("show_preview", True)
        self.declare_parameter("preview_width", 1280)
        # calibration params
        self.declare_parameter("fx", 600.0)
        self.declare_parameter("fy", 600.0)
        self.declare_parameter("cx", 320.0)
        self.declare_parameter("cy", 240.0)
        self.declare_parameter("k1", 0.0)
        self.declare_parameter("k2", 0.0)
        self.declare_parameter("p1", 0.0)
        self.declare_parameter("p2", 0.0)
        self.declare_parameter("k3", 0.0)
        self.declare_parameter("k4", 0.0)
        self.declare_parameter("k5", 0.0)
        self.declare_parameter("k6", 0.0)

        self.image_topic = self.get_parameter("image_topic").value
        self.camera_link = self.get_parameter("camera_link").value
        self.camera_optical_frame = self.get_parameter("camera_optical_frame").value
        self.marker_frame = self.get_parameter("marker_frame").value
        if (not self.camera_link or not self.camera_optical_frame
                or self.marker_frame in (self.camera_link, self.camera_optical_frame)):
            raise ValueError("Camera frames must be nonempty and distinct from marker_frame")
        self.board_type = self.get_parameter("board_type").value
        self.pattern_size = (
            int(self.get_parameter("chessboard_cols").value),
            int(self.get_parameter("chessboard_rows").value),
        )
        self.square_size = float(self.get_parameter("square_size").value)
        self.detection_scale = float(self.get_parameter("detection_scale").value)
        self.show_preview = bool(self.get_parameter("show_preview").value)
        self.preview_width = int(self.get_parameter("preview_width").value)

        p = lambda name: float(self.get_parameter(name).value)
        self.camera_matrix = np.array(
            [[p("fx"), 0, p("cx")], [0, p("fy"), p("cy")], [0, 0, 1]], dtype=np.float64
        )
        self.dist_coeffs = np.array(
            [[p("k1"), p("k2"), p("p1"), p("p2"), p("k3"),
              p("k4"), p("k5"), p("k6")]], dtype=np.float64
        )

        cols, rows = self.pattern_size
        grid = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
        self.object_points = np.zeros((cols * rows, 3), dtype=np.float64)
        self.object_points[:, :2] = grid * self.square_size

        if self.board_type == "charuco":
            self.setup_charuco()
        elif self.board_type != "chessboard":
            raise ValueError(
                f"Unknown board_type {self.board_type}, use chessboard or charuco"
            )

        self.bridge = CvBridge()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer(node=self)
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            rclpy.qos.qos_profile_sensor_data,
        )

        if self.board_type == "charuco":
            board_info = (
                f"ChArUco {self.squares[0]}x{self.squares[1]} squares, {self.dictionary_name}, "
                f"marker {self.marker_size} m"
            )
        else:
            board_info = f"chessboard {cols}x{rows} inner corners"
        self.get_logger().info(
            f"Tracker on {self.image_topic}: {board_info}, square {self.square_size} m, "
            f"pose in {self.camera_optical_frame}, "
            f"publishing {self.camera_link} -> {self.marker_frame}"
        )

    def setup_charuco(self):
        self.squares = (
            int(self.get_parameter("squares_x").value),
            int(self.get_parameter("squares_y").value),
        )
        self.marker_size = float(self.get_parameter("marker_size").value)
        self.dictionary_name = self.get_parameter("aruco_dictionary").value
        dictionary = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, self.dictionary_name)
        )
        self.board = cv2.aruco.CharucoBoard(
            self.squares, self.square_size, self.marker_size, dictionary
        )
        self.board.setLegacyPattern(bool(self.get_parameter("legacy_pattern").value))
        self.board_marker_ids = set(int(i) for i in self.board.getIds().ravel())
        self.get_logger().info(
            f"Expected board marker IDs: {sorted(self.board_marker_ids)}; "
            f"ChArUco corners: {len(self.board.getChessboardCorners())}"
        )
        charuco_params = cv2.aruco.CharucoParameters()
        charuco_params.cameraMatrix = self.camera_matrix
        charuco_params.distCoeffs = self.dist_coeffs
        detector_params = cv2.aruco.DetectorParameters()
        refinement = self.get_parameter("marker_corner_refinement").value.upper()
        if refinement not in ("NONE", "SUBPIX"):
            raise ValueError("marker_corner_refinement must be NONE or SUBPIX")
        detector_params.cornerRefinementMethod = getattr(
            cv2.aruco, f"CORNER_REFINE_{refinement}"
        )
        self.charuco_detector = cv2.aruco.CharucoDetector(
            self.board, charuco_params, detector_params
        )
        self.get_logger().info(
            f"OpenCV {cv2.__version__}; marker refinement={refinement}; "
            f"legacy_pattern={self.get_parameter('legacy_pattern').value}; "
            f"K={self.camera_matrix.tolist()}; D={self.dist_coeffs.ravel().tolist()}"
        )

    def detect(self, gray):
        scale = self.detection_scale if 0.0 < self.detection_scale < 1.0 else 1.0
        small = (
            cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
            if scale != 1.0
            else gray
        )
        flags = (
            cv2.CALIB_CB_ADAPTIVE_THRESH
            | cv2.CALIB_CB_NORMALIZE_IMAGE
            | cv2.CALIB_CB_FAST_CHECK
        )
        found, corners = cv2.findChessboardCorners(small, self.pattern_size, flags)
        if not found:
            return None
        corners = corners / scale
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)
        corners = cv2.cornerSubPix(
            gray, corners.astype(np.float32), (11, 11), (-1, -1), criteria
        )
        # the corner order of a chessboard is ambiguous up to a 180 deg rotation: make the first
        # corner the one closest to the image top-left so the board frame does not flip between samples
        if corners[0, 0].sum() > corners[-1, 0].sum():
            corners = corners[::-1].copy()
        return corners

    def detect_charuco(self, gray, frame):
        """
        Returns (object_points, image_points) of the detected ChArUco corners, or None.
        Draws the detected markers and corners on frame.
        """
        charuco_corners, charuco_ids, marker_corners, marker_ids = (
            self.charuco_detector.detectBoard(gray)
        )
        self.n_markers = 0 if marker_ids is None else len(marker_ids)
        self.detected_marker_ids = (
            [] if marker_ids is None else [int(i) for i in marker_ids.ravel()]
        )
        self.n_board_markers = (
            0 if marker_ids is None else
            sum(int(i) in self.board_marker_ids for i in marker_ids.ravel())
        )
        self.n_charuco_corners = 0 if charuco_ids is None else len(charuco_ids)
        if self.show_preview and marker_ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, marker_corners, marker_ids)
        if self.show_preview and charuco_ids is not None:
            cv2.aruco.drawDetectedCornersCharuco(
                frame, charuco_corners, charuco_ids, (0, 255, 0)
            )
        if charuco_ids is None or len(charuco_ids) < 4:
            self.detection_reason = "fewer than 4 ChArUco corners"
            return None
        # Collinear corners do not constrain a planar pose.
        if self.board.checkCharucoCornersCollinear(charuco_ids):
            self.detection_reason = "collinear ChArUco corners"
            return None
        self.detection_reason = "pose rejected"
        object_points, image_points = self.board.matchImagePoints(
            charuco_corners, charuco_ids
        )
        return object_points, image_points

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.board_type == "charuco":
            points = self.detect_charuco(gray, frame)
        else:
            corners = self.detect(gray)
            points = None if corners is None else (self.object_points, corners)
            if self.show_preview and corners is not None:
                cv2.drawChessboardCorners(frame, self.pattern_size, corners, True)

        detected = False
        if points is not None:
            success, rvec, tvec = cv2.solvePnP(
                points[0],
                points[1],
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE,
            )
            if (
                success
                and np.isfinite(rvec).all()
                and np.isfinite(tvec).all()
                and tvec[2][0] > 0
            ):
                detected = self.publish_transform(rvec, tvec, msg.header.stamp)
                if self.board_type == "charuco":
                    self.detection_reason = (
                        "pose accepted" if detected else "camera link/optical TF unavailable"
                    )
                if self.show_preview:
                    cv2.drawFrameAxes(
                        frame,
                        self.camera_matrix,
                        self.dist_coeffs,
                        rvec,
                        tvec,
                        2 * self.square_size,
                        3,
                    )

        if self.board_type == "charuco":
            unexpected = sorted(set(self.detected_marker_ids) - self.board_marker_ids)
            self.get_logger().info(
                f"ChArUco: ids={self.detected_marker_ids}, "
                f"board_markers={self.n_board_markers}/{len(self.board_marker_ids)}, "
                f"unexpected_ids={unexpected}, corners={self.n_charuco_corners}/"
                f"{len(self.board.getChessboardCorners())}, {self.detection_reason}; "
                f"image={frame.shape[1]}x{frame.shape[0]}",
                throttle_duration_sec=2.0,
            )

        if self.show_preview:
            self.show(
                frame,
                detected,
                tvec if detected else None,
                len(points[1]) if points is not None else 0,
            )

    def show(self, frame, detected, tvec, n_corners):
        name = "CHARUCO" if self.board_type == "charuco" else "CHESSBOARD"
        if detected:
            text, color = (
                f"{name} DETECTED  corners={n_corners}  z={tvec[2][0]:.3f} m",
                (0, 200, 0),
            )
        else:
            text, color = f"{name} NOT DETECTED", (0, 0, 255)
        if self.board_type == "charuco":
            text += (
                f"  markers={self.n_markers}"
                f"  board_markers={self.n_board_markers}/{len(self.board_marker_ids)}"
            )
            if not detected:
                cv2.putText(
                    frame,
                    f"corners={self.n_charuco_corners}: {self.detection_reason}",
                    (30, 110), cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2, cv2.LINE_AA,
                )
        cv2.putText(
            frame, text, (30, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.5, color, 3, cv2.LINE_AA
        )
        h, w = frame.shape[:2]
        if self.preview_width > 0 and w > self.preview_width:
            frame = cv2.resize(
                frame, (self.preview_width, int(h * self.preview_width / w))
            )
        cv2.imshow("Chessboard tracker", frame)
        cv2.waitKey(1)

    def publish_transform(self, rvec, tvec, stamp):
        # solvePnP returns optical_T_marker. TF returns link_T_optical,
        # including any intermediate camera frames provided by the driver.
        optical_rotation = R_convert.from_rotvec(np.asarray(rvec).reshape(3))
        translation = np.asarray(tvec).reshape(3)
        rotation = optical_rotation
        if self.camera_link != self.camera_optical_frame:
            try:
                link_to_optical = self.tf_buffer.lookup_transform(
                    self.camera_link, self.camera_optical_frame, Time()
                ).transform
            except tf2_ros.TransformException as exc:
                self.get_logger().warning(
                    f"Cannot publish board pose: missing {self.camera_link} <- "
                    f"{self.camera_optical_frame} TF: {exc}",
                    throttle_duration_sec=5.0,
                )
                return False
            q = link_to_optical.rotation
            link_rotation = R_convert.from_quat([q.x, q.y, q.z, q.w])
            p = link_to_optical.translation
            translation = link_rotation.apply(translation) + np.array([p.x, p.y, p.z])
            rotation = link_rotation * optical_rotation

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.camera_link
        transform.child_frame_id = self.marker_frame
        transform.transform.translation = Vector3(
            x=float(translation[0]), y=float(translation[1]), z=float(translation[2])
        )

        q = rotation.as_quat()
        transform.transform.rotation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        self.tf_broadcaster.sendTransform(transform)
        return True


def main(args=None):
    rclpy.init(args=args)
    node = ChessboardTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
