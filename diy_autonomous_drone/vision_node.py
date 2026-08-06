"""Capture camera frames and publish target and gesture observations."""

from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Pose2D
from rclpy.node import Node
from std_msgs.msg import Int32

try:
    import cv2
except ImportError:  # pragma: no cover - depends on the target platform.
    cv2 = None

try:
    from picamera2 import Picamera2
except ImportError:  # pragma: no cover - depends on the target platform.
    Picamera2 = None


class VisionNode(Node):
    """Own the camera and expose normalized vision observations to ROS."""

    GESTURE_NONE = 0

    def __init__(self) -> None:
        """Declare parameters, publishers, camera capture, and frame timer."""
        super().__init__('vision_node')

        self.declare_parameter('camera_index', 0)
        self.declare_parameter('camera_backend', 'auto')
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('fps', 20)

        self._camera_index = int(
            self.get_parameter('camera_index').value)
        self._requested_backend = str(
            self.get_parameter('camera_backend').value).strip().lower()
        self._frame_width = max(
            1, int(self.get_parameter('frame_width').value))
        self._frame_height = max(
            1, int(self.get_parameter('frame_height').value))
        self._fps = max(1, int(self.get_parameter('fps').value))

        self._tracking_publisher = self.create_publisher(
            Pose2D, '/drone/target_tracking_box', 10)
        self._gesture_publisher = self.create_publisher(
            Int32, '/drone/active_gesture', 10)

        self._capture = None
        self._capture_backend = None
        self._capture_failures = 0
        self._open_camera()
        self._frame_timer = self.create_timer(
            1.0 / float(self._fps), self._process_next_frame)

        self.get_logger().info(
            'Vision node started: camera=%d, resolution=%dx%d, fps=%d'
            % (
                self._camera_index,
                self._frame_width,
                self._frame_height,
                self._fps,
            )
        )

    def _open_camera(self) -> None:
        """Open Picamera2 when available, then fall back to OpenCV."""
        if self._requested_backend not in {'auto', 'picamera2', 'opencv'}:
            self.get_logger().error(
                'Unknown camera backend %r.' % self._requested_backend)
            return

        if self._requested_backend in {'auto', 'picamera2'}:
            if self._open_picamera2():
                return
            if self._requested_backend == 'picamera2':
                return

        self._open_opencv()

    def _open_picamera2(self) -> bool:
        """Open the Raspberry Pi libcamera-backed Python interface."""
        if Picamera2 is None:
            if self._requested_backend == 'picamera2':
                self.get_logger().error('Picamera2 is unavailable.')
            return False

        camera = None
        try:
            camera = Picamera2(camera_num=self._camera_index)
            configuration = camera.create_video_configuration(
                main={
                    'size': (self._frame_width, self._frame_height),
                    'format': 'RGB888',
                },
                controls={'FrameRate': float(self._fps)},
                buffer_count=2,
            )
            camera.configure(configuration)
            camera.start()
        except Exception as error:
            if camera is not None:
                try:
                    camera.close()
                except Exception:
                    pass
            self.get_logger().error(
                'Unable to open Picamera2 camera %d: %s'
                % (self._camera_index, error))
            return False

        self._capture = camera
        self._capture_backend = 'picamera2'
        self.get_logger().info('Using the Picamera2 camera backend.')
        return True

    def _open_opencv(self) -> bool:
        """Open and configure an OpenCV-compatible camera device."""
        if cv2 is None:
            self.get_logger().error(
                'OpenCV is unavailable; vision output will remain inactive.')
            return False

        capture = cv2.VideoCapture(self._camera_index)
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self._frame_width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self._frame_height)
        capture.set(cv2.CAP_PROP_FPS, self._fps)

        if not capture.isOpened():
            capture.release()
            self.get_logger().error(
                'Unable to open camera index %d.' % self._camera_index)
            return False

        self._capture = capture
        self._capture_backend = 'opencv'
        self.get_logger().info('Using the OpenCV camera backend.')
        return True

    def _process_next_frame(self) -> None:
        """Read one frame, run inference hooks, and publish observations."""
        if self._capture is None:
            return

        if self._capture_backend == 'picamera2':
            try:
                frame = self._capture.capture_array('main')
                success = frame is not None
            except Exception:
                success, frame = False, None
        else:
            success, frame = self._capture.read()

        if not success:
            self._capture_failures += 1
            if self._capture_failures == 1 or \
                    self._capture_failures % self._fps == 0:
                self.get_logger().warning('Camera frame capture failed.')
            return
        self._capture_failures = 0

        tracking_box, gesture_id = self._infer_observations(frame)
        if tracking_box is not None:
            center_x, center_y, box_height = tracking_box
            tracking_message = Pose2D()
            tracking_message.x = self._clamp(center_x, -1.0, 1.0)
            tracking_message.y = self._clamp(center_y, -1.0, 1.0)
            tracking_message.theta = self._clamp(box_height, 0.0, 1.0)
            self._tracking_publisher.publish(tracking_message)

        gesture_message = Int32()
        gesture_message.data = int(gesture_id)
        self._gesture_publisher.publish(gesture_message)

    def _infer_observations(
        self, frame: object
    ) -> Tuple[Optional[Tuple[float, float, float]], int]:
        """Return normalized tracking data and a gesture identifier.

        The tracking tuple is ``(center_x, center_y, box_height)``. A missing
        target is represented by ``None`` so downstream watchdogs can detect
        target loss. Gesture IDs are 0=None, 1=Unlock, 2=Up, 3=Down, 4=Left,
        and 5=Right.
        """
        del frame
        # TODO: Run YOLO and convert the selected bounding box to normalized
        # coordinates. Keep publishing disabled when no valid target exists.
        # TODO: Run MediaPipe Pose and replace GESTURE_NONE with a classified
        # and suitably debounced gesture ID.
        return None, self.GESTURE_NONE

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        """Clamp a number to an inclusive range."""
        return max(lower, min(upper, float(value)))

    def destroy_node(self) -> bool:
        """Release the camera before destroying the ROS node."""
        if self._capture is not None:
            if self._capture_backend == 'picamera2':
                try:
                    self._capture.stop()
                finally:
                    self._capture.close()
            else:
                self._capture.release()
            self._capture = None
            self._capture_backend = None
        return super().destroy_node()


def main(args=None) -> None:
    """Run the vision node until ROS shuts down."""
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Vision node interrupted by user.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
