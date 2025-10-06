#!/usr/bin/env python3
import math, struct, numpy as np, cv2, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point

class RGBCloudTargetToMap(Node):
    def __init__(self):
        super().__init__('rgb_cloud_target_to_map')

        # --- Params ---
        self.declare_parameter('rgb_topic',   '/camera/image')              # match your bridge
        self.declare_parameter('cloud_topic', '/camera/depth/points')       # organized cloud
        self.declare_parameter('target_frame','map')                         # or 'odom'
        self.declare_parameter('roi_px', 7)                                  # radius for median over neighborhood
        self.declare_parameter('min_m', 0.1)
        self.declare_parameter('max_m', 60.0)

        # HSV thresholds (tune!)
        # Foliage/green example:
        self.declare_parameter('hsv_low',  [35, 40, 40])
        self.declare_parameter('hsv_high', [90,255,255])
        # Optional second range; set to [-1,-1,-1] to disable
        self.declare_parameter('hsv2_low',  [-1,-1,-1])
        self.declare_parameter('hsv2_high', [-1,-1,-1])

        self.rgb_topic   = self.get_parameter('rgb_topic').get_parameter_value().string_value
        self.cloud_topic = self.get_parameter('cloud_topic').get_parameter_value().string_value
        self.target_frame= self.get_parameter('target_frame').get_parameter_value().string_value
        self.roi_px = int(self.get_parameter('roi_px').value)
        self.min_m  = float(self.get_parameter('min_m').value)
        self.max_m  = float(self.get_parameter('max_m').value)

        self.hsv_low  = np.array(self.get_parameter('hsv_low').value,  np.int32)
        self.hsv_high = np.array(self.get_parameter('hsv_high').value, np.int32)
        self.hsv2_low  = np.array(self.get_parameter('hsv2_low').value,  np.int32)
        self.hsv2_high = np.array(self.get_parameter('hsv2_high').value, np.int32)
        self.use_second = np.all(self.hsv2_low >= 0) and np.all(self.hsv2_high >= 0)

        self.bridge = CvBridge()

        # Sync RGB + Cloud
        self.sub_rgb   = Subscriber(self, Image, self.rgb_topic)
        self.sub_cloud = Subscriber(self, PointCloud2, self.cloud_topic)
        self.sync = ApproximateTimeSynchronizer([self.sub_rgb, self.sub_cloud], queue_size=10, slop=0.08)
        self.sync.registerCallback(self.cb_sync)

        # TF + pubs
        self.tf = Buffer(); self.tfl = TransformListener(self.tf, self)
        self.pub_point  = self.create_publisher(PointStamped, 'tree/point_' + self.target_frame, 10)
        self.pub_marker = self.create_publisher(Marker, 'tree/marker', 10)

        self.get_logger().info(f'RGB:{self.rgb_topic} CLOUD:{self.cloud_topic} → {self.target_frame}')

    def cb_sync(self, rgb_msg: Image, cloud: PointCloud2):
        # Ensure cloud is organized so we can index by (u,v)
        if cloud.height < 2 or cloud.width < 2:
            self.get_logger().warn_once('PointCloud2 not organized (height/width <=1); cannot index by pixel.')
            return

        # Convert RGB to HSV and segment
        rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_low.astype(np.uint8), self.hsv_high.astype(np.uint8))
        if self.use_second:
            mask |= cv2.inRange(hsv, self.hsv2_low.astype(np.uint8), self.hsv2_high.astype(np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5),np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5,5),np.uint8))

        cnts,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return
        cnt = max(cnts, key=cv2.contourArea)
        if cv2.contourArea(cnt) < 80:
            return
        M = cv2.moments(cnt)
        if M['m00'] == 0: return
        u = int(M['m10']/M['m00'])
        v = int(M['m01']/M['m00'])

        # Clamp to image/cloud bounds
        w, h = rgb_msg.width, rgb_msg.height
        if not (0 <= u < w and 0 <= v < h and w == cloud.width and h == cloud.height):
            # If sizes differ (rare), scale u,v to cloud dims
            u = int(u * (cloud.width  / max(1,w)))
            v = int(v * (cloud.height / max(1,h)))

        # Prepare field offsets for fast access
        fields = {f.name: f.offset for f in cloud.fields}
        if not all(k in fields for k in ('x','y','z')):
            self.get_logger().warn_once('PointCloud2 missing x/y/z fields.')
            return

        # Median over a small ROI in the cloud to reduce noise / NaNs
        pts = []
        r = self.roi_px
        x0,x1 = max(0,u-r), min(cloud.width,  u+r+1)
        y0,y1 = max(0,v-r), min(cloud.height, v+r+1)

        for vv in range(y0, y1):
            base_row = vv * cloud.row_step
            for uu in range(x0, x1):
                off = base_row + uu * cloud.point_step
                # unpack 3 float32 at given offsets
                try:
                    X = struct.unpack_from('f', cloud.data, off + fields['x'])[0]
                    Y = struct.unpack_from('f', cloud.data, off + fields['y'])[0]
                    Z = struct.unpack_from('f', cloud.data, off + fields['z'])[0]
                except struct.error:
                    continue
                if np.isfinite(X) and np.isfinite(Y) and np.isfinite(Z) and (self.min_m <= math.sqrt(X*X+Y*Y+Z*Z) <= self.max_m):
                    pts.append((X,Y,Z))

        if not pts:
            return

        # median of ROI
        arr = np.array(pts, dtype=np.float32)
        X, Y, Z = np.median(arr, axis=0).tolist()

        pt_cam = PointStamped()
        pt_cam.header = cloud.header          # cloud frame (camera optical)
        pt_cam.point.x, pt_cam.point.y, pt_cam.point.z = float(X), float(Y), float(Z)

        # Transform to target frame
        try:
            tfm = self.tf.lookup_transform(self.target_frame, pt_cam.header.frame_id, rclpy.time.Time())
            pt_world = do_transform_point(pt_cam, tfm)
        except Exception as e:
            self.get_logger().warn(f"TF error: {e}")
            return

        # Publish
        self.pub_point.publish(pt_world)
        mk = Marker()
        mk.header = pt_world.header
        mk.ns, mk.id, mk.type, mk.action = 'tree', 0, Marker.SPHERE, Marker.ADD
        mk.pose.orientation.w = 1.0
        mk.pose.position = pt_world.point
        mk.scale.x = mk.scale.y = mk.scale.z = 0.2
        mk.color.r, mk.color.g, mk.color.b, mk.color.a = 0.1, 0.9, 0.1, 0.9
        self.pub_marker.publish(mk)

        rng = math.sqrt(X*X + Y*Y + Z*Z)
        self.get_logger().info(f"Tree @ {self.target_frame}: x={pt_world.point.x:.3f}, y={pt_world.point.y:.3f}, z={pt_world.point.z:.3f} "
                               f"(range≈{rng:.3f} m, pixel={u},{v})")

def main():
    rclpy.init(); rclpy.spin(RGBCloudTargetToMap()); rclpy.shutdown()
if __name__ == '__main__':
    main()