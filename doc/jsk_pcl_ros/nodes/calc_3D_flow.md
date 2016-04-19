# Calc3DFlow


Calculate 3D Optical Flow from image and pointcloud

## Subscribing Topic
* `~input` (`sensor_msgs/PointCloud2`)

  Input pointcloud. It needs to be organized.

* `~input/image` (`sensor_msgs/Image`)

  Input image.

## Publishing Topic
* `~output/flow` (`jsk_recognition_msgs/Flow3DArrayStamped`)

  Output 3D Optical Flow.

* `~output/image` (`sensor_msgs/Image`)

  Output image of flow added on input image.

* `~output/visualized_flow` (`visualization_msgs/Marker`)

  Output flow for Rviz. This topic is exaggerated to make it easy to see and published only when `publish_marker` is `true`.

## Advertising Service
* `~initialize` (`std_srvs/Empty`)

  Initialize corners to calculate flow. This Service is advertised only when `tracking_mode` is `true`.

## Parameters
* `tracking_mode` (Boolean, default: `true`)

  When this parameter is set `true`, flow is calculated for corners found in initial frame. When `false`, corners to calculate flow are replaced in every frame.

* `approximate_sync` (Boolean, default: `true`)

  When this parameter is set `true`, `~input` and `~input/image` are synchronized with approximate time policy.

* `publish_marker` (Boolean, default: `true`)

  Toggle marker publishing.

* `maxCorners` (Integer, default: `100`)
  
  Maximum number of corners to calculate flow.

* `qualityLevel` (Double, default: `0.05`)

* `minDistance` (Double, default: `5.0`)

* `blockSize` (Integer, default: `3`)

* `subPixWinSize` (Integer, default: `15`)

* `winSize` (Integer, default: `20`)

* `maxLevel` (Integer, default: `5`)

