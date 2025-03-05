ros2 topic pub /detected_markers visualization_msgs/msg/MarkerArray "markers: [
    {header: {frame_id: 'map'}, id: 0, type: 2, action: 0, pose: {position: {x: 1.0, y: 1.0, z: 0.0}}, scale: {x: 0.2, y: 0.2, z: 0.2}, color: {r: 1.0, g: 0.0, b: 0.0, a: 1.0}},
    {header: {frame_id: 'map'}, id: 1, type: 2, action: 0, pose: {position: {x: -1.0, y: -1.0, z: 0.0}}, scale: {x: 0.2, y: 0.2, z: 0.2}, color: {r: 0.0, g: 1.0, b: 0.0, a: 1.0}},
    {header: {frame_id: 'map'}, id: 2, type: 2, action: 0, pose: {position: {x: 2.0, y: 2.0, z: 0.0}}, scale: {x: 0.2, y: 0.2, z: 0.2}, color: {r: 0.0, g: 0.0, b: 1.0, a: 1.0}}
]" --once
