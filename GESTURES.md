# Gestures: file format and server protocol

_Author: Artur-Fedjukevits_

The **Gestures** tab saves the model's pose to JSON and can publish the file to the robot over rosbridge.
**Body and face are always separate files and separate topics** (on the robot they correspond to the
`BODY_JOINTS` and `FACE_JOINTS` tables; face = jaw, upper lip, cheeks, eyebrows, forehead, eyes, eyelids;
everything else is body).

## Storage in the app

`user://gestures/body/<name>.json` and `user://gestures/face/<name>.json`
(on Android this is the app's private folder; the **JSON** button copies the content to the clipboard).

## File format

```json
{
	"format": "inmoov_gesture",
	"version": 1,
	"kind": "body",                      // "body" | "face"
	"name": "wave",                      // name as typed by the user
	"robot": "inmoov_i2",
	"units": "urdf_rad",                 // positions are URDF radians (as in /urdf_joint_cmd)
	"created": "2026-10-05T12:00:00Z",
	"positions": {                       // only controllable joints of this kind; mimic joints are not written
		"left_elbow_x_joint": 0.5236,
		"right_shoulder_x_joint": 1.2
	},
	"servo_deg": {                       // optional: the same pose in servo degrees (from the urdf_bridge table,
		"bicep_L": 83.0                  // if the app has connected to the robot and received get_map)
	}
}
```

`positions` contains **only the joints the user moved** since the last *Reset* or the last save of that kind;
the server must not touch any other joint. The body has up to 25 joints (torso, neck/head, shoulders, elbows,
wrists, fingers), the face up to 16 (jaw, lip, cheeks, eyebrows, forehead, eyes, eyelids). Saving a body gesture
clears the "touched" memory for body joints only, saving a face gesture clears it for face joints only; *Reset*
clears everything. If nothing was moved, no file is created. Converting URDF radians ↔ servo degrees is the
server's job (`servo_urdf_map.yaml`); `servo_deg` is informational.

## Sending to the server

The **To server** button publishes the **whole file** (a single-line JSON string) as `std_msgs/msg/String`, field `data`:

| Topic | Type | Content |
|---|---|---|
| `/gesture/body_joints` | `std_msgs/msg/String` | gesture file with `"kind": "body"` |
| `/gesture/face_joints` | `std_msgs/msg/String` | gesture file with `"kind": "face"` |

The topic names are the constants `T_GESTURE_BODY` / `T_GESTURE_FACE` in `scripts/robot_link.gd` — change them to
match your server. The server (rosbridge on port 9090) should subscribe to these topics, parse the JSON, and
save/play it as it sees fit (use the `name` field as the file name). The app calls `advertise` for both topics when
it connects and does not wait for a reply: the status "Sent" means "published to rosbridge".
