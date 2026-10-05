## Minimal rosbridge v2 protocol client (WebSocket + JSON).
## Subscriptions and advertisements are remembered and replayed after reconnecting.
##
##   var ros := RosbridgeClient.new(); add_child(ros)
##   ros.connect_to_url("ws://192.168.10.50:9090")
##   ros.subscribe("/urdf_joint_states", "sensor_msgs/msg/JointState", 40)
##   ros.message_received.connect(func(topic, msg): ...)
##   ros.publish("/urdf_joint_cmd", "inmoov_msgs/msg/JointCommand", {...})
class_name RosbridgeClient
extends Node

signal connected
signal disconnected
signal message_received(topic: String, msg: Dictionary)
signal service_response(id: String, ok: bool, values: Variant)

enum State { CLOSED, CONNECTING, OPEN }

## Reconnect automatically when the connection drops.
@export var auto_reconnect := true
@export var reconnect_interval := 2.0

var state := State.CLOSED
var url := ""

var _ws := WebSocketPeer.new()
var _subs := {}        ## topic -> {type, throttle_rate, queue_length}
var _adverts := {}     ## topic -> type
var _next_id := 0
var _retry_in := 0.0
var _want_open := false


func connect_to_url(u: String) -> void:
	url = u.strip_edges()
	if not url.begins_with("ws://") and not url.begins_with("wss://"):
		url = "ws://" + url
	if url.count(":") < 2:        # no port
		url += ":9090"
	_want_open = true
	_open()


func close() -> void:
	_want_open = false
	if state != State.CLOSED:
		_ws.close(1000, "bye")


func is_open() -> bool:
	return state == State.OPEN


## Connecting or waiting for auto-reconnect (the user did not press “Disconnect”).
func is_trying() -> bool:
	return _want_open and state != State.OPEN


func subscribe(topic: String, type: String, throttle_ms := 0, queue_length := 1) -> void:
	_subs[topic] = {"type": type, "throttle_rate": throttle_ms, "queue_length": queue_length}
	if is_open():
		_send_subscribe(topic)


func unsubscribe(topic: String) -> void:
	_subs.erase(topic)
	if is_open():
		_send({"op": "unsubscribe", "topic": topic})


func advertise(topic: String, type: String) -> void:
	_adverts[topic] = type
	if is_open():
		_send({"op": "advertise", "topic": topic, "type": type})


func publish(topic: String, type: String, msg: Dictionary) -> bool:
	if not _adverts.has(topic):
		advertise(topic, type)
	if not is_open():
		return false
	return _send({"op": "publish", "topic": topic, "msg": msg})


## Returns the request id; the response arrives via the service_response(id, ok, values) signal.
func call_service(service: String, type: String, args := {}) -> String:
	_next_id += 1
	var id := "call_%d" % _next_id
	if is_open():
		_send({"op": "call_service", "id": id, "service": service, "type": type, "args": args})
	else:
		service_response.emit.call_deferred(id, false, "not connected")
	return id


## std_msgs/Header with the current time.
static func header(frame_id := "") -> Dictionary:
	var t := Time.get_unix_time_from_system()
	return {"stamp": {"sec": int(t), "nanosec": int(fmod(t, 1.0) * 1e9)}, "frame_id": frame_id}


# ------------------------------------------------------------------ internal

func _open() -> void:
	_ws = WebSocketPeer.new()
	_ws.inbound_buffer_size = 1 << 20       # get_map ~20 KB, with headroom
	_ws.outbound_buffer_size = 1 << 18
	var err := _ws.connect_to_url(url)
	state = State.CONNECTING if err == OK else State.CLOSED
	if err != OK:
		_retry_in = reconnect_interval


func _send(d: Dictionary) -> bool:
	return _ws.send_text(JSON.stringify(d)) == OK


func _send_subscribe(topic: String) -> void:
	var s: Dictionary = _subs[topic]
	_send({"op": "subscribe", "topic": topic, "type": s.type,
		"throttle_rate": s.throttle_rate, "queue_length": s.queue_length})


func _process(dt: float) -> void:
	if state == State.CLOSED:
		if _want_open and auto_reconnect:
			_retry_in -= dt
			if _retry_in <= 0.0:
				_open()
		return
	_ws.poll()
	match _ws.get_ready_state():
		WebSocketPeer.STATE_OPEN:
			if state != State.OPEN:
				state = State.OPEN
				for t in _adverts:
					_send({"op": "advertise", "topic": t, "type": _adverts[t]})
				for t in _subs:
					_send_subscribe(t)
				connected.emit()
			while _ws.get_available_packet_count() > 0:
				_handle(_ws.get_packet().get_string_from_utf8())
		WebSocketPeer.STATE_CLOSED:
			var was_open := state == State.OPEN
			state = State.CLOSED
			_retry_in = reconnect_interval
			if was_open:
				disconnected.emit()


func _handle(text: String) -> void:
	var d = JSON.parse_string(text)
	if typeof(d) != TYPE_DICTIONARY:
		return
	match d.get("op", ""):
		"publish":
			message_received.emit(d.get("topic", ""), d.get("msg", {}))
		"service_response":
			service_response.emit(str(d.get("id", "")), bool(d.get("result", false)), d.get("values"))
		"status":
			push_warning("rosbridge: %s" % d.get("msg", ""))
