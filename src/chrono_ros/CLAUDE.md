# Chrono::ROS Rewrite — Schema-Driven Generic Bridge

> Design memory and working agreement for the chrono_ros rewrite.
> Scope: `src/chrono_ros`, its SWIG module (`src/chrono_swig/interface/ros/ChModuleROS.i`),
> and the ROS demos (`src/demos/ros`, `src/demos/python/ros`).
> Maintainer: Patrick Chen. Last updated: 2026-06-11.
>
> **Read this file before touching anything in this package.** It is the durable record of
> requirements, decisions, and plan — it must survive context loss and the deletion of the
> current code (which is expected; see D5).

---

## 1. Problem statement

Chrono::ROS bridges a Chrono simulation with a ROS 2 graph. ROS 2's dependency tree causes
ABI-level symbol collisions with other Chrono dependencies (notably VSG visualization) —
deep, not namespace-isolatable. The accepted solution (already implemented) is full process
isolation: the ROS node runs in a subprocess (`chrono_ros_node`), connected to the sim
process via a shared-memory IPC channel.

**The cost of that isolation, today:** every data pathway (topic) is a hand-written,
compiled contract. Adding one new pathway requires editing ~6 places *inside chrono_ros
itself* (a `MessageType` enum value, a `_ipc.h` POD struct, handler `.h/.cpp`, a `_ros.cpp`
for the subprocess, two CMake targets) and rebuilding Chrono. The old upstream design let a
user subclass `ChROSHandler` in *their own application* and be done; that user experience
was lost. Worse: in IPC mode `ChROSInterface::GetNode()` returns `nullptr`, so the existing
`demo_ROS_custom_handler.cpp` is broken. And Python users have it worst — there is **no way
to author a custom pathway from PyChrono at all**; every handler must be written in C++
inside chrono_ros and individually SWIG-wrapped.

**Goal of the rewrite:** keep absolute process isolation, but make the bridge
**schema-driven at runtime** so that *no user — C++ or Python — ever writes bridge code,
recompiles chrono_ros, or touches the subprocess* to add a new pathway of any installed
ROS message type, including custom message packages.

## 2. Hard requirements (from maintainer, 2026-06-11)

- **R1 — Professional quality.** This is a public-facing open-source project
  (Project Chrono). No jank: clean API surface, documented wire protocol, fail-fast
  diagnostics, tests, migration of docs. Design it right; there is no schedule pressure.
- **R2 — Python is the key driver.** The API must be designed *for* SWIG/PyChrono from day
  one (see §7). A PyChrono user must be able to create publishers/subscribers for arbitrary
  message types with numpy-friendly payloads, purely from Python.
- **R3 — No backward compatibility required.** All current code/infrastructure in this
  package may be voided. **Exception:** every current built-in handler (full list in §9)
  must be re-delivered on the new architecture as the final step — they double as the
  reference examples for users.
- **R4 — Isolation is non-negotiable.** Zero ROS symbols in the simulation process, ever.
- **R5 — Big sensors are first-class.** Camera (up to 4K RGBA) and lidar (PointCloud2 +
  LaserScan) must work with no extra data copies vs. the current implementation (§8).
- **R6 — Built-in handlers are also the documentation.** They must be exemplary uses of the
  public API, not privileged internal code paths.

## 3. Architecture decision (D1): schema-driven generic bridge

**The wire payload becomes the ROS serialized message itself (CDR), not bespoke POD structs.**

Two facts make this possible:

1. **The subprocess needs no compiled message types.** `rclcpp` has
   `create_generic_publisher(topic, type_string, qos)` /
   `create_generic_subscription(...)` (stable since Galactic; the foundation of rosbag2).
   They publish/deliver raw `rclcpp::SerializedMessage` (CDR bytes) for **any message type
   whose package is installed**, dlopening the typesupport at runtime.
2. **The subprocess can describe any type at runtime.**
   `rosidl_typesupport_introspection_cpp` exposes complete field metadata (names, types,
   nesting, array kinds) for any installed type, also via dlopen. It also provides
   init/fini functions, so the subprocess can generically construct a message instance and
   round-trip it through `rclcpp::SerializationBase` — used for our validation handshake.

Therefore:

- The **subprocess** (`chrono_ros_node`) becomes a fixed, message-agnostic program
  (~500 lines) that is **never modified again**, even for user-defined message packages:
  it advertises generic publishers/subscriptions on request and shovels CDR bytes.
- The **sim process** holds a *runtime schema* per type (obtained from the subprocess at
  init — §6) and serializes/deserializes CDR itself with a small, dependency-free
  CDR codec (~300 lines; CDR is little-endian + alignment + length-prefixed sequences).
- The **user** addresses message fields by name through a dict-like `ChROSMessage` API,
  from C++ or Python. New pathway = one callback in *their own code*. Custom message
  packages work by being built/sourced in the environment — nothing else.

### Rejected alternatives (do not revisit without new facts)

- **FMI / FMU co-simulation** (was suggested externally): FMI standardizes a co-sim
  *master* interface — flat, statically declared variable lists and lock-step `doStep()`.
  Every new pathway would mean regenerating `modelDescription.xml` *and* maintaining a
  ROS-side FMU-importer mapping — it relocates the "write bridge code per pathway" problem
  instead of removing it, and maps terribly onto image/pointcloud streams. FMI remains the
  right tool for plugging Chrono dynamics into Simulink-style co-sim masters; that is
  `chrono_fmi`'s job, not this package's.
- **Direct DDS from the sim process** (no subprocess; publish RTPS as `rt/<topic>`):
  elegant, no IPC, but couples Chrono to the user's rmw vendor (breaks under `rmw_zenoh`,
  cyclonedds quirks), loses graph services/parameters, and reintroduces a (smaller)
  dependency surface into the sim process. Rejected.
- **Public dlopen plugin API for the subprocess** ("interim" idea, D3): rejected as public
  API after re-evaluation. Historical context: in old upstream chrono_ros the bridge and
  the user's rclcpp code shared one process, so "user logic inside the bridge" existed
  only as an accident of architecture — a custom handler was simply the only convenient
  place to touch rclcpp. The IPC split moved that code into `chrono_ros_node` (forcing
  rebuilds); the plugin idea would have made that injection dynamic (dlopen user `.so`s
  into the subprocess). Dropped because: users' ROS control logic belongs in their *own*
  ROS nodes — the bridge's only job is data movement, and the generic path covers 100% of
  existing built-ins (all are pure topics, §9). A plugin API would contradict the "never
  write bridge code" promise, expose an unstable ABI (rclcpp + internal headers), double
  the docs/maintenance surface, and be C++-only — directly against R2 (Python-first).
  Niche residual benefits (saving one localhost DDS hop for huge payloads, e.g. in-bridge
  `image_transport` compression; hosting generic service *servers* on older distros) are
  to be added as **built-in bridge capabilities behind options**, never as user plugins.
  Exotic/large sensors are built on the standard generic path — if the generic path can't
  express a sensor, that is a bug in the generic path.

## 4. Target architecture

```
┌──────────────── sim process (ZERO ROS symbols) ───────────────┐
│ ChROSManager                                                  │
│  ├─ ChROSHandler(s)  — rate-scheduled user/built-in callbacks │
│  │    └─ use ChROSPublisher / ChROSSubscription handles       │
│  ├─ ChROSBridge      — advertise/publish/receive, schema cache│
│  │    ├─ ChROSMessageSchema / ChROSMessage / ChROSMessageView │
│  │    └─ CDR codec (in-house, no deps)                        │
│  └─ ipc: SubprocessManager + IPCChannel (shm ring buffer)     │
└───────────────────────────┬───────────────────────────────────┘
                            │  control + data frames (§5)
┌───────────────────────────┴───────────────────────────────────┐
│ chrono_ros_node (subprocess; ZERO Chrono physics symbols)     │
│  fixed generic program:                                       │
│   ├─ DESCRIBE/VALIDATE → introspection typesupport (dlopen)   │
│   ├─ ADVERTISE → create_generic_publisher / _subscription     │
│   ├─ PUBLISH   → GenericPublisher::publish(SerializedMessage) │
│   └─ generic subscription callback → RECEIVED frame to sim    │
└────────────────────────────────────────────────────────────────┘
```

Key consequences (decisions D2, D7):

- **D2:** Isolation becomes **mirrored and total**. The `Chrono_ros` *library* drops its
  ROS link entirely (today it links `${ROS2_TARGETS}` PUBLIC — every consumer, including
  the SWIG module, pulls rclcpp; the unfinished half of the isolation story). Conversely,
  `chrono_ros_node` drops its `Chrono_core` link (today it has one — the other unfinished
  half). The IPC/protocol layer becomes a tiny internal static library
  (`chrono_ros_ipc`: ring buffer, channel, frame structs, schema blob defs) with **zero
  dependencies on either world**; the sim side links it + Chrono, the node links it +
  rclcpp. Both ends are always built from the same source tree, so the protocol/schema
  blob format never crosses a version boundary (this is why it can stay a simple packed
  format). Direct (in-process rclcpp) mode and the old `ChROSInterface` are deleted.
  Single mode: IPC.
- **D7:** The shm ring-buffer IPC layer (`ipc/ChROSSharedMemory`, `ChROSRingBuffer`,
  `ChROSIPCChannel`, `ChROSSubprocessManager`) is kept (it is solid; the ring is already a
  variable-size byte stream). Changes: (a) delete the fixed 64 MB `ipc::Message` heap
  buffers and the `MAX_PAYLOAD_SIZE` cap — serialize directly into the ring; (b) replace
  the hard-coded 512 MB-per-direction channel default with a configurable capacity
  (`ChROSManager::SetChannelCapacity`), default **256 MB total, split ~7:1
  sim→ROS : ROS→sim** (a 4K RGBA frame is ~32 MiB, so the default holds several frames in
  flight; the return direction carries only small command messages); (c) a publish whose
  frame exceeds capacity fails at `Initialize`/first-publish with an actionable error
  naming the knob — a user with a huge custom topic raises one number, nothing else;
  (d) docs: Docker `/dev/shm` must exceed the configured capacity + slack (replaces the
  blanket "reserve ≥ 1 GB" guidance — multi-GB reservations are no longer needed).
- All user callbacks (publisher fill, subscriber receive) fire synchronously inside
  `ChROSManager::Update()` on the sim thread → deterministic in sim time and trivially
  GIL-safe for Python directors (I5).

### 4.1 Time synchronization & stepping model

Chrono is a fixed-step integrator driven by the user's loop (`DoStepDynamics(step)`);
every module follows the "call `Update(time)` from the loop" convention. Chrono::Sensor
already models sensors ticking at sim-time rates with asynchronously filled buffers;
SynChrono does distributed lockstep at heartbeat intervals. The bridge follows the same
philosophy as old upstream chrono_ros and the Gazebo/ROS ecosystem convention:
**simulation time is the master clock; pub/sub is asynchronous; the sim never blocks on
ROS by default.**

- **Outbound:** `ChROSManager::Update(time, step)` ticks each handler at its sim-time rate
  (`update_rate` is relative to sim time, not wall time); frames carry `sim_time_ns`;
  message `header.stamp` fields are sim time. The Clock handler publishes `/clock` so ROS
  nodes run with `use_sim_time:=true` and follow the sim whether it runs faster or slower
  than real time. Wall-clock throttling stays the application's job
  (`ChRealtimeStepTimer`), never the bridge's.
- **Inbound:** the subprocess forwards received ROS messages immediately; the sim side
  drains them at exactly one deterministic point — inside `Update()`, before handlers
  tick. Commands therefore take effect at step boundaries, on the sim thread (I5).
- **Determinism caveat (documented honestly):** closed-loop runs with an external ROS
  controller are *not* bit-reproducible in this default mode — command arrival relative to
  step boundaries depends on wall-clock scheduling. This matches Gazebo's default and old
  chrono_ros. For users who need gated/lockstep stepping (RL training, regression runs),
  v1 provides the primitive `ChROSSubscription::WaitForMessage(timeout)` (block draining
  IPC until a frame for that channel arrives) so the user loop can gate
  `DoStepDynamics` on controller output — the CARLA-synchronous-mode pattern, opt-in,
  built in user code, consider keeping. An optional per-update flush barrier (PUBLISH acks) is listed in
  §14; full FMI-style lockstep co-sim orchestration is explicitly out of scope (that is
  `chrono_fmi`'s domain).

## 5. Wire protocol (v1) — control + data frames

One bidirectional IPC channel. Every frame: `{magic, version, kind, channel_id,
sim_time_ns, payload_size}` + payload. Kinds:

| Kind              | Dir        | Payload                                              |
|-------------------|------------|------------------------------------------------------|
| `HELLO`           | sub → main | protocol version, pid, ROS distro, rmw id            |
| `DESCRIBE_TYPE`   | main → sub | type name (e.g. `sensor_msgs/msg/Image`)             |
| `TYPE_SCHEMA`     | sub → main | status + schema blob (§6.1)                          |
| `VALIDATE_TYPE`   | main → sub | type name + CDR of canonical test message            |
| `VALIDATE_RESULT` | sub → main | ok / first-diff offset + expected vs actual bytes    |
| `ADVERTISE`       | main → sub | channel_id, pub\|sub, topic, type name, QoS (§6.3)   |
| `ADVERTISE_ACK`   | sub → main | channel_id, ok / human-readable error                |
| `PUBLISH`         | main → sub | channel_id + CDR bytes                               |
| `RECEIVED`        | sub → main | channel_id + CDR bytes                               |
| `UNADVERTISE`     | main → sub | channel_id                                           |
| `CHANNEL_INFO`    | sub → main | channel_id + live subscription/publisher count (sent on change; lets sim-side handlers skip extraction when nobody listens) |
| `SHUTDOWN`        | main → sub | —                                                    |

The per-handler `MessageType` enum and `ChROSHandlerRegistry` are deleted; the dispatcher
switches on these ~10 fixed kinds forever.

## 6. Runtime schema & CDR codec

### 6.0 Where type knowledge lives (read this first)

The sim process never parses `.msg` files and never loads ROS code. Type-layout knowledge
is *acquired at runtime over IPC*: the subprocess (which can dlopen the installed
introspection typesupport for any type name) answers `DESCRIBE_TYPE` with a schema blob;
the sim side builds its serializer/deserializer from that.

**What triggers schema loading: declaration, not discovery.** The user's
`CreatePublisher("/cam1/feed", "sensor_msgs/msg/Image", qos)` call *is* the instruction —
the type-name string travels in `DESCRIBE_TYPE`/`ADVERTISE`, and the subprocess resolves
it to the installed typesupport `.so` through the sourced ROS environment
(`AMENT_PREFIX_PATH`), exactly as rosbag2 does. The ROS graph plays no part: whether any
subscriber exists is irrelevant (advertising a topic never requires subscribers — normal
ROS semantics; subscribers may join any time later). If the type's package is not
installed/sourced in the subprocess environment, `ADVERTISE_ACK` carries the error and
`Initialize()` fails fast naming the missing package — that sourcing step is the *only*
"registration" a custom message package ever needs. The user holds only *semantic*
knowledge — the type name string and the names of the fields they choose to set/read
(unset fields serialize as zero/empty; every rosidl field has a name; `DescribeType()`
prints the schema for discovery; typos fail fast with the valid field list). The
subprocess never populates or parses message fields at runtime in either direction — it
moves ready-made CDR bytes between the ring buffer and generic pub/sub. Forward and
backward paths are the same mechanism mirrored.

### 6.1 Schema blob
Versioned, packed, little-endian; produced by the subprocess walking
`rosidl_typesupport_introspection_cpp` recursively. A type table (each entry = list of
field records) + root index. Field record: `{name, kind, array_kind(none|fixed|bounded|
unbounded), array_size, nested_type_index}`. Kinds: bool, byte, char, int8–64, uint8–64,
float32/64, string, wstring, message. Both ends compile the same header that defines this
blob, so it can stay a simple packed format with a version field (it never crosses machine
or version boundaries — the subprocess is always built with the sim process).

### 6.2 CDR rules implemented by the sim-side codec
ROS 2 serialized form = 4-byte encapsulation header (`0x00 0x01 0x00 0x00` = CDR_LE +
options) followed by XCDR1 ("plain CDR") body: primitives aligned to their size (origin =
first byte after encapsulation header), `string` = u32 length incl. NUL + bytes + NUL,
unbounded/bounded sequence = u32 count + aligned elements, fixed array = elements only,
nested message = fields in order. All ROS 2 message types are `@final`, serialized as plain
CDR by both rmw_fastrtps and rmw_cyclonedds.

**We do not trust this by assertion — we verify per type at runtime (I6):** at
`Initialize()`, for each advertised type, the sim side serializes a canonical test-pattern
message and sends `VALIDATE_TYPE`; the subprocess generically constructs the message
(introspection init), deserializes our bytes through `rclcpp::SerializationBase`,
re-serializes, and byte-compares. Any mismatch (e.g. a future rmw moving to XCDR2) aborts
init with a precise error instead of publishing garbage. This also gives us a free
conformance test harness.

### 6.3 QoS surface
Minimal explicit struct: reliability {reliable, best_effort}, durability {volatile,
transient_local}, history depth — plus named presets: `"default"`, `"sensor_data"`,
`"clock"`, `"latched"`. (Latched/transient_local is required for `/robot_description`;
ClockQoS for `/clock`.)

### 6.4 Public sim-side API (C++)

```cpp
auto manager = chrono_types::make_shared<ChROSManager>("my_node");

// Low level: bridge handles (what built-ins are made of)
auto pub = manager->GetBridge()->CreatePublisher("~/output/gps", "sensor_msgs/msg/NavSatFix",
                                                 ChROSQoS::SensorData());
ChROSMessage msg = pub->NewMessage();          // backed by the runtime schema
msg.Set("header.frame_id", "gps_link");
msg.Set("latitude", lat);
msg.SetBlob("position_covariance", cov.data(), 9 * sizeof(double)); // primitive arrays
pub->Publish(msg);                              // CDR-serialize straight into ring buffer

auto sub = manager->GetBridge()->CreateSubscription("~/input/twist", "geometry_msgs/msg/Twist",
    ChROSQoS::Default(),
    [&](const ChROSMessageView& m) { throttle = m.GetDouble("linear.x"); });

// High level: rate-scheduled handler (old upstream UX restored)
class MyHandler : public ChROSHandler {            // SWIG director class
  bool Initialize(ChROSBridge& bridge) override;   // create handles here
  void Tick(double time) override;                 // fill + publish
};
manager->RegisterHandler(chrono_types::make_shared<MyHandler>(25 /*Hz*/));
```

`ChROSMessage` internals: field-value tree over the schema (random-order `Set`), blobs held
as pointer+length (no copy) until `Publish()` walks the schema in order and writes CDR
directly into the IPC frame — one copy total for bulk data, same as today. Unset fields
serialize as zero/empty. `ChROSMessageView` parses received CDR lazily by schema.

### 6.5 Canonical hello-world (the "arbitrary number sensor" test)

The acceptance bar for the whole rewrite: publishing one number from the sim on a new
topic must be exactly this much work, with **zero ROS-side code** (consumers are ordinary
ROS nodes / `ros2 topic echo`):

```python
class HeightSensor(chros.ChROSHandler):          # Python, via SWIG directors
    def __init__(self, box):
        super().__init__(100.0)                  # 100 Hz in sim time
        self.box = box
    def Initialize(self, bridge):
        self.pub = bridge.CreatePublisher("~/output/height", "std_msgs/msg/Float64",
                                          chros.QoS.Default())
        return True
    def Tick(self, time):
        msg = self.pub.NewMessage()
        msg["data"] = self.box.GetPos().z
        self.pub.Publish(msg)

manager.RegisterHandler(HeightSensor(box))
```

(C++ identical in shape; a `std::function`-based convenience wrapper exists for C++ so
trivial publishers need no subclass.) Compare today: enum edit + `_ipc.h` struct +
handler `.h/.cpp` + `_ros.cpp` + 2 CMake targets + full rebuild, impossible from Python.

## 7. SWIG / PyChrono design rules (R2 — the key driver)

Current state: `ChModuleROS.i` already builds with `%module(directors="1") ros`; every
built-in handler is individually wrapped; custom pathways from Python are impossible; and
the module links rclcpp (because the library does). After the rewrite the module wraps
**~8 small classes total**, links zero ROS, and Python becomes a first-class authoring
environment.

Binding rules (these constrain the C++ API design — enforce in review):

1. **No `std::function`, no templates, no raw `void*` in SWIG-visible signatures.**
   Callbacks are virtual classes with `%feature("director")`:
   `ChROSHandler` (`Initialize`/`Tick`) and `ChROSSubscriptionCallback` (`OnMessage`).
   C++-only `std::function` conveniences live in headers SWIG doesn't see (`%ignore`d
   overloads), implemented on top of the virtual classes.
2. **`%shared_ptr` for every public class** (matches PyChrono convention).
3. **`ChROSMessage` gets pythonic sugar via `%extend`:** `__setitem__`/`__getitem__`
   taking/returning Python scalars, str, bytes; `msg["header.frame_id"] = "cam"`.
   Value bridging via a small non-template variant (`ChROSValue`) with hand-written
   in/out typemaps.
4. **Bulk data: Python buffer protocol, zero-copy.** `SetBlob` gets a typemap accepting any
   C-contiguous buffer (numpy arrays included) via `PyObject_GetBuffer` → writes pixel/point
   data straight to the ring buffer at `Publish()`. Received blobs: `GetBlob` returns
   `bytes` (copy, safe default); `GetBlobView` returns a `memoryview` valid only inside the
   callback (documented; for high-rate consumers).
5. **GIL/threading:** all directors are invoked only inside `ChROSManager::Update()`,
   called from the user's Python sim loop → GIL already held, no cross-thread director
   calls, deterministic ordering. This is invariant I5 — never spawn sim-side threads that
   call user callbacks.
6. **Discoverability:** `bridge.DescribeType("sensor_msgs/msg/Image")` returns the schema
   (iterable fields, types) → REPL exploration; `str(schema)` pretty-prints. Error messages
   must name fields and types (e.g. `KeyError: no field 'lattitude' in
   sensor_msgs/msg/NavSatFix (did you mean 'latitude'?)`).
7. New Python demos required: custom publisher/subscriber (the pathway that is impossible
   today), and a numpy camera publisher (4K) as the performance proof.

## 8. Big-sensor compatibility analysis (R5)

Verified against current `_ipc.h` structs and Chrono::Sensor buffer shapes:

| Sensor | ROS type | Bulk part | Size @ max | New-path cost |
|---|---|---|---|---|
| Camera | `sensor_msgs/msg/Image` | `data: uint8[]` | 4K RGBA8 = 3840×2160×4 ≈ 31.6 MiB | header fields (~10 small CDR writes) + **one memcpy** of pixels into ring buffer — identical to today |
| Lidar | `sensor_msgs/msg/PointCloud2` | `data: uint8[]` (xyzi, 16 B/pt) | 128ch × 2048 ≈ 262k pts ≈ 4 MiB | `fields: PointField[]` = sequence of nested messages (4 entries) + one memcpy |
| Lidar 2D | `sensor_msgs/msg/LaserScan` | `ranges`, `intensities: float32[]` | ~KBs | two `SetBlob`s |

Conclusions: (a) every "big" type is *small structured header + one opaque primitive
sequence* — exactly the shape `SetBlob` optimizes; the only O(size) operation remains the
single copy into shared memory that the current design also performs. (b) The schema/codec
**must support sequences of nested messages** (PointCloud2's `PointField[]`) — covered in
the fixture corpus. (c) Frame sizing must accommodate ≥ 64 MiB payloads (D7).
A benchmark (4K camera @30 Hz + lidar, old vs new path) is a Phase 5 exit criterion.
(d) Built-in big-sensor handlers consult the `CHANNEL_INFO` subscription count and skip
extraction + transfer entirely while nobody subscribes (standard rclcpp-practice analog of
`get_subscription_count()`); user handlers can do the same via
`ChROSPublisher::GetSubscriptionCount()`.

## 9. Built-in handlers to re-deliver on the new API (R3 — final phase)

Clock (`rosgraph_msgs/Clock`, ClockQoS) · Body (`nav_msgs/Odometry`-style pose/twist/accel
topics) · TF (`tf2_msgs/TFMessage` on `/tf`) · RobotModel (`std_msgs/String` on
`/robot_description`, latched) · DriverInputs (subscriber, `chrono_ros_interfaces` — proves
custom-package support) · GPS · Accelerometer · Gyroscope · Magnetometer · IMU · Camera ·
Lidar (PC2 + LaserScan) · ViperDCMotorControl (subscriber) · CraneState · ActuatorState.

All are pure topics → all expressible on the generic path (this fact is why D3 holds).
Chrono-side extraction logic (e.g. GPS covariance running average, camera buffer access)
is ported from the legacy handlers — those files were removed from the tree on 2026-06-12;
retrieve them from git history (the commit before the maintainer's "remove legacy code"
commit) when porting in Phase 5.

## 10. Implementation plan & status

**Status (updated 2026-06-12): Phases 1-3 implemented; awaiting first Docker build
(maintainer builds; paste errors back). Phase 1 is fully unit-tested locally (56 tests,
ASan/UBSan, `tests_dev/run_dev_tests.sh`); Phases 2-3 are compile-verified only where
possible (sim-side sources syntax-checked against stub headers; `node/` needs rclcpp).**

**Legacy code REMOVED 2026-06-12** (maintainer's decision; he marks it with a dedicated
"remove legacy code" git commit in the chrono-wisc submodule — recover any legacy file
for Phase 5 porting reference via `git log`/`git show` on that commit's parent). Removed:
`ChROSInterface.*`, `ChROSIPCInterface.*`, `ChROSHandlerRegistry.h`, the root legacy
`chrono_ros_node.cpp`, `ipc/`, `handlers/` (including all `_ipc.h`/`_ros.cpp` and the old
authoring README), the six legacy C++ demos, the five legacy Python demos, and the three
legacy unit tests. The legacy utests were replaced by
`tests/unit_tests/ros/utest_ROS_bridge_roundtrip.cpp` — a gtest end-to-end suite (bridge
pub → rclcpp peer, rclcpp peer → bridge callback, pixel-exact Image round trip,
DescribeType + fail-fast error cases) that doubles as the Phase 2/3 integration test and
deliberately links rclcpp + Chrono_ros in one binary to prove the isolation design.
Note: the repo also moved on 2026-06-12 — chrono-wisc is now a git submodule at
`/home/chen2465/wautosim/WAutoSim/chrono-wisc`.

Implementation layout (new code):
- `core/` — chrono_ros_core static lib (no Chrono, no ROS): `ChROSCdr`, `ChROSSchema`,
  `ChROSMessageCodec` (MessageBuilder/MessageReader), `ChROSFrame`, `ChROSControl`,
  `ChROSQoSSpec`, `core/transport/` (RingBuffer, SharedMemory, Channel).
- `node/` — chrono_ros_node executable (ROS only): `ChROSNodeTypeSupport`
  (introspection→schema walker + VALIDATE round-trip), `chrono_ros_node.cpp` (generic app).
- module root — Chrono_ros shared lib (Chrono only): `ChROSBridge`, `ChROSManager`,
  `ChROSHandler`, `ChROSMessage`/`ChROSMessageView`, `ChROSPublisher`/`ChROSSubscription`
  (+ `ChROSSubscriptionCallback` director base), `ChROSQoS`, `ChROSNodeProcess`.
- `tests_dev/` — dev tests + runner (see its README); `.stubs/` for Chrono-free syntax checks.
- `demo_ROS_custom_handler.cpp` rewritten on the new API (only demo built for now;
  others return in Phase 5).
- PyChrono `ros` module disabled in `chrono_swig/chrono_python/CMakeLists.txt` until
  Phase 4 (guard: never-defined `CH_PYCHRONO_ROS_LEGACY`).

Known deferred items: the rclcpp version shim in `node/ChROSNodeTypeSupport.cpp`
(`RCLCPP_VERSION_MAJOR >= 17`) must be verified on the CI distros; validation tolerates
trailing zero padding (some serializers pad to 4 bytes) — confirm against rmw_fastrtps
and rmw_cyclonedds during Phase 2 integration testing.

**Reference build environment (updated 2026-06-12):** `contrib/docker/` upgraded from
Ubuntu 22.04/Humble to **Ubuntu 26.04 "resolute" + ROS 2 Lyrical + CUDA 13.1**. Package
names verified on a live ubuntu:26.04 image. Non-obvious 26.04 facts baked into the
dockerfiles: CUDA toolkit now comes from Ubuntu multiverse (`cuda-toolkit-13-1`; NVIDIA's
repo has no 26.04 toolkit), LunarG's Ubuntu Vulkan repo is discontinued (distro Vulkan
dev packages instead), ROS installs via the `ros2-apt-source` deb, pip needs PEP 668
handling, and GCC 15.2 + CMake 4.2 require compat flags for the SynChrono FastDDS 2.4.0
build.

SynChrono ⨯ Chrono::ROS Fast-DDS conflict (2026-06-12): ROS Lyrical bundles Fast-DDS
**3.6.1** / Fast-CDR 2.3.6; SynChrono's standalone build is Fast-DDS **2.4.0**. They share
the imported CMake target `eProsima_atomic`, so when `find_package(rclcpp)` pulls ROS's 3.x
config it fatal-errors on the half-defined export set ("some but not all targets defined").
Patrick's earlier fix (commit a52be5a) made them coexist under **Humble**, where ROS also
shipped Fast-DDS 2.x (matching target sets); Lyrical's 3.x breaks that premise. Note: the
`IMPORTED_GLOBAL` foreach a52be5a added to chrono_ros/CMakeLists.txt is moot under D2 (the
new `Chrono_ros` links no ROS, so nothing downstream needs those targets global) and was
intentionally not restored. RESOLUTION (this image): SynChrono runs **MPI-only** while
Chrono::ROS is enabled — `ch_synchrono.dockerfile` comments out the `CH_USE_SYNCHRONO_FASTDDS`
+ `*_DIR` options (one-line reversible; the standalone Fast-DDS build block is left intact).
A separate agent is porting SynChrono's DDS layer (SynDDSCommunicator) to Fast-DDS 3.x so
the FastDDS backend can coexist with Lyrical later; do NOT edit chrono_synchrono sources
here — that is the other agent's scope.

Chrono-core ⨯ CUDA 13.3 CCCL/Thrust (FIXED 2026-06-13, pending Docker verify): CUDA 13.3
bundles CCCL/Thrust 3.3.3, which removed `thrust::distance/advance/iterator_difference` and
moved `thrust::tuple/make_tuple/get` to `cuda::std::`. `find_package(Thrust)` resolves to
CUDA's `/usr/local/cuda/include/cccl`, and the affected code is built into Chrono_core
UNCONDITIONALLY (multicore collision; not gated on the Sensor/CUDA module). Upstream Chrono
`main` is also unmigrated — no fix to pull. Patrick has Chrono maintenance rights and chose
the proper backwards-compatible port. FIX (3 files = this build's whole exposure):
`multicore_math/thrust.h` adds a version-guarded alias
`namespace chrono::ch_thrust = cuda::std (THRUST_VERSION>=300000) | thrust (2.x)` for
tuple/make_tuple/get, and switches `Thrust_Expand`'s iterator ops to `std::distance`/
`std::advance`/`std::iterator_traits::difference_type` (universal; host/OMP backend);
`collision/multicore/ChBroadphase.cpp` + `ChNarrowphase.cpp` use `ch_thrust::` for
tuple/make_tuple/get (kept `thrust::make_zip_iterator`, present on 3.x and accepts a
cuda::std::tuple). Backwards-compatible: CUDA ≤12 / Thrust 2.x keeps `thrust::`. Docker
verify points: (1) `thrust::make_zip_iterator(cuda::std::make_tuple(...))` accepts the tuple
on 3.x; (2) functor `operator()(const ch_thrust::tuple<...>&)` binds the zip reference type.
NOT compile-checked offline (no CUDA/CCCL here). Same `ch_thrust` pattern will be needed in
`chrono_fsi` (.cu) and `chrono_multicore` if those modules are ever enabled — NOT enabled in
this image, so deferred. Chrono::Sensor (enabled) does not use the removed symbols.

First-build shake-out (2026-06-12, against this docker):
- SynChrono fatal-error'd: its `find_package(fastrtps PATHS ${FastDDS_ROOT}/cmake)` hint
  is wrong — Fast-DDS 2.4 installs config to `share/fastrtps/cmake` (fastcdr →
  `share/fastcdr/cmake`, foonathan_memory → `lib/foonathan_memory/cmake`, all verified
  from upstream install rules). FIXED in ch_synchrono.dockerfile by pinning
  `fastrtps_DIR`/`fastcdr_DIR`/`foonathan_memory_DIR` (cache vars beat the bad PATHS and
  the ROS-bundled copies). The bad PATHS in `src/chrono_synchrono/CMakeLists.txt` is the
  true root cause and could alternatively be fixed there.
- CUDA (RESOLVED 2026-06-12): the `rsqrt` exception-spec conflict (nvcc fails on glibc
  2.43; cmake `check_language(CUDA)` → NOTFOUND → Sensor silently dropped OptiX) was
  specific to **CUDA 13.1**, which is all Ubuntu 26.04's multiverse ships. The earlier
  switch to multiverse (from a wrong "NVIDIA has no 26.04 toolkit" web finding) is what
  caused it. NVIDIA's own `ubuntu2604` repo carries **CUDA 13.3** (13.3.0-1), whose docs
  validate Ubuntu 26.04 and which fixed the header. Verified in a container: 13.3 compiles
  a kernel on glibc 2.43 and cmake detects it. `cuda.dockerfile` now uses NVIDIA's repo;
  `CUDA_VERSION=13-3`. No glibc downgrade (impossible on 26.04) and no header patch needed.
  NVIDIA's CUDA-13.3 doc table lists "glibc 2.38" for the 26.04 row — that cell is a doc
  error (26.04 ships 2.43; the same row's GCC 15.2 is correct).
- Parsers "Package ROS2 NOT found": root-caused to an incomplete fork refactor — commit
  aa0278d4c2 ("modern cmake WIP") deleted `cmake/FindROS2.cmake` and rewired chrono_ros to
  direct `find_package(rclcpp)`, but left `chrono_parsers/CMakeLists.txt` still calling
  `find_package(ROS2 COMPONENTS ament_index_cpp)`, which needs that module. FIXED by
  restoring `cmake/FindROS2.cmake` from aa0278d4c2^ (54-line wrapper; only parsers consumes
  it, so chrono_ros isolation is untouched). Unrelated to the chrono_ros rewrite and to
  the distro bump. chrono_ros configure wasn't reached yet (SynChrono died first), so the
  rewrite still has no reported issues.
- NumPy 2.x vs chrono_swig `numpy.i` (PyChrono) and CUDA 13 dropping pre-Turing arches
  remain watch items. ros-lyrical mirror 404s possible in early weeks.

(Approved by maintainer 2026-06-11: schema-driven direction, no-backcompat, Python-first,
plugin API dropped.)

- [x] **Phase 0 — Specs.** Wire protocol v1 (§5) and schema blob v1 (§6.1) frozen (in
      this file; promote to `docs/` pages in Phase 6); CI matrix per D6 (Lyrical > Jazzy
      > Humble). *Adjusted:* golden fixtures are hand-computed from the CDR spec in
      `tests_dev/` (no ROS needed locally); the authoritative cross-check against the
      real rmw is the per-type VALIDATE handshake at runtime + Phase 2 integration tests.
- [x] **Phase 1 — Schema core + CDR codec (sim side, zero ROS).** Done as
      `core/` (`Schema`, `MessageBuilder`/`MessageReader`, `CdrWriter`/`CdrReader`,
      frames, control payloads, ring/shm/channel transport). 56 tests green under
      ASan/UBSan via `tests_dev/run_dev_tests.sh`, including Image/PointCloud2/JointState
      -shaped round trips and golden wire bytes.
- [x] **Phase 2 — Generic subprocess.** (code complete, NOT yet compiled — needs ROS)
      `node/chrono_ros_node.cpp` + `node/ChROSNodeTypeSupport.cpp`. Registry deleted from
      the build. ☐ remaining: compile on CI distros, integration test both directions,
      VALIDATE against rmw_fastrtps + rmw_cyclonedds.
- [x] **Phase 3 — Sim-side bridge + manager.** (code complete, syntax-checked via stub
      headers — needs Chrono to fully compile) `ChROSBridge` (handshake, schema cache,
      per-type validation, dispatch, WaitForMessage), handles, new `ChROSHandler`,
      `ChROSManager`, QoS presets, `demo_ROS_custom_handler.cpp` rewritten; ROS link
      dropped from `Chrono_ros`, Chrono link dropped from the node (D2). ☐ remaining:
      end-to-end run in Docker.
- [ ] **Phase 4 — SWIG/PyChrono.** Rewrite `ChModuleROS.i` per §7 (directors, variant
      typemaps, buffer-protocol blobs, `%extend` sugar). New demos:
      `demo_ROS_custom_handler.py`, `demo_ROS_numpy_camera.py`. Exit: Python-only custom
      pub/sub of a custom message package, no C++ written.
- [ ] **Phase 5 — Re-deliver built-ins (R3) + perf.** Port all §9 handlers as exemplary
      uses of the public API; 4K camera + lidar benchmark vs legacy path (exit: ≤ legacy
      cost). **Same-function demos must return** (hard requirement, maintainer
      2026-06-12; inventory verified against upstream release/9.0): C++
      `demo_ROS_sensor`, `demo_ROS_urdf`, `demo_ROS_vehicle`, `demo_ROS_viper`,
      `demo_ROS_two_managers`, `demo_ROS_hydraulic_crane` (fork-specific); Python
      `demo_ROS_sensor/urdf/vehicle/viper/two_managers.py` (after Phase 4). Legacy
      sources for porting: git history (D8) or upstream
      github.com/projectchrono/chrono tree `release/9.0/src/chrono_ros`.
- [ ] **Phase 6 — Cleanup & docs.** ~~Delete legacy files~~ (done early, 2026-06-12, at
      maintainer's request — see status note). Remaining: write the new handler-authoring
      guide ("you don't write bridge code") + exotic-sensor tutorial; promote protocol/
      schema specs to `docs/` pages; doxygen group pages; register `tests_dev` suites with
      CTest; final API review pass.

Each phase merges only when its exit criterion passes; later phases may not begin on a
broken base. No deadlines (R1: done right > done fast).

## 11. ROS 2 distro compatibility (verified 2026-06-11)

| Capability (needed by) | Humble (LTS→5/2027) | Jazzy (LTS→5/2029) | Kilted | **Lyrical Luth (LTS 5/2026→5/2031)** |
|---|---|---|---|---|
| Generic pub/sub (core of design) | ✅ | ✅ | ✅ | ✅ |
| Introspection typesupport (schema/validate) | ✅ | ✅ | ✅ | ✅ |
| Generic service **client** (future opt.) | ❌ | ✅ | ✅ | ✅ |
| Generic service **server** (future opt.) | ❌ | ❌ (backport requested) | ✅ | ✅ |

Topics-only v1 works on **every current distro including both active LTSes**. CI priority:
**Lyrical > Jazzy > Humble** (Humble best-effort). Services/actions: out of v1 scope; add
later as built-in bridge capability gated on distro (never as user plugins, per D3).
C++17 stays (rclcpp minimum).

## 12. Invariants (check every PR against these)

- **I1** Zero ROS symbols/links in the sim process (`Chrono_ros` lib, PyChrono module).
- **I2** Users never write bridge/subprocess code or rebuild chrono_ros for a new pathway.
- **I3** Any installed message type — including user packages — usable by type name string.
- **I4** Bulk sensor data crosses process boundary with exactly one copy.
- **I5** All user callbacks fire on the sim thread inside `ChROSManager::Update()`.
- **I6** Schema/serialization mismatches abort at `Initialize()` with actionable errors —
  never silent corruption at runtime.
- **I7** Built-in handlers use only the public API (R6).
- **I8** Zero Chrono symbols/links in `chrono_ros_node` (mirror of I1; the shared
  `chrono_ros_ipc` static lib depends on neither world).

## 13. Decision log

- **D1** (2026-06-11) Schema-driven generic bridge; wire payload = ROS CDR. Over FMI,
  direct-DDS, plugin-API alternatives (§3).
- **D2** (2026-06-11) `Chrono_ros` library becomes 100% ROS-free; ROS only in
  `chrono_ros_node`; node becomes Chrono-free (shared `chrono_ros_ipc` static lib
  depends on neither); direct rclcpp mode + `ChROSInterface` deleted.
  *Why the current cross-linking survives despite the known VSG↔ROS collision:* a
  collision needs both dependency trees mapped **and activated** in one process. The
  node links Chrono_core but Chrono_core pulls no VSG (that's Chrono_vsg) — no overlap.
  The sim process maps rclcpp (via Chrono_ros) alongside VSG but never calls
  `rclcpp::init()` in IPC mode, so the rmw/DDS plugin stack — where the heavy colliding
  deps load from — is never dlopened, and lazily-bound symbols for uncalled code never
  resolve. That is restraint + luck, not a guarantee (one eager-bound data symbol or one
  in-process init away from the original crash), and it taxes every consumer (PyChrono
  carries the full ROS tree). The rewrite replaces "happens not to collide" with
  "cannot collide" (I1 + I8: disjoint link sets).
- **D3** (2026-06-11) No public subprocess plugin API. Re-evaluated at maintainer's
  prompt; confirmed drop (reasons in §3). Future gaps → built-in bridge capabilities.
- **D4** (2026-06-11) Python-first API rules (§7) are design constraints, not afterthoughts.
- **D5** (2026-06-11) No backward compatibility; legacy code will be deleted; built-ins
  re-delivered last (R3).
- **D6** (2026-06-11, updated same day) CI priority: **Lyrical Luth > Jazzy > Humble**;
  Humble is best-effort ("great if it works at all" — maintainer). Protocol v1 is
  topics-only; maintainer concurs simulation rarely models as services, so generic
  service support is low-priority future work.
- **D7** (2026-06-11) Keep shm ring-buffer IPC; move to variable-size frames; payload
  capacity configurable (≥ 64 MiB).
- **D8** (2026-06-12) Legacy code removed from the tree ahead of Phase 6, at the
  maintainer's request; he marks the removal with a dedicated git commit so legacy code
  remains retrievable via git history for Phase 5 porting. Legacy unit tests replaced by
  the end-to-end `utest_ROS_bridge_roundtrip` suite.
  **Lineage terminology (maintainer, 2026-06-12):** there are TWO legacies. (1)
  *Super-legacy* = upstream Chrono 9.0 chrono_ros (in-process rclcpp, no IPC; tree at
  github.com/projectchrono/chrono `release/9.0/src/chrono_ros`): origin of the built-in
  handlers and demos; supported runtime custom handlers because users had direct rclcpp
  access. (2) *Legacy* = the maintainer's IPC rewrite in this fork (Chrono 10 era; the
  code this rewrite inherited and deleted, in git history at D8's commit parent): ported
  all super-legacy functionality into the isolated-process architecture *except* runtime
  custom handlers — the capability this schema-driven rewrite restores, by different
  means. For Phase 5 porting: super-legacy is often the cleaner reference for Chrono-side
  extraction logic and demo structure (its handlers are self-contained, no IPC split);
  the IPC legacy is the reference for fork-specific additions (CraneState/ActuatorState,
  hydraulic crane demo) and for process/transport lessons already absorbed into the new
  core.

## 14. Open questions (resolve before the phase that needs them)

- Exact `ChROSValue` variant set for SWIG typemaps (Phase 4): scalars + str + bytes —
  is a list-of-str needed (e.g. `PointField` names) or is nested-message `Set` enough?
- Zero-copy *received* blobs in Python: ship `GetBlobView` in v1 or defer?
- PointCloud2 numpy structured-array sugar (helper that builds `fields[]` + blob from a
  structured dtype) — nice-to-have, Phase 5?
- Multiple `ChROSManager`s (multi-node) — channel naming already unique per instance;
  confirm subprocess-per-manager is acceptable (it is today).
- Whether to expose deadline/lifespan QoS in v1 or keep the minimal trio.
- Optional per-update flush barrier (PUBLISH acks, "strict outbound mode") for
  deterministic CI runs — ship in v1 or defer? (`WaitForMessage` is committed; §4.1.)
