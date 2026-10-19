package com.hoverboard.robotnav

import android.app.Activity
import android.app.AlertDialog
import android.content.Context
import android.content.res.ColorStateList
import android.graphics.Bitmap
import android.graphics.Typeface
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.os.SystemClock
import android.text.InputType
import android.view.Gravity
import android.view.View
import android.view.WindowManager
import android.widget.Button
import android.widget.EditText
import android.widget.LinearLayout
import android.widget.SeekBar
import android.widget.TableLayout
import android.widget.TableRow
import android.widget.TextView
import org.json.JSONArray
import org.json.JSONObject
import java.util.Locale
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.roundToInt
import kotlin.math.sin

/**
 * Navigate the hoverboard robot through rosbridge (robot_web_nav on the PC):
 * same topics as the web page - /map, /plan, /web/robot_pose, /web/nav_status,
 * /ultrasonic/obstacle_near in; /web/goal, /web/cancel, /initialpose and
 * /cmd_vel_manual (the drive stick) out. Plus the hoverboard bridge's
 * /hoverboard/battery_{state,volts} and /range/{front,left,right} in, and the
 * twist_mux lock /cmd_vel_lock both ways.
 */
class MainActivity : Activity(), RosBridge.Listener {

    private lateinit var ros: RosBridge
    private lateinit var mapView: MapView
    private val ui = Handler(Looper.getMainLooper())
    private val prefs by lazy { getSharedPreferences("robot_nav", Context.MODE_PRIVATE) }

    private var robot: MapView.Pose2D? = null
    private var robotT = 0L
    private var statusT = 0L
    private var navState = "idle"

    private lateinit var pConn: TextView
    private lateinit var pStack: TextView
    private lateinit var pNav: TextView
    private lateinit var pMap: TextView
    private lateinit var sState: TextView
    private lateinit var sMsg: TextView
    private lateinit var sDot: View
    private lateinit var btnCancel: Button
    private val detail = HashMap<String, TextView>()

    private val drive = Drive()
    private lateinit var stick: JoystickView
    private lateinit var pSonar: TextView
    private lateinit var dCmd: TextView
    private lateinit var dSonar: TextView
    private lateinit var dLock: TextView
    private lateinit var pBatt: TextView

    // Battery, from the ESP32 via the bridge (firmware battery.h).
    private var battState = ""                 // "", ok, low, critical, unknown
    private var battVolts = Double.NaN
    // Sonars in metres; NaN = no reading yet, +inf = no echo.
    private val sonar = doubleArrayOf(Double.NaN, Double.NaN, Double.NaN)
    // twist_mux lock. lockSeen is what /cmd_vel_lock last carried (from anyone,
    // this app included); null = nothing seen since connecting. While holding,
    // the app re-sends true every LOCK_REPEAT_MS so other clients see it too.
    private var lockSeen: Boolean? = null
    private var holding = false

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)

        mapView = findViewById(R.id.map)
        pConn = findViewById(R.id.pConn); pStack = findViewById(R.id.pStack)
        pNav = findViewById(R.id.pNav); pMap = findViewById(R.id.pMap)
        sState = findViewById(R.id.sState); sMsg = findViewById(R.id.sMsg); sDot = findViewById(R.id.sDot)
        btnCancel = findViewById(R.id.btnCancel)
        buildDetails()

        ros = RosBridge(this)
        ros.subscribe("/map", "nav_msgs/msg/OccupancyGrid", throttleMs = 2000)
        ros.subscribe("/web/robot_pose", "geometry_msgs/msg/PoseStamped", throttleMs = 100)
        ros.subscribe("/web/nav_status", "std_msgs/msg/String")
        ros.subscribe("/plan", "nav_msgs/msg/Path", throttleMs = 500)
        ros.subscribe("/ultrasonic/obstacle_near", "std_msgs/msg/Bool", throttleMs = 200)
        ros.subscribe("/hoverboard/battery_state", "std_msgs/msg/String")
        ros.subscribe("/hoverboard/battery_volts", "std_msgs/msg/Float32", throttleMs = 1000)
        for (side in SIDES) ros.subscribe("/range/$side", "sensor_msgs/msg/Range", throttleMs = 200)
        ros.subscribe("/cmd_vel_lock", "std_msgs/msg/Bool")
        setupDrive()

        mapView.onPick = { mode, x, y, yaw -> onPick(mode, x, y, yaw) }
        findViewById<Button>(R.id.mGoal).setOnClickListener { setMode(MapView.Mode.GOAL) }
        findViewById<Button>(R.id.mPose).setOnClickListener { setMode(MapView.Mode.POSE) }
        findViewById<Button>(R.id.mPan).setOnClickListener { setMode(MapView.Mode.PAN) }
        findViewById<Button>(R.id.zFit).setOnClickListener { mapView.fit() }
        findViewById<Button>(R.id.zRobot).setOnClickListener { mapView.centreOnRobot() }
        findViewById<Button>(R.id.btnRobot).setOnClickListener { askAddress() }
        findViewById<Button>(R.id.spotSave).setOnClickListener { saveSpot() }
        btnCancel.setOnClickListener {
            if (!ros.publish("/web/cancel", "std_msgs/msg/Empty", JSONObject())) toast("Not connected")
        }
        setMode(MapView.Mode.GOAL)
        showState("idle", "")
        renderSpots()
    }

    override fun onStart() {
        super.onStart()
        val url = prefs.getString("url", null)
        if (url == null) askAddress() else ros.connect(url)
        ui.post(ticker)
        ui.post(driveTicker)
        ui.post(lockTicker)
    }

    // Leaving the app (home, another app, screen off) lets go of the stick and
    // sends the stop burst while the socket is still open.
    override fun onPause() {
        super.onPause()
        stopDriving()
    }

    override fun onStop() {
        super.onStop()
        ui.removeCallbacks(ticker)
        ui.removeCallbacks(driveTicker)
        ui.removeCallbacks(lockTicker)
        ros.disconnect()
    }

    // ---------------------------------------------------------------- address
    private fun askAddress() {
        val input = EditText(this).apply {
            setText(prefs.getString("url", "ws://192.168.1.12:9090"))
            inputType = InputType.TYPE_CLASS_TEXT or InputType.TYPE_TEXT_VARIATION_URI
            setSingleLine()
            setSelection(text.length)
        }
        val box = LinearLayout(this).apply {
            setPadding(dp(20), dp(8), dp(20), 0)
            addView(input, LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT))
        }
        AlertDialog.Builder(this)
            .setTitle("Robot PC address")
            .setMessage("rosbridge websocket of the PC running robot_web_nav, e.g. ws://192.168.1.12:9090")
            .setView(box)
            .setPositiveButton("Connect") { _, _ ->
                var url = input.text.toString().trim()
                if (!url.startsWith("ws://") && !url.startsWith("wss://")) url = "ws://$url"
                if (!url.substringAfter("://").contains(":")) url += ":9090"
                prefs.edit().putString("url", url).apply()
                ros.connect(url)
            }
            .setNegativeButton("Cancel", null)
            .show()
    }

    // ---------------------------------------------------------------- rosbridge
    override fun onState(state: RosBridge.State, detail: String) {
        when (state) {
            RosBridge.State.CONNECTED -> pill(pConn, "connected", R.color.ok, R.color.ok_soft)
            RosBridge.State.CONNECTING -> pill(pConn, "connecting…", R.color.muted, R.color.sunk)
            RosBridge.State.DISCONNECTED -> {
                pill(pSonar, "sonar ?", R.color.muted, R.color.sunk)
                lockSeen = null; showLock()
                pill(pConn, "no rosbridge", R.color.bad, R.color.bad_soft)
                if (detail.isNotEmpty() && detail != "stopped") sMsg.text = "Connection: $detail"
            }
        }
    }

    override fun onMessage(topic: String, msg: JSONObject) {
        when (topic) {
            "/map" -> {                       // decode here, off the UI thread
                val grid = decodeMap(msg) ?: return
                ui.post {
                    mapView.setMap(grid)
                    findViewById<View>(R.id.overlay).visibility = View.GONE
                    pill(pMap, "map ${grid.w}×${grid.h} @ ${(grid.res * 100).roundToInt()} cm", R.color.ok, R.color.ok_soft)
                }
            }
            "/web/robot_pose" -> {
                val p = msg.getJSONObject("pose")
                val pos = p.getJSONObject("position")
                val pose = MapView.Pose2D(pos.getDouble("x"), pos.getDouble("y"), yawOf(p.getJSONObject("orientation")))
                ui.post {
                    robot = pose; robotT = SystemClock.elapsedRealtime()
                    mapView.robotStale = false
                    mapView.setRobot(pose)
                    detail["xy"]?.text = String.format(Locale.US, "%.2f, %.2f m", pose.x, pose.y)
                    detail["heading"]?.text = deg(pose.yaw)
                }
            }
            "/plan" -> {
                val poses = msg.getJSONArray("poses")
                val xy = DoubleArray(poses.length() * 2)
                for (i in 0 until poses.length()) {
                    val pos = poses.getJSONObject(i).getJSONObject("pose").getJSONObject("position")
                    xy[2 * i] = pos.getDouble("x"); xy[2 * i + 1] = pos.getDouble("y")
                }
                ui.post { if (navState == "active" || navState == "sending") mapView.setPlan(xy) }
            }
            "/web/nav_status" -> {
                val st = try { JSONObject(msg.getString("data")) } catch (e: Exception) { return }
                ui.post { onStatus(st) }
            }
            "/hoverboard/battery_state" -> {
                val s = msg.optString("data")
                ui.post { battState = s; showBattery() }
            }
            "/hoverboard/battery_volts" -> {
                val v = msg.optDouble("data", Double.NaN)
                ui.post { battVolts = v; showBattery() }
            }
            "/range/front", "/range/left", "/range/right" -> {
                val i = SIDES.indexOf(topic.substringAfterLast('/'))
                val r = msg.optDouble("range", Double.NaN)
                val max = msg.optDouble("max_range", Double.POSITIVE_INFINITY)
                // The bridge sends exactly max_range for "no echo".
                val m = if (r.isNaN() || r >= max - 1e-3) Double.POSITIVE_INFINITY else r
                ui.post { sonar[i] = m; showSonar() }
            }
            "/cmd_vel_lock" -> {
                val on = msg.optBoolean("data")
                ui.post { lockSeen = on; showLock() }
            }
            "/ultrasonic/obstacle_near" -> {
                val near = msg.optBoolean("data")
                ui.post {
                    if (near) pill(pSonar, "obstacle ahead", R.color.warn, R.color.warn_soft)
                    else pill(pSonar, "sonar clear", R.color.ok, R.color.ok_soft)
                }
            }
        }
    }

    private fun decodeMap(msg: JSONObject): MapView.GridMap? {
        val info = msg.getJSONObject("info")
        val w = info.getInt("width"); val h = info.getInt("height")
        if (w <= 0 || h <= 0) return null
        val origin = info.getJSONObject("origin").getJSONObject("position")
        val data: JSONArray = msg.getJSONArray("data")
        val px = IntArray(w * h)
        for (y in 0 until h) {
            val row = (h - 1 - y) * w            // grid row 0 is the bottom of the map
            for (x in 0 until w) {
                val v = data.getInt(y * w + x)
                px[row + x] = if (v < 0) 0xFFC8CCD2.toInt() else {
                    val g = 250 - v * 222 / 100   // free 250 .. occupied 28
                    (0xFF shl 24) or (g shl 16) or (g shl 8) or g
                }
            }
        }
        val bmp = Bitmap.createBitmap(px, w, h, Bitmap.Config.ARGB_8888)
        return MapView.GridMap(bmp, w, h, info.getDouble("resolution"), origin.getDouble("x"), origin.getDouble("y"))
    }

    private fun onStatus(st: JSONObject) {
        statusT = SystemClock.elapsedRealtime()
        navState = st.optString("state", "idle")
        showState(navState, st.optString("message"))
        val g = st.optJSONObject("goal")
        if (g != null) {
            val goal = MapView.Pose2D(g.getDouble("x"), g.getDouble("y"), g.getDouble("yaw"))
            mapView.setGoal(goal)
            detail["goal"]?.text = String.format(Locale.US, "%.2f, %.2f · %s", goal.x, goal.y, deg(goal.yaw))
        } else {
            mapView.setGoal(null); detail["goal"]?.text = "—"
        }
        val live = navState == "active"
        detail["dist"]?.text = if (live && st.has("distance_remaining")) String.format(Locale.US, "%.2f m", st.getDouble("distance_remaining")) else "—"
        detail["eta"]?.text = if (live && st.optDouble("eta_s", 0.0) > 0) "${st.getDouble("eta_s").roundToInt()} s" else "—"
        detail["time"]?.text = if (live && st.has("nav_time_s")) "${st.getDouble("nav_time_s").roundToInt()} s" else "—"
        detail["rec"]?.text = if (live && st.has("recoveries")) st.getInt("recoveries").toString() else "—"
        pill(pStack, "stack " + st.optString("stack", "?"), R.color.accent, R.color.accent_soft)
        if (st.optBoolean("server_ready")) pill(pNav, "Nav2 ready", R.color.ok, R.color.ok_soft)
        else pill(pNav, "Nav2 not running", R.color.bad, R.color.bad_soft)
        if (!live && navState != "sending") mapView.setPlan(DoubleArray(0))
    }

    private val ticker = object : Runnable {
        override fun run() {
            val now = SystemClock.elapsedRealtime()
            if (ros.state == RosBridge.State.CONNECTED && now - statusT > 3000) {
                pill(pNav, "web_nav_bridge silent", R.color.bad, R.color.bad_soft)
            }
            if (robot != null) {
                val age = (now - robotT) / 1000.0
                detail["age"]?.text = String.format(Locale.US, "%.1f s", age)
                mapView.robotStale = age > 2.0
            }
            ui.postDelayed(this, 1000)
        }
    }

    // ---------------------------------------------------------------- manual drive
    private fun setupDrive() {
        stick = findViewById(R.id.stick)
        pSonar = findViewById(R.id.pSonar)
        dCmd = findViewById(R.id.dCmd)
        // Publish at once on press and release; while held, driveTicker sends at RATE_HZ
        // (touch moves arrive at 60+ Hz, too fast to forward one by one).
        stick.onStick = { x, y ->
            val was = drive.moving
            drive.stick(x, y)
            if (!was || (x == 0f && y == 0f)) driveNow()
        }
        findViewById<Button>(R.id.btnStop).setOnClickListener { stopDriving() }
        dSonar = findViewById(R.id.dSonar)
        dLock = findViewById(R.id.dLock)
        pBatt = findViewById(R.id.pBatt)
        findViewById<Button>(R.id.btnHold).setOnClickListener { setHold(true) }
        findViewById<Button>(R.id.btnRelease).setOnClickListener { setHold(false) }
        showLock()

        // Speed 0.05..0.5 m/s in 0.05 steps, turn 0.2..2.0 rad/s in 0.1 steps; remembered.
        val dV = findViewById<SeekBar>(R.id.dV); val dVo = findViewById<TextView>(R.id.dVo)
        val dW = findViewById<SeekBar>(R.id.dW); val dWo = findViewById<TextView>(R.id.dWo)
        fun apply() {
            drive.maxSpeed = 0.05 + dV.progress * 0.05
            drive.maxTurn = 0.2 + dW.progress * 0.1
            dVo.text = String.format(Locale.US, "%.2f m/s", drive.maxSpeed)
            dWo.text = String.format(Locale.US, "%.1f rad/s", drive.maxTurn)
        }
        dV.progress = prefs.getInt("drive_v", 3)      // 0.2 m/s
        dW.progress = prefs.getInt("drive_w", 6)      // 0.8 rad/s
        apply()
        val listener = object : SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(s: SeekBar, p: Int, fromUser: Boolean) { apply() }
            override fun onStartTrackingTouch(s: SeekBar) {}
            override fun onStopTrackingTouch(s: SeekBar) {
                prefs.edit().putInt("drive_v", dV.progress).putInt("drive_w", dW.progress).apply()
            }
        }
        dV.setOnSeekBarChangeListener(listener); dW.setOnSeekBarChangeListener(listener)
    }

    private val driveTicker = object : Runnable {
        override fun run() {
            driveNow()
            ui.postDelayed(this, 1000L / Drive.RATE_HZ)
        }
    }

    /** One drive step: publish what Drive says, cancel a goal when driving starts. */
    private fun driveNow() {
        // A held robot ignores the stick anyway (twist_mux); say so by greying it.
        stick.isEnabled = ros.state == RosBridge.State.CONNECTED && lockSeen != true
        val (cmd, started) = drive.tick()
        if (started && (navState == "active" || navState == "sending")) {
            ros.publish("/web/cancel", "std_msgs/msg/Empty", JSONObject())
            sMsg.text = "Goal cancelled: driving by hand."
        }
        if (cmd != null) {
            ros.publish("/cmd_vel_manual", "geometry_msgs/msg/Twist", JSONObject()
                .put("linear", JSONObject().put("x", cmd.vx).put("y", 0.0).put("z", 0.0))
                .put("angular", JSONObject().put("x", 0.0).put("y", 0.0).put("z", cmd.wz)))
        }
        val now = drive.command()
        dCmd.text = if (now.moving) String.format(Locale.US, "%.2f m/s · %.2f rad/s", now.vx, now.wz) else "stopped"
    }

    // ---------------------------------------------------------------- lock, battery, sonar
    private fun publishLock(on: Boolean) =
        ros.publish("/cmd_vel_lock", "std_msgs/msg/Bool", JSONObject().put("data", on))

    /** Hold still (true) or release (false). Release is sent three times, 0.3 s apart. */
    private fun setHold(on: Boolean) {
        holding = on
        if (on) stopDriving()
        if (!publishLock(on)) { toast("Not connected"); return }
        if (!on) for (k in 1..2) ui.postDelayed({ if (!holding) publishLock(false) }, 300L * k)
    }

    private val lockTicker = object : Runnable {
        override fun run() {
            if (holding) publishLock(true)
            ui.postDelayed(this, LOCK_REPEAT_MS)
        }
    }

    private fun showLock() {
        if (!::dLock.isInitialized) return
        dLock.text = when (lockSeen) {
            true -> "lock: ON, robot held still"
            false -> "lock: off"
            null -> "lock: unknown (needs twist_mux)"
        }
        dLock.setTextColor(getColor(if (lockSeen == true) R.color.bad else R.color.muted))
    }

    private fun showBattery() {
        val v = if (battVolts.isNaN()) "" else String.format(Locale.US, " %.1f V", battVolts)
        when (battState) {
            "low" -> pill(pBatt, "battery low$v", R.color.warn, R.color.warn_soft)
            "critical" -> pill(pBatt, "battery critical$v", R.color.bad, R.color.bad_soft)
            "ok" -> pill(pBatt, "battery$v", R.color.ok, R.color.ok_soft)
            else -> pill(pBatt, if (v.isEmpty()) "battery ?" else "battery$v", R.color.muted, R.color.sunk)
        }
    }

    private fun showSonar() {
        fun f(m: Double) = when {
            m.isNaN() -> "—"
            m.isInfinite() -> "clear"
            else -> String.format(Locale.US, "%.2f", m)
        }
        dSonar.text = "sonar  front ${f(sonar[0])}  left ${f(sonar[1])}  right ${f(sonar[2])}"
    }

    private fun stopDriving() {
        drive.stop()
        if (::stick.isInitialized) stick.release()   // reports (0, 0) -> driveNow()
        driveNow()
    }

    // ---------------------------------------------------------------- commands
    private fun onPick(mode: MapView.Mode, x: Double, y: Double, yaw: Double?) {
        val r = robot
        if (mode == MapView.Mode.GOAL) {
            // a plain tap: face the way it drove there
            val heading = yaw ?: if (r != null) atan2(y - r.y, x - r.x) else 0.0
            sendGoal(x, y, heading)
        } else {
            sendInitialPose(x, y, yaw ?: r?.yaw ?: 0.0)
        }
    }

    private fun header(): JSONObject {
        val ms = System.currentTimeMillis()
        return JSONObject().put("frame_id", "map")
            .put("stamp", JSONObject().put("sec", ms / 1000).put("nanosec", (ms % 1000) * 1_000_000))
    }

    private fun pose(x: Double, y: Double, yaw: Double) = JSONObject()
        .put("position", JSONObject().put("x", x).put("y", y).put("z", 0.0))
        .put("orientation", JSONObject().put("x", 0.0).put("y", 0.0).put("z", sin(yaw / 2)).put("w", cos(yaw / 2)))

    private fun sendGoal(x: Double, y: Double, yaw: Double) {
        val ok = ros.publish("/web/goal", "geometry_msgs/msg/PoseStamped",
            JSONObject().put("header", header()).put("pose", pose(x, y, yaw)))
        if (ok) mapView.setGoal(MapView.Pose2D(x, y, yaw)) else toast("Not connected: goal not sent")
    }

    private fun sendInitialPose(x: Double, y: Double, yaw: Double) {
        val cov = JSONArray()
        for (i in 0 until 36) cov.put(when (i) { 0, 7 -> 0.25; 35 -> 0.0685; else -> 0.0 })
        val ok = ros.publish("/initialpose", "geometry_msgs/msg/PoseWithCovarianceStamped",
            JSONObject().put("header", header()).put("pose", JSONObject().put("pose", pose(x, y, yaw)).put("covariance", cov)))
        sMsg.text = if (ok) String.format(Locale.US, "Initial pose sent: %.2f, %.2f · %s", x, y, deg(yaw))
                    else "Not connected: pose not sent"
    }

    // ---------------------------------------------------------------- UI helpers
    private fun setMode(m: MapView.Mode) {
        mapView.mode = m
        mapOf(R.id.mGoal to MapView.Mode.GOAL, R.id.mPose to MapView.Mode.POSE, R.id.mPan to MapView.Mode.PAN)
            .forEach { (id, mode) ->
                val b = findViewById<Button>(id)
                val on = mode == m
                b.backgroundTintList = ColorStateList.valueOf(getColor(if (on) R.color.accent else R.color.surface))
                b.setTextColor(getColor(if (on) R.color.on_accent else R.color.fg))
            }
        findViewById<TextView>(R.id.hint).text = when (m) {
            MapView.Mode.GOAL -> "Tap where to go · drag to set heading · two fingers to zoom"
            MapView.Mode.POSE -> "Tap where the robot really is · drag the way it faces"
            MapView.Mode.PAN -> "Drag to move · pinch to zoom"
        }
    }

    private val looks = mapOf(
        "idle" to Triple("Idle", R.color.muted, false), "sending" to Triple("Sending…", R.color.accent, true),
        "active" to Triple("Driving", R.color.accent, true), "succeeded" to Triple("Arrived", R.color.ok, false),
        "aborted" to Triple("Failed", R.color.bad, false), "canceled" to Triple("Cancelled", R.color.warn, false),
        "rejected" to Triple("Rejected", R.color.bad, false), "unavailable" to Triple("Nav2 not running", R.color.bad, false),
    )

    private fun showState(state: String, message: String) {
        val (label, color, cancellable) = looks[state] ?: Triple(state, R.color.muted, false)
        sState.text = label
        sDot.backgroundTintList = ColorStateList.valueOf(getColor(color))
        if (message.isNotEmpty() || state != "idle") sMsg.text = message
        btnCancel.isEnabled = cancellable
        btnCancel.alpha = if (cancellable) 1f else 0.35f
    }

    private fun pill(v: TextView, text: String, fg: Int, bg: Int) {
        v.text = text
        v.setTextColor(getColor(fg))
        v.backgroundTintList = ColorStateList.valueOf(getColor(bg))
    }

    private fun buildDetails() {
        val table = findViewById<TableLayout>(R.id.details)
        listOf("goal" to "Goal", "dist" to "Distance left", "eta" to "ETA", "time" to "Driving for",
            "rec" to "Recoveries", "xy" to "Robot x, y", "heading" to "Robot heading", "age" to "Pose age")
            .forEach { (key, label) ->
                val row = TableRow(this).apply { setPadding(0, dp(3), 0, dp(3)) }
                row.addView(TextView(this).apply { text = label; setTextColor(getColor(R.color.muted)); textSize = 14f })
                val value = TextView(this).apply {
                    text = "—"; gravity = Gravity.END; typeface = Typeface.MONOSPACE
                    setTextColor(getColor(R.color.fg)); textSize = 14f
                }
                row.addView(value)
                detail[key] = value
                table.addView(row)
            }
    }

    // ---------------------------------------------------------------- saved spots
    private fun loadSpots(): JSONArray = try { JSONArray(prefs.getString("spots", "[]")) } catch (e: Exception) { JSONArray() }

    private fun saveSpot() {
        val r = robot ?: return toast("No robot pose yet, so nothing to save")
        val field = findViewById<EditText>(R.id.spotName)
        val spots = loadSpots()
        val name = field.text.toString().trim().ifEmpty { "Spot ${spots.length() + 1}" }
        spots.put(JSONObject().put("name", name).put("x", r.x).put("y", r.y).put("yaw", r.yaw))
        prefs.edit().putString("spots", spots.toString()).apply()
        field.setText("")
        renderSpots()
    }

    private fun renderSpots() {
        val list = findViewById<LinearLayout>(R.id.spots)
        list.removeAllViews()
        val spots = loadSpots()
        if (spots.length() == 0) {
            list.addView(TextView(this).apply {
                text = "None yet. Drive the robot somewhere, name it, and tap “Save here”."
                setTextColor(getColor(R.color.muted)); textSize = 14f
            })
            return
        }
        for (i in 0 until spots.length()) {
            val s = spots.getJSONObject(i)
            val row = LinearLayout(this).apply {
                orientation = LinearLayout.HORIZONTAL; gravity = Gravity.CENTER_VERTICAL
                setPadding(0, dp(4), 0, dp(4))
            }
            val label = TextView(this).apply {
                text = String.format(Locale.US, "%s\n%.2f, %.2f · %s", s.getString("name"),
                    s.getDouble("x"), s.getDouble("y"), deg(s.getDouble("yaw")))
                setTextColor(getColor(R.color.fg)); textSize = 14f
            }
            row.addView(label, LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f))
            row.addView(smallButton("Go", true) { sendGoal(s.getDouble("x"), s.getDouble("y"), s.getDouble("yaw")) })
            row.addView(smallButton("✕", false) {
                val a = loadSpots(); a.remove(i)
                prefs.edit().putString("spots", a.toString()).apply(); renderSpots()
            })
            list.addView(row)
        }
    }

    private fun smallButton(label: String, primary: Boolean, onClick: () -> Unit) = Button(this).apply {
        text = label; isAllCaps = false; textSize = 14f; minWidth = 0; minimumWidth = 0; minHeight = 0; minimumHeight = 0
        setPadding(dp(14), 0, dp(14), 0)
        setBackgroundResource(R.drawable.rounded)
        backgroundTintList = ColorStateList.valueOf(getColor(if (primary) R.color.accent else R.color.sunk))
        setTextColor(getColor(if (primary) R.color.on_accent else R.color.fg))
        stateListAnimator = null
        layoutParams = LinearLayout.LayoutParams(LinearLayout.LayoutParams.WRAP_CONTENT, dp(38)).apply { marginStart = dp(6) }
        setOnClickListener { onClick() }
    }

    private fun toast(t: String) { android.widget.Toast.makeText(this, t, android.widget.Toast.LENGTH_SHORT).show() }
    private fun dp(v: Int) = (v * resources.displayMetrics.density).roundToInt()
    private fun yawOf(q: JSONObject): Double {
        val x = q.getDouble("x"); val y = q.getDouble("y"); val z = q.getDouble("z"); val w = q.getDouble("w")
        return atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    }
    private fun deg(r: Double) = "${Math.toDegrees(r).roundToInt()}°"

    companion object {
        private val SIDES = listOf("front", "left", "right")
        private const val LOCK_REPEAT_MS = 2000L
    }
}
