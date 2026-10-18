package com.hoverboard.robotnav

/**
 * Manual driving rules, shared by the stick and the Stop button.
 *
 * tick() runs at RATE_HZ. While the stick is off-centre it returns a command
 * to publish on /cmd_vel_manual; on release it returns ZEROES zero commands
 * and then nothing, so Nav2's /cmd_vel is left alone while you're not driving.
 * The hoverboard bridge stops the wheels by itself 0.5 s after the last
 * command (cmd_timeout_s), which covers a dropped connection.
 */
class Drive {
    data class Twist(val vx: Double, val wz: Double) {
        val moving get() = vx != 0.0 || wz != 0.0
    }

    var maxSpeed = 0.2      // m/s at full stick forward
    var maxTurn = 0.8       // rad/s at full stick sideways
    private var stickX = 0f
    private var stickY = 0f
    private var zeroes = 0
    var moving = false
        private set

    fun stick(x: Float, y: Float) { stickX = x; stickY = y }

    /** Stick right = turn right = negative angular.z (REP 103). */
    fun command() = Twist(stickY * maxSpeed, -stickX * maxTurn).let {
        Twist(it.vx + 0.0, it.wz + 0.0)      // no -0.0 on the wire
    }

    /**
     * The command to publish now, or null for "send nothing". [started] is true
     * on the tick the robot goes from standing to moving.
     */
    fun tick(): Pair<Twist?, Boolean> {
        val cmd = command()
        val started = cmd.moving && !moving
        moving = cmd.moving
        return when {
            cmd.moving -> { zeroes = ZEROES; cmd to started }
            zeroes > 0 -> { zeroes--; Twist(0.0, 0.0) to false }
            else -> null to false
        }
    }

    /** Centre the stick and queue a burst of zeros. */
    fun stop() { stickX = 0f; stickY = 0f; zeroes = STOP_ZEROES }

    companion object {
        const val RATE_HZ = 10
        const val ZEROES = 3
        const val STOP_ZEROES = 5
    }
}
