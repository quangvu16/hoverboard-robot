package com.hoverboard.robotnav

import android.content.Context
import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.Matrix
import android.graphics.Paint
import android.graphics.Path
import android.util.AttributeSet
import android.view.MotionEvent
import android.view.ScaleGestureDetector
import android.view.View
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sin

/**
 * Occupancy grid + robot + plan + goal, with RViz-style picking:
 * press = position, drag = heading, release = send. Two fingers always pan/zoom.
 *
 * Screen = offset + cell * scale, with the grid's y flipped (row 0 is the
 * bottom of the map), same as the web page.
 */
class MapView @JvmOverloads constructor(context: Context, attrs: AttributeSet? = null) :
    View(context, attrs) {

    enum class Mode { GOAL, POSE, PAN }

    class GridMap(val bitmap: Bitmap, val w: Int, val h: Int, val res: Double, val x0: Double, val y0: Double)
    data class Pose2D(val x: Double, val y: Double, val yaw: Double)

    /** yaw is null when the user only tapped (no heading dragged). */
    var onPick: ((mode: Mode, x: Double, y: Double, yaw: Double?) -> Unit)? = null

    var mode = Mode.GOAL
    private var map: GridMap? = null
    private var robot: Pose2D? = null
    var robotStale = false
        set(v) { field = v; invalidate() }
    private var goal: Pose2D? = null
    private var plan = DoubleArray(0)

    private var scale = 0f
    private var ox = 0f
    private var oy = 0f

    private val density = resources.displayMetrics.density
    private val c = { id: Int -> context.getColor(id) }
    private val mapPaint = Paint().apply { isFilterBitmap = false }
    private val stroke = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE; strokeCap = Paint.Cap.ROUND; strokeJoin = Paint.Join.ROUND
    }
    private val fill = Paint(Paint.ANTI_ALIAS_FLAG).apply { style = Paint.Style.FILL }
    private val text = Paint(Paint.ANTI_ALIAS_FLAG).apply { textSize = 12 * density; color = c(R.color.fg) }
    private val matrix = Matrix()
    private val path = Path()

    // ---------------------------------------------------------------- data
    fun setMap(m: GridMap) {
        val old = map
        map = m
        if (old == null || scale == 0f) fit()
        else if (old.w != m.w || old.h != m.h || old.x0 != m.x0 || old.y0 != m.y0) {
            // Cartographer grows the grid: keep the same world point under the same pixel.
            ox += ((m.x0 - old.x0) / m.res * scale).toFloat()
            oy += ((old.h - m.h + (old.y0 - m.y0) / m.res) * scale).toFloat()
        }
        invalidate()
    }

    fun setRobot(p: Pose2D) { robot = p; invalidate() }
    fun setGoal(p: Pose2D?) { goal = p; invalidate() }
    fun setPlan(xy: DoubleArray) { plan = xy; invalidate() }
    fun hasMap() = map != null

    fun fit() {
        val m = map ?: return
        if (width == 0 || height == 0) return
        scale = min(width.toFloat() / m.w, height.toFloat() / m.h) * 0.95f
        ox = (width - m.w * scale) / 2f
        oy = (height - m.h * scale) / 2f
        invalidate()
    }

    fun centreOnRobot() {
        val r = robot ?: return
        if (map == null) return
        val (sx, sy) = toScreen(r.x, r.y)
        ox += width / 2f - sx
        oy += height / 2f - sy
        invalidate()
    }

    override fun onSizeChanged(w: Int, h: Int, oldw: Int, oldh: Int) {
        if (scale == 0f) fit() else invalidate()
    }

    private fun toScreen(x: Double, y: Double): Pair<Float, Float> {
        val m = map!!
        val cx = (x - m.x0) / m.res
        val cy = (y - m.y0) / m.res
        return Pair((ox + cx * scale).toFloat(), (oy + (m.h - cy) * scale).toFloat())
    }

    private fun toWorld(sx: Float, sy: Float): Pair<Double, Double> {
        val m = map!!
        val cx = (sx - ox) / scale
        val cy = m.h - (sy - oy) / scale
        return Pair(m.x0 + cx * m.res, m.y0 + cy * m.res)
    }

    // ---------------------------------------------------------------- touch
    private var downX = 0f
    private var downY = 0f
    private var curX = 0f
    private var curY = 0f
    private var picking = false
    private var panning = false
    private var lastFocusX = 0f
    private var lastFocusY = 0f

    private val scaler = ScaleGestureDetector(context, object : ScaleGestureDetector.SimpleOnScaleGestureListener() {
        override fun onScale(d: ScaleGestureDetector): Boolean {
            val ns = max(0.2f, min(80f, scale * d.scaleFactor))
            val k = ns / scale
            ox = d.focusX - (d.focusX - ox) * k
            oy = d.focusY - (d.focusY - oy) * k
            scale = ns
            invalidate()
            return true
        }
    })

    override fun onTouchEvent(e: MotionEvent): Boolean {
        if (map == null) return true
        scaler.onTouchEvent(e)
        when (e.actionMasked) {
            MotionEvent.ACTION_DOWN -> {
                downX = e.x; downY = e.y; curX = e.x; curY = e.y
                lastFocusX = e.x; lastFocusY = e.y
                picking = mode != Mode.PAN
                panning = mode == Mode.PAN
                parent?.requestDisallowInterceptTouchEvent(true)
            }
            MotionEvent.ACTION_POINTER_DOWN -> {
                picking = false              // a second finger turns any pick into pan/zoom
                panning = true
                lastFocusX = focusX(e); lastFocusY = focusY(e)
            }
            MotionEvent.ACTION_MOVE -> {
                if (panning) {
                    val fx = focusX(e); val fy = focusY(e)
                    ox += fx - lastFocusX; oy += fy - lastFocusY
                    lastFocusX = fx; lastFocusY = fy
                } else if (picking) {
                    curX = e.x; curY = e.y
                }
                invalidate()
            }
            MotionEvent.ACTION_POINTER_UP -> {
                // keep panning with the remaining finger without a jump
                val skip = e.actionIndex
                var sx = 0f; var sy = 0f; var n = 0
                for (i in 0 until e.pointerCount) if (i != skip) { sx += e.getX(i); sy += e.getY(i); n++ }
                if (n > 0) { lastFocusX = sx / n; lastFocusY = sy / n }
            }
            MotionEvent.ACTION_UP -> {
                if (picking) {
                    picking = false
                    val (wx, wy) = toWorld(downX, downY)
                    val dragged = hypot(curX - downX, curY - downY) > 14 * density
                    val yaw = if (dragged) atan2(-(curY - downY), curX - downX).toDouble() else null
                    performClick()
                    onPick?.invoke(mode, wx, wy, yaw)
                }
                panning = false
                invalidate()
            }
            MotionEvent.ACTION_CANCEL -> { picking = false; panning = false; invalidate() }
        }
        return true
    }

    override fun performClick(): Boolean = super.performClick()

    private fun focusX(e: MotionEvent): Float { var s = 0f; for (i in 0 until e.pointerCount) s += e.getX(i); return s / e.pointerCount }
    private fun focusY(e: MotionEvent): Float { var s = 0f; for (i in 0 until e.pointerCount) s += e.getY(i); return s / e.pointerCount }

    // ---------------------------------------------------------------- drawing
    private fun arrow(canvas: Canvas, x: Float, y: Float, yaw: Double, len: Float, color: Int, width: Float) {
        stroke.color = color; stroke.strokeWidth = width; fill.color = color
        val ex = x + (cos(yaw) * len).toFloat()
        val ey = y - (sin(yaw) * len).toFloat()
        canvas.drawLine(x, y, ex, ey, stroke)
        val h = max(8 * density, len * 0.3f)
        path.reset()
        path.moveTo(ex, ey)
        path.lineTo(ex - (h * cos(yaw - 0.5)).toFloat(), ey + (h * sin(yaw - 0.5)).toFloat())
        path.lineTo(ex - (h * cos(yaw + 0.5)).toFloat(), ey + (h * sin(yaw + 0.5)).toFloat())
        path.close()
        canvas.drawPath(path, fill)
    }

    override fun onDraw(canvas: Canvas) {
        val m = map ?: return
        matrix.reset()
        matrix.setScale(scale, scale)
        matrix.postTranslate(ox, oy)
        canvas.drawBitmap(m.bitmap, matrix, mapPaint)
        val pxPerM = (scale / m.res).toFloat()

        // 1 m scale bar
        fill.color = c(R.color.fg)
        val bx = width - 16 * density - pxPerM
        val by = height - 16 * density
        canvas.drawRect(bx, by, bx + pxPerM, by + 3 * density, fill)
        canvas.drawText("1 m", bx, by - 5 * density, text)

        if (plan.size >= 4) {
            path.reset()
            for (i in plan.indices step 2) {
                val (sx, sy) = toScreen(plan[i], plan[i + 1])
                if (i == 0) path.moveTo(sx, sy) else path.lineTo(sx, sy)
            }
            stroke.color = c(R.color.accent); stroke.strokeWidth = 3 * density
            canvas.drawPath(path, stroke)
        }
        goal?.let { g ->
            val (gx, gy) = toScreen(g.x, g.y)
            fill.color = c(R.color.goal)
            canvas.drawCircle(gx, gy, 6 * density, fill)
            arrow(canvas, gx, gy, g.yaw, max(24 * density, 0.4f * pxPerM), c(R.color.goal), 3 * density)
        }
        robot?.let { r ->
            val (rx, ry) = toScreen(r.x, r.y)
            val rad = max(8 * density, 0.18f * pxPerM)   // chassis is ~0.33 m wide
            val col = if (robotStale) c(R.color.muted) else c(R.color.accent)
            fill.color = col; fill.alpha = 64
            canvas.drawCircle(rx, ry, rad, fill)
            fill.alpha = 255
            stroke.color = col; stroke.strokeWidth = 2 * density
            canvas.drawCircle(rx, ry, rad, stroke)
            arrow(canvas, rx, ry, r.yaw, rad * 1.6f, col, 3 * density)
        }
        if (picking) {
            val col = if (mode == Mode.GOAL) c(R.color.goal) else c(R.color.ok)
            fill.color = col
            canvas.drawCircle(downX, downY, 7 * density, fill)
            val len = hypot(curX - downX, curY - downY)
            if (len > 14 * density) {
                arrow(canvas, downX, downY, atan2(-(curY - downY), curX - downX).toDouble(), len, col, 3 * density)
            }
        }
    }
}
