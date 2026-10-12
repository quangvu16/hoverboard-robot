package com.hoverboard.robotnav

import android.os.Handler
import android.os.Looper
import okhttp3.OkHttpClient
import okhttp3.Request
import okhttp3.Response
import okhttp3.WebSocket
import okhttp3.WebSocketListener
import org.json.JSONObject
import java.util.concurrent.TimeUnit

/**
 * Minimal rosbridge v2 client (the JSON protocol roslib.js speaks).
 *
 * subscribe() is remembered and re-sent after every reconnect; publish()
 * advertises a topic the first time it is used on a connection. State changes
 * are delivered on the main thread; messages arrive on OkHttp's reader thread
 * so big ones (/map) can be decoded off the UI thread.
 */
class RosBridge(private val listener: Listener) {

    enum class State { DISCONNECTED, CONNECTING, CONNECTED }

    interface Listener {
        fun onState(state: State, detail: String)      // main thread
        fun onMessage(topic: String, msg: JSONObject)  // OkHttp thread
    }

    private val client = OkHttpClient.Builder()
        .connectTimeout(5, TimeUnit.SECONDS)
        .pingInterval(10, TimeUnit.SECONDS)
        .build()
    private val main = Handler(Looper.getMainLooper())
    private val subscriptions = LinkedHashMap<String, JSONObject>()
    private val advertised = HashSet<String>()
    private var socket: WebSocket? = null
    private var url = ""
    private var wanted = false
    @Volatile var state = State.DISCONNECTED
        private set

    fun subscribe(topic: String, type: String, throttleMs: Int = 0, queueLength: Int = 1) {
        val op = JSONObject()
            .put("op", "subscribe").put("id", "sub:$topic")
            .put("topic", topic).put("type", type)
            .put("throttle_rate", throttleMs).put("queue_length", queueLength)
        subscriptions[topic] = op
        if (state == State.CONNECTED) socket?.send(op.toString())
    }

    /** Returns false when not connected (nothing is queued). */
    fun publish(topic: String, type: String, msg: JSONObject): Boolean {
        val ws = socket ?: return false
        if (state != State.CONNECTED) return false
        if (advertised.add(topic)) {
            ws.send(JSONObject().put("op", "advertise").put("id", "adv:$topic")
                .put("topic", topic).put("type", type).toString())
        }
        return ws.send(JSONObject().put("op", "publish").put("topic", topic).put("msg", msg).toString())
    }

    fun connect(newUrl: String) {
        wanted = true
        if (newUrl != url || state == State.DISCONNECTED) {
            url = newUrl
            socket?.cancel()
            open()
        }
    }

    fun disconnect() {
        wanted = false
        main.removeCallbacksAndMessages(null)
        socket?.close(1000, "bye")
        socket = null
        setState(State.DISCONNECTED, "stopped")
    }

    private fun open() {
        main.removeCallbacksAndMessages(null)
        setState(State.CONNECTING, url)
        val request = try {
            Request.Builder().url(url).build()
        } catch (e: IllegalArgumentException) {
            setState(State.DISCONNECTED, "bad address: $url")
            return
        }
        lateinit var ws: WebSocket
        ws = client.newWebSocket(request, object : WebSocketListener() {
            override fun onOpen(webSocket: WebSocket, response: Response) {
                main.post {
                    if (socket !== ws) return@post
                    advertised.clear()
                    setState(State.CONNECTED, url)
                    subscriptions.values.forEach { ws.send(it.toString()) }
                }
            }

            override fun onMessage(webSocket: WebSocket, text: String) {
                if (socket !== ws) return
                val json = try { JSONObject(text) } catch (e: Exception) { return }
                if (json.optString("op") == "publish") {
                    val msg = json.optJSONObject("msg") ?: return
                    listener.onMessage(json.optString("topic"), msg)
                }
            }

            override fun onFailure(webSocket: WebSocket, t: Throwable, response: Response?) =
                lost(ws, t.message ?: t.javaClass.simpleName)

            override fun onClosed(webSocket: WebSocket, code: Int, reason: String) =
                lost(ws, "closed")
        })
        socket = ws
    }

    private fun lost(ws: WebSocket, why: String) {
        main.post {
            if (socket !== ws) return@post
            socket = null
            setState(State.DISCONNECTED, why)
            if (wanted) main.postDelayed({ if (wanted && socket == null) open() }, 2000)
        }
    }

    private fun setState(s: State, detail: String) {
        state = s
        listener.onState(s, detail)
    }
}
