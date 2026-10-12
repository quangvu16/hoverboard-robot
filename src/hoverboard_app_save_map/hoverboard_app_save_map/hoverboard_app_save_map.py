"""Save the current /map on request, and record it in a small SQLite catalogue.

Service (hoverboard_web_savmap_interfaces/srv/WebSaveMap):
    request.mapname  -> saved as <maps_dir>/<mapname>.yaml/.pgm
    response.response = "<mapname>" on success, "error: ..." otherwise

The default service name is Yahboom's "yahboomAppSaveMap", so the Yahboom
phone/web app keeps working; set the service_name parameter to change it.

Parameters:
    service_name   default "yahboomAppSaveMap"
    maps_dir       default ~/maps
    db_path        default ~/.ros/hoverboard_maps.db (created on first use,
                   same xgo_map table layout as Yahboom's xgo.db)
    save_timeout   seconds to wait for map_saver_cli, default 20
"""
import datetime
import hashlib
import os
import sqlite3
import subprocess

import rclpy
from rclpy.node import Node

from hoverboard_web_savmap_interfaces.srv import WebSaveMap

SCHEMA = """CREATE TABLE IF NOT EXISTS xgo_map (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    map_id TEXT NOT NULL,
    map_name TEXT NOT NULL UNIQUE,
    map_path TEXT NOT NULL,
    create_time DATETIME DEFAULT CURRENT_TIMESTAMP)"""


class AppSaveMap(Node):

    def __init__(self):
        super().__init__('app_save_map')
        self.service_name = self.declare_parameter('service_name', 'yahboomAppSaveMap').value
        self.maps_dir = os.path.expanduser(self.declare_parameter('maps_dir', '~/maps').value)
        self.db_path = os.path.expanduser(
            self.declare_parameter('db_path', '~/.ros/hoverboard_maps.db').value)
        self.save_timeout = float(self.declare_parameter('save_timeout', 20.0).value)

        os.makedirs(self.maps_dir, exist_ok=True)
        os.makedirs(os.path.dirname(self.db_path), exist_ok=True)
        with sqlite3.connect(self.db_path) as conn:
            conn.execute(SCHEMA)

        self.srv = self.create_service(WebSaveMap, self.service_name, self.on_save)
        self.get_logger().info(
            f'service "{self.service_name}": maps -> {self.maps_dir}, catalogue {self.db_path}')

    def on_save(self, request, response):
        name = os.path.basename(request.mapname.strip())   # no path tricks
        if not name:
            response.response = 'error: empty map name'
            return response
        map_path = os.path.join(self.maps_dir, name)

        cmd = ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', map_path,
               '--ros-args', '-p', f'save_map_timeout:={self.save_timeout * 1000.0:.1f}']
        self.get_logger().info(' '.join(cmd))
        try:
            proc = subprocess.run(cmd, capture_output=True, text=True,
                                  timeout=self.save_timeout + 10.0)
        except subprocess.TimeoutExpired:
            response.response = 'error: map_saver_cli timed out (is /map published?)'
            self.get_logger().error(response.response)
            return response
        if proc.returncode != 0 or not os.path.exists(map_path + '.yaml'):
            response.response = 'error: map_saver_cli failed: ' + (proc.stderr or proc.stdout)[-300:]
            self.get_logger().error(response.response)
            return response

        stamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
        map_id = hashlib.md5((stamp + name).encode()).hexdigest()
        try:
            with sqlite3.connect(self.db_path) as conn:
                conn.execute('INSERT OR REPLACE INTO xgo_map (map_name, map_id, map_path) '
                             'VALUES (?, ?, ?)', (name, map_id, map_path))
        except sqlite3.Error as e:
            # The map files are saved either way; only the catalogue failed.
            response.response = f'error: saved {map_path}.yaml but catalogue failed: {e}'
            self.get_logger().error(response.response)
            return response

        response.response = name
        self.get_logger().info(f'saved {map_path}.yaml')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = AppSaveMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
