"""Exercise actual records and a read-only observer in an isolated ROS graph."""
import csv
import io
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import urllib.request

import pytest

from build_scope import HERE, ROOT, build
from serve_scope import LiveBuffer


@pytest.fixture(scope="module")
def browser():
    from playwright.sync_api import sync_playwright
    with sync_playwright() as playwright:
        candidates = sorted((Path.home() / ".cache/ms-playwright").glob("chromium-*/chrome-linux64/chrome"))
        options = {"executable_path": str(candidates[-1])} if candidates else {}
        browser = playwright.chromium.launch(headless=True, args=["--no-sandbox"], **options)
        yield browser
        browser.close()


@pytest.fixture(scope="module")
def artifact():
    return build()


def test_buffer_ignores_nonfinite_and_returns_only_new_events():
    buffer = LiveBuffer()
    buffer.append("feedback", [1, 50, 0.1, 0.2, 6])
    buffer.append("feedback", [2, 100, float("nan"), 0.2, 6])
    first = buffer.snapshot(0)
    assert len(first["events"]) == 1
    assert first["feedback_age"] is not None
    buffer.append("command_active", [False])
    later = buffer.snapshot(first["cursor"])
    assert [row[1] for row in later["events"]] == ["command_active"]
    assert later["session"] == first["session"]


def test_receive_calendar_time_survives_wall_clock_adjustment(monkeypatch):
    buffer = LiveBuffer()
    ticks = iter([buffer.started + 1, buffer.started + 2, buffer.started + 3])
    wall = iter([1790500000123000000, 1790499995123000000])
    monkeypatch.setattr(time, "monotonic", lambda: next(ticks))
    monkeypatch.setattr(time, "time_ns", lambda: next(wall))
    buffer.append("feedback", [1, 50, 0.1, 0.2, 6])
    buffer.append("feedback", [2, 100, 0.1, 0.2, 6])
    packet = buffer.snapshot(0)
    first, second = [event[2] for event in packet["events"]]
    index = packet["wall_time_indices"]["feedback"]
    assert second[0] - first[0] == 1
    assert first[index] == 1790500000123
    assert second[index] - first[index] == -5000


def test_calendar_display_exports_and_legacy_fallback(browser, artifact, tmp_path):
    from datetime import datetime

    page = browser.new_page(viewport={"width": 1440, "height": 1000}, accept_downloads=True)
    errors = []
    page.on("pageerror", lambda error: errors.append(str(error)))
    page.goto(artifact.as_uri())
    chart = "echarts.getInstanceByDom(document.getElementById('chart'))"
    assert "未记录实际时间" in page.evaluate(chart + ".getOption().title[0].subtext")
    epoch = int(datetime.fromisoformat("2026-09-27T20:15:30.123+08:00").timestamp() * 1000)
    record = {"rows": {"feedback": [[1, 1, 50, 0.1, 0.2, 6, epoch],
                                      [2, 2, 100, 0.1, 0.2, 3, epoch + 1000]],
                       "forwarded": [[1, 0.1, 0.2, epoch]]},
              "scope": {"wall_time_indices": {"feedback": 6, "forwarded": 3},
                        "wall_time_source": "observer_host_receive_unix_ms"}}
    page.locator("#file-input").set_input_files({"name": "calendar.json", "mimeType": "application/json", "buffer": json.dumps(record).encode()})
    page.wait_for_function("document.querySelector('#metric-count').textContent === '2'")
    assert "2026-09-27 20:15:30.123" in page.evaluate(chart + ".getOption().title[0].subtext")
    assert "2026-09-27 20:15:31.123" in page.locator("#events").inner_text()
    page.evaluate(chart + ".dispatchAction({type:'showTip',seriesIndex:0,dataIndex:0})")
    assert "2026-09-27 20:15:30.123 +08:00" in page.locator("body").inner_text()
    for selector, suffix in [("#export-csv", ".csv"), ("#save-json", ".json"), ("#export-png", ".png")]:
        with page.expect_download() as info:
            page.locator(selector).click()
        info.value.save_as(tmp_path / ("calendar" + suffix))
    exported = json.loads((tmp_path / "calendar.json").read_text())
    assert exported["rows"]["feedback"][0][6] == epoch
    assert exported["scope"]["wall_time_indices"]["feedback"] == 6
    rows = list(csv.DictReader(io.StringIO((tmp_path / "calendar.csv").read_text())))
    assert rows[0]["host_receive_time_utc08"] == "2026-09-27T20:15:30.123+08:00"
    page.locator("#file-input").set_input_files(str(tmp_path / "calendar.json"))
    page.wait_for_function("document.querySelector('#source-select').selectedOptions[0].textContent === 'calendar.json'")
    assert "2026-09-27 20:15:30.123" in page.evaluate(chart + ".getOption().title[0].subtext")
    page.screenshot(path="/tmp/gimbal-calendar-desktop.png", full_page=True)
    for width in [390, 320]:
        page.set_viewport_size({"width": width, "height": 844})
        page.wait_for_timeout(200)
        assert not page.evaluate("document.documentElement.scrollWidth > innerWidth")
    page.screenshot(path="/tmp/gimbal-calendar-mobile.png", full_page=True)
    assert errors == []
    page.close()


def test_history_metrics_zoom_and_exports(browser, artifact, tmp_path):
    page = browser.new_page(viewport={"width": 1440, "height": 1000}, accept_downloads=True)
    errors = []
    page.on("pageerror", lambda error: errors.append(str(error)))
    page.goto(artifact.as_uri())
    assert page.locator(".metric strong").all_text_contents()[:4] == ["78", "1.846", "0.633", "1.230"]
    assert page.locator("#fact-commands").inner_text() == "0 条"
    assert page.locator("#fact-mode").inner_text() == "6 视觉"
    assert page.evaluate("echarts.getInstanceByDom(document.getElementById('chart')).getOption().yAxis[0].max") < 2
    assert page.evaluate("""() => {
      const c = document.querySelector('canvas');
      const pixels = c.getContext('2d').getImageData(0, 0, c.width, c.height).data;
      let teal = 0;
      for (let i = 0; i < pixels.length; i += 4)
        if (pixels[i] < 80 && pixels[i+1] > 90 && pixels[i+1] < 160 && pixels[i+2] < 160) teal++;
      return teal > 100;
    }""")
    page.screenshot(path="/tmp/gimbal-scope-desktop.png", full_page=True)
    for selector, suffix in [("#export-csv", ".csv"), ("#export-png", ".png"), ("#save-json", ".json")]:
        with page.expect_download() as info:
            page.locator(selector).click()
        target = tmp_path / ("export" + suffix)
        info.value.save_as(target)
        assert target.stat().st_size > 100
    csv_rows = list(csv.DictReader(io.StringIO((tmp_path / "export.csv").read_text())))
    assert len([row for row in csv_rows if row["kind"] == "feedback"]) == 78
    assert not any(row["kind"] == "target" for row in csv_rows)
    page.select_option("#preset-select", "disabled")
    assert page.locator("#metric-count").inner_text() == "204"
    assert page.locator("#metric-peak").inner_text() == "0.000"
    page.select_option("#preset-select", "all")
    page.evaluate("echarts.getInstanceByDom(document.getElementById('chart')).dispatchAction({type:'dataZoom',startValue:13,endValue:17})")
    assert page.locator("#metric-count").inner_text() == "78"
    page.locator('[data-axis="yaw"]').click()
    assert page.locator("#chart-title").inner_text() == "Yaw 偏航角"
    page.locator('[data-axis="pitch"]').click()
    page.select_option("#source-select", "earlier")
    assert int(page.locator("#fact-commands").inner_text().split()[0]) == 1924
    page.locator("#file-input").set_input_files(str(tmp_path / "export.json"))
    page.wait_for_function("document.querySelector('#metric-count').textContent === '78'")
    previous_source = page.locator("#source-select").input_value()
    page.locator("#file-input").set_input_files({"name": "broken.json", "mimeType": "application/json", "buffer": b'{"rows":{}}'})
    page.wait_for_function("!document.querySelector('#toast').hidden")
    assert page.locator("#source-select").input_value() == previous_source
    assert "无法打开记录" in page.locator("#toast").inner_text()
    page.locator("#events button").filter(has_text="模式").click()
    assert 17 < float(page.locator("#range-start").input_value()) < 19
    for width in [390, 320]:
        page.set_viewport_size({"width": width, "height": 844})
        page.wait_for_timeout(150)
        assert not page.evaluate("document.documentElement.scrollWidth > innerWidth")
    page.set_viewport_size({"width": 390, "height": 844})
    page.select_option("#preset-select", "post")
    page.screenshot(path="/tmp/gimbal-scope-mobile.png", full_page=True)
    assert errors == []
    page.close()


def test_yaw_wrap_and_missing_feedback(browser, artifact):
    page = browser.new_page()
    page.goto(artifact.as_uri())
    record = {"rows": {"feedback": [[0, 1, 0, math.radians(179), 0, 6],
                                     [0.05, 2, 50, math.radians(-179), 0, 6],
                                     [1, 3, 1000, math.radians(-178), 0, 6]],
                       "forwarded": [[0.05, math.radians(-179), 0]]}}
    page.locator("#file-input").set_input_files({"name": "wrap.json", "mimeType": "application/json", "buffer": json.dumps(record).encode()})
    page.wait_for_function("document.querySelector('#metric-count').textContent === '3'")
    page.locator('[data-axis="yaw"]').click()
    assert page.locator("#metric-peak").inner_text() == "3.000"
    assert page.locator("#metric-step").inner_text() == "2.000"
    assert page.locator("#metric-gap").inner_text() == "950.0"
    data = page.evaluate("echarts.getInstanceByDom(document.getElementById('chart')).getOption().series[0].data")
    assert any(point[1] is None for point in data)
    page.close()


@pytest.fixture
def isolated_server(monkeypatch):
    pytest.importorskip("rclpy")
    monkeypatch.setenv("ROS_DOMAIN_ID", "195")
    monkeypatch.setenv("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
    monkeypatch.setenv("CYCLONEDDS_URI", f"file://{ROOT}/tools/handeye_calibration/cyclonedds_wsl_local.xml")
    process = subprocess.Popen(["/usr/bin/python3", str(HERE / "serve_scope.py"), "--port", "0"],
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, env=os.environ.copy())
    try:
        line = process.stdout.readline()
        assert "http://localhost:" in line, line
        url = line.split()[2]
        for _ in range(50):
            with urllib.request.urlopen(url + "/api/live", timeout=2) as response:
                status = json.load(response)
            if status["ros_ready"]:
                break
            time.sleep(0.1)
        assert status["ros_ready"], status
        yield url
    finally:
        process.send_signal(signal.SIGINT)
        try:
            process.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.communicate()


def test_readonly_live_receives_pauses_and_goes_stale(browser, isolated_server):
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from auto_aim_interfaces.msg import AimCommand, GimbalFeedback
    from std_msgs.msg import Bool

    context = Context()
    rclpy.init(context=context)
    node = Node("scope_isolated_test_source", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    feedback = node.create_publisher(GimbalFeedback, "/serial_bridge/feedback", qos_profile_sensor_data)
    targets = node.create_publisher(AimCommand, "/serial_bridge/aim_command", qos_profile_sensor_data)
    active = node.create_publisher(Bool, "/serial_bridge/command_active", qos_profile_sensor_data)
    stop = threading.Event()
    page = browser.new_page()
    thread = None
    try:
        page.goto(isolated_server)
        page.select_option("#source-select", "live")
        page.wait_for_function("document.querySelector('#connection-text').textContent === '等待反馈'")
        assert page.locator("#empty-state").is_visible()
        deadline = time.monotonic() + 5
        while feedback.get_subscription_count() < 1 and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        assert feedback.get_subscription_count() == 1

        def publish():
            sequence = 0
            while not stop.wait(0.05):
                sequence += 1
                feedback.publish(GimbalFeedback(sequence=sequence, mcu_timestamp_ms=sequence * 50,
                                                yaw=0.1, pitch=0.01 * (-1) ** sequence, mode=6))
                targets.publish(AimCommand(valid=False, yaw=0.0, pitch=0.0))
                targets.publish(AimCommand(valid=True, yaw=0.1, pitch=0.0))
                active.publish(Bool(data=True))

        thread = threading.Thread(target=publish)
        thread.start()
        page.wait_for_function("Number(document.querySelector('#metric-count').textContent) >= 6")
        assert not page.locator("#empty-state").is_visible()
        assert "实时" in page.locator("#connection-text").inner_text()
        page.locator("#pause-button").click()
        paused_count = page.locator("#metric-count").inner_text()
        page.wait_for_timeout(500)
        assert page.locator("#metric-count").inner_text() == paused_count
        page.locator("#pause-button").click()
        page.wait_for_function("Number(document.querySelector('#metric-count').textContent) > " + paused_count)
        stop.set()
        thread.join(timeout=2)
        page.wait_for_function("document.querySelector('#connection-text').textContent === '等待反馈'")
        with urllib.request.urlopen(isolated_server + "/api/live") as response:
            events = json.load(response)["events"]
        commands = [event[2] for event in events if event[1] == "forwarded"]
        assert commands and all(row[1] == 0.1 for row in commands)
        assert all(info.node_name != "gimbal_scope_observer"
                   for info in node.get_publishers_info_by_topic("/serial_bridge/aim_command"))
        page.screenshot(path="/tmp/gimbal-scope-live.png", full_page=True)
    finally:
        stop.set()
        if thread:
            thread.join(timeout=2)
        page.close()
        executor.shutdown()
        node.destroy_node()
        context.shutdown()
