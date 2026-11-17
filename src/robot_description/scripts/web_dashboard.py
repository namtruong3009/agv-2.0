#!/usr/bin/env python3
import math
import json
import time
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformListener

TYPE_MAP = {
    'nav_msgs/msg/Odometry': Odometry,
    'geometry_msgs/msg/Twist': Twist,
    'sensor_msgs/msg/Imu': Imu,
    'geometry_msgs/msg/PoseStamped': PoseStamped,
    'geometry_msgs/msg/PoseWithCovarianceStamped': PoseWithCovarianceStamped,
}

HTML_PAGE = """<!DOCTYPE html>
<html lang="vi">
<head>
  <meta charset="UTF-8">
  <title>ROS 2 Control & EKF Localization Dashboard</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body { background: #11111b; color: #cdd6f4; font-family: 'Segoe UI', sans-serif; padding: 12px; }
    .header { display: flex; justify-content: space-between; align-items: center; background: #1e1e2e; padding: 10px 16px; border-radius: 10px; margin-bottom: 10px; border: 1px solid #313244; }
    .header h2 { color: #89b4fa; font-size: 17px; }

    .conn-badge { padding: 6px 12px; border-radius: 6px; font-weight: bold; font-size: 13px; display: inline-block; }
    .conn-ok { background: rgba(166, 227, 161, 0.15); color: #a6e3a1; border: 1px solid #a6e3a1; }
    .conn-lost { background: rgba(243, 139, 168, 0.2); color: #f38ba8; border: 1px solid #f38ba8; animation: blink 1s infinite; }
    @keyframes blink { 50% { opacity: 0.4; } }

    .alert-banner { display: none; background: #3b1c26; color: #f38ba8; border: 1px solid #f38ba8; padding: 10px 16px; border-radius: 8px; margin-bottom: 10px; font-weight: bold; font-size: 14px; }

    .tabs { display: flex; gap: 8px; margin-bottom: 10px; border-bottom: 2px solid #313244; padding-bottom: 8px; }
    .tab-btn { background: #1e1e2e; color: #a6adc8; border: 1px solid #45475a; padding: 8px 16px; border-radius: 8px; cursor: pointer; font-weight: bold; font-size: 13px; }
    .tab-btn.active { background: #89b4fa; color: #11111b; border-color: #89b4fa; }

    .control-bar { display: flex; flex-wrap: wrap; gap: 10px; align-items: center; background: #181825; padding: 10px 14px; border-radius: 8px; margin-bottom: 10px; border: 1px solid #45475a; font-size: 13px; }
    select, button { padding: 6px 10px; border-radius: 6px; border: 1px solid #45475a; background: #313244; color: #cdd6f4; font-size: 13px; }
    .btn { background: #a6e3a1; color: #11111b; border: none; font-weight: bold; cursor: pointer; }
    .btn-clear { background: #f38ba8; color: #11111b; border: none; font-weight: bold; cursor: pointer; }

    .kpi-row { display: grid; grid-template-columns: repeat(4, 1fr); gap: 10px; margin-bottom: 10px; }
    .kpi { background: #1e1e2e; padding: 9px 12px; border-radius: 8px; border-left: 4px solid #89b4fa; font-family: monospace; font-size: 12.5px; }
    .kpi-title { color: #a6adc8; font-size: 11px; margin-bottom: 3px; font-family: sans-serif; }

    .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; }
    .card { background: #1e1e2e; padding: 10px; border-radius: 10px; border: 1px solid #313244; height: 290px; display: flex; flex-direction: column; }
    .card-full { grid-column: span 2; height: 290px; }
    .card h3 { font-size: 13px; color: #f9e2af; margin-bottom: 4px; display: flex; justify-content: space-between; }
    .legend { display: flex; flex-wrap: wrap; gap: 12px; font-size: 12px; margin-bottom: 4px; }
    .legend span { display: inline-flex; align-items: center; gap: 5px; cursor: pointer; user-select: none; }
    .dot { width: 10px; height: 10px; border-radius: 2px; display: inline-block; }
    canvas { flex: 1; width: 100%; height: 100%; background: #181825; border-radius: 6px; border: 1px solid #313244; }
    .tab-panel { display: none; }
    .tab-panel.active { display: block; }
    .sub-tag { font-size: 11px; color: #a6adc8; font-weight: normal; font-family: monospace; }
  </style>
</head>
<body>

  <div class="header">
    <h2>ROS 2 REAL-TIME ALGORITHM MONITOR (PID & EKF & SLAM)</h2>
    <div style="display:flex; align-items:center; gap:10px;">
      <span id="connBadge" class="conn-badge conn-lost">● ĐANG KIỂM TRA MÔ HÌNH...</span>
      <button class="btn" onclick="refreshTopics()">↻ Quét Topic</button>
      <button class="btn" onclick="exportCSV()">⬇ Xuất CSV</button>
      <button class="btn-clear" onclick="clearAll()">✖ Xóa đồ thị</button>
    </div>
  </div>

  <div id="alertBanner" class="alert-banner">
    ⚠ KHÔNG NHẬN ĐƯỢC TÍN HIỆU TỪ MÔ HÌNH ROBOT! (Gazebo đã tắt, tạm dừng hoặc chưa chạy file launch). Đồ thị đang tự động tạm dừng để tránh vẽ dữ liệu ảo.
  </div>

  <div class="tabs">
    <button class="tab-btn active" onclick="switchTab('tab_pid', this)">1. Bộ điều khiển PID (Vận tốc & Heading)</button>
    <button class="tab-btn" onclick="switchTab('tab_ekf', this)">2. Bộ lọc EKF (5 Đồ thị So sánh)</button>
    <button class="tab-btn" onclick="switchTab('tab_slam', this)">3. SLAM & Quy hoạch A* (Quỹ đạo 2D)</button>
    <button class="tab-btn" onclick="switchTab('tab_custom', this)">4. Tự chọn Topic bất kỳ</button>
  </div>

  <!-- ================= TAB 1: PID CONTROLLER ================= -->
  <div id="tab_pid" class="tab-panel active">
    <div class="control-bar">
      <strong style="color:#f9e2af;">Cấu hình kênh đo PID:</strong>
      <label>Topic Đặt (Setpoint):</label>
      <select id="pidRefTopic" onchange="updatePidLabels()"></select>
      <label>Topic Phản hồi (Feedback):</label>
      <select id="pidFbTopic" onchange="updatePidLabels()"></select>
      <label>Nguồn Heading Đặt:</label>
      <select id="pidYawRefMode">
        <option value="integrated">Tích phân từ vận tốc góc đặt (∫w_ref dt)</option>
        <option value="goal_pose">Hướng theo đích /goal_pose</option>
        <option value="ref_topic">Lấy trường yaw của Topic Đặt</option>
      </select>
      <button class="btn" style="background:#89b4fa;" onclick="syncYawRef()">Đồng bộ Góc Đặt = Odom</button>
    </div>

    <div class="kpi-row">
      <div class="kpi" style="border-color:#f9e2af;">
        <div class="kpi-title">VẬN TỐC DÀI (ĐẶT vs ODOM)</div>
        <div id="kpi_pid_v">Ref: 0.00 | Fb: 0.00 m/s</div>
      </div>
      <div class="kpi" style="border-color:#a6e3a1;">
        <div class="kpi-title">HEADING / YAW (ĐẶT vs ODOM)</div>
        <div id="kpi_pid_yaw">Ref: 0.0° | Fb: 0.0°</div>
      </div>
      <div class="kpi" style="border-color:#89b4fa;">
        <div class="kpi-title">VẬN TỐC GÓC (ĐẶT vs ODOM)</div>
        <div id="kpi_pid_w">Ref: 0.00 | Fb: 0.00 rad/s</div>
      </div>
      <div class="kpi" style="border-color:#f38ba8;">
        <div class="kpi-title">SAI SỐ TỨC THỜI (|e_v| / |e_yaw|)</div>
        <div id="kpi_pid_err">e_v: 0.000 m/s | e_yaw: 0.0°</div>
      </div>
    </div>

    <div class="grid">
      <div class="card">
        <h3><span>1. Đáp ứng Vận tốc dài v (m/s): Đặt vs Thực tế</span></h3>
        <div class="legend" id="leg_pid_v"></div>
        <canvas id="c_pid_v"></canvas>
      </div>
      <div class="card">
        <h3><span>2. Đáp ứng Góc hướng Heading / Yaw (Độ): Đặt vs Thực tế</span></h3>
        <div class="legend" id="leg_pid_yaw"></div>
        <canvas id="c_pid_yaw"></canvas>
      </div>
      <div class="card">
        <h3><span>3. Đáp ứng Vận tốc góc ω (rad/s): Đặt vs Thực tế</span></h3>
        <div class="legend" id="leg_pid_w"></div>
        <canvas id="c_pid_w"></canvas>
      </div>
      <div class="card">
        <h3><span>4. Đồ thị Sai số bám PID theo thời gian (e_v & e_ω)</span></h3>
        <div class="legend" id="leg_pid_err"></div>
        <canvas id="c_pid_err"></canvas>
      </div>
    </div>
  </div>

  <!-- ================= TAB 2: EKF LOCALIZATION (5 PLOTS) ================= -->
  <div id="tab_ekf" class="tab-panel">
    <div class="control-bar">
      <strong style="color:#f9e2af;">Cấu hình các Topic đầu vào/đầu ra EKF:</strong>
      <label>Topic Odom:</label>
      <select id="ekfOdomTopic" onchange="updateEkfLabels()"></select>
      <label>Topic IMU:</label>
      <select id="ekfImuTopic" onchange="updateEkfLabels()"></select>
      <label>Topic EKF:</label>
      <select id="ekfFilteredTopic" onchange="updateEkfLabels()"></select>
    </div>

    <div class="kpi-row">
      <div class="kpi" style="border-color:#f38ba8;">
        <div class="kpi-title">VỊ TRÍ (X, Y) ODOM vs EKF</div>
        <div id="kpi_ekf_pos">Odom:(0.00, 0.00) | EKF:(0.00, 0.00)</div>
      </div>
      <div class="kpi" style="border-color:#89b4fa;">
        <div class="kpi-title">YAW (ODOM vs EKF) & Vy EKF</div>
        <div id="kpi_ekf_yaw">Yaw: 0.0° vs 0.0° | Vy: 0.00 m/s</div>
      </div>
      <div class="kpi" style="border-color:#cba6f7;">
        <div class="kpi-title">PHƯƠNG SAI EKF (pose.cov[0],[7],[35] | twist.cov[0])</div>
        <div id="kpi_ekf_cov">P_x:0.000 P_y:0.000 P_yaw:0.000 P_vx:0.000</div>
      </div>
      <div class="kpi" style="border-color:#a6e3a1;">
        <div class="kpi-title">TẦN SỐ DỮ LIỆU NHẬN (Hz)</div>
        <div id="kpi_ekf_hz">Odom: 0Hz | IMU: 0Hz | EKF: 0Hz</div>
      </div>
    </div>

    <div class="grid">
      <div class="card">
        <h3>
          <span>1. Vị trí X theo thời gian (m): X Odom vs X EKF</span>
          <span class="sub-tag">pose.pose.position.x</span>
        </h3>
        <div class="legend" id="leg_ekf_x"></div>
        <canvas id="c_ekf_x"></canvas>
      </div>

      <div class="card">
        <h3>
          <span>2. Vị trí Y theo thời gian (m): Y Odom vs Y EKF</span>
          <span class="sub-tag">pose.pose.position.y</span>
        </h3>
        <div class="legend" id="leg_ekf_y"></div>
        <canvas id="c_ekf_y"></canvas>
      </div>

      <div class="card">
        <h3>
          <span>3. Góc Heading / Yaw (Độ): Yaw Odom vs Yaw EKF</span>
          <span class="sub-tag">pose.pose.orientation</span>
        </h3>
        <div class="legend" id="leg_ekf_yaw"></div>
        <canvas id="c_ekf_yaw"></canvas>
      </div>

      <div class="card">
        <h3>
          <span>4. Vận tốc dài Vx (m/s): Vx Odom vs Vx EKF</span>
          <span class="sub-tag">twist.twist.linear.x</span>
        </h3>
        <div class="legend" id="leg_ekf_vx"></div>
        <canvas id="c_ekf_vx"></canvas>
      </div>

      <div class="card card-full">
        <h3>
          <span>5. Vận tốc góc ωz (rad/s): ωz IMU vs ωz EKF</span>
          <span class="sub-tag">angular_velocity.z vs twist.twist.angular.z</span>
        </h3>
        <div class="legend" id="leg_ekf_wz"></div>
        <canvas id="c_ekf_wz"></canvas>
      </div>
    </div>
  </div>

  <!-- ================= TAB 3: SLAM & A* ================= -->
  <div id="tab_slam" class="tab-panel">
    <div class="card" style="height: 520px;">
      <h3><span>Bản đồ Quỹ đạo 2D (X - Y): Raw Odom vs EKF vs SLAM vs Đường quy hoạch A* (/plan)</span></h3>
      <div class="legend" id="leg_traj"></div>
      <canvas id="c_traj"></canvas>
    </div>
  </div>

  <!-- ================= TAB 4: CUSTOM PLOTTER ================= -->
  <div id="tab_custom" class="tab-panel">
    <div class="control-bar">
      <strong style="color:#f9e2af;">+ Ghép thêm đường vào đồ thị tùy chọn:</strong>
      <label>Topic:</label>
      <select id="customTopicSelect"></select>
      <label>Trường (Field):</label>
      <select id="customFieldSelect">
        <option value="v">linear.x / vận tốc dài Vx (m/s)</option>
        <option value="vy">linear.y / vận tốc ngang Vy (m/s)</option>
        <option value="w">angular.z / vận tốc góc ωz (rad/s)</option>
        <option value="yaw">orientation.yaw / heading (độ)</option>
        <option value="x">position.x (m)</option>
        <option value="y">position.y (m)</option>
      </select>
      <button class="btn" style="background:#89b4fa;" onclick="addCustomSeries()">+ Vẽ thêm</button>
      <button class="btn-clear" onclick="resetCustomSeries()">Xóa hết đường</button>
    </div>
    <div class="card" style="height: 460px;">
      <h3><span>Đồ thị So sánh Tùy chọn</span></h3>
      <div class="legend" id="leg_custom"></div>
      <canvas id="c_custom"></canvas>
    </div>
  </div>

<script>
  const maxPts = 160;
  const palette = ['#f9e2af', '#89b4fa', '#a6e3a1', '#f38ba8', '#cba6f7', '#fab387'];
  let activeTab = 'tab_pid';
  let timeHistory = [];
  let logCSV = [];

  // Dữ liệu Tab 1: PID
  let pidVSeries = [
    { label: 'Vận tốc đặt (v_ref)', color: '#f9e2af', data: [], show: true },
    { label: 'Vận tốc thực (v_odom)', color: '#89b4fa', data: [], show: true }
  ];
  let pidYawSeries = [
    { label: 'Heading đặt (yaw_ref)', color: '#f9e2af', data: [], show: true },
    { label: 'Heading thực (yaw_odom)', color: '#a6e3a1', data: [], show: true }
  ];
  let pidWSeries = [
    { label: 'Vận tốc góc đặt (w_ref)', color: '#f9e2af', data: [], show: true },
    { label: 'Vận tốc góc thực (w_odom)', color: '#cba6f7', data: [], show: true }
  ];
  let pidErrSeries = [
    { label: 'Sai số vận tốc dài e_v (m/s)', color: '#f38ba8', data: [], show: true },
    { label: 'Sai số vận tốc góc e_w (rad/s)', color: '#fab387', data: [], show: true }
  ];

  // Dữ liệu Tab 2: 5 Đồ thị EKF
  let ekfXSeries = [
    { label: 'X Odom (/odom)', color: '#f38ba8', data: [], show: true },
    { label: 'X EKF (/odometry/filtered)', color: '#89b4fa', data: [], show: true }
  ];
  let ekfYSeries = [
    { label: 'Y Odom (/odom)', color: '#f38ba8', data: [], show: true },
    { label: 'Y EKF (/odometry/filtered)', color: '#a6e3a1', data: [], show: true }
  ];
  let ekfYawSeries = [
    { label: 'Yaw Odom (/odom)', color: '#f38ba8', data: [], show: true },
    { label: 'Yaw EKF (/odometry/filtered)', color: '#f9e2af', data: [], show: true }
  ];
  let ekfVxSeries = [
    { label: 'Vx Odom (/odom)', color: '#f38ba8', data: [], show: true },
    { label: 'Vx EKF (/odometry/filtered)', color: '#89b4fa', data: [], show: true }
  ];
  let ekfWzSeries = [
    { label: 'ωz IMU (/imu)', color: '#cba6f7', data: [], show: true },
    { label: 'ωz EKF (/odometry/filtered)', color: '#a6e3a1', data: [], show: true }
  ];

  // Dữ liệu Tab 3: Quỹ đạo 2D
  let trajData = [
    { label: '/odom', color: '#f38ba8', pts: [], show: true },
    { label: '/odometry/filtered', color: '#89b4fa', pts: [], show: true },
    { label: 'TF map→base', color: '#a6e3a1', pts: [], show: true },
    { label: '/plan (A*)', color: '#f9e2af', pts: [], show: true }
  ];

  // Dữ liệu Tab 4: Tùy chọn
  let customSeries = [
    { label: '/cmd_vel [v]', topic: '/cmd_vel', field: 'v', color: '#f9e2af', data: [], show: true },
    { label: '/odom [v]', topic: '/odom', field: 'v', color: '#89b4fa', data: [], show: true }
  ];

  function switchTab(tabId, btn) {
    activeTab = tabId;
    document.querySelectorAll('.tab-panel').forEach(p => p.classList.remove('active'));
    document.querySelectorAll('.tab-btn').forEach(b => b.classList.remove('active'));
    document.getElementById(tabId).classList.add('active');
    btn.classList.add('active');
    redrawAll();
  }

  function bindLegend(elId, arr) {
    const el = document.getElementById(elId);
    if (!el) return;
    el.innerHTML = '';
    arr.forEach(s => {
      const sp = document.createElement('span');
      sp.style.opacity = s.show ? '1.0' : '0.35';
      sp.innerHTML = `<i class="dot" style="background:${s.color}"></i>${s.label}`;
      sp.onclick = () => { s.show = !s.show; bindLegend(elId, arr); redrawAll(); };
      el.appendChild(sp);
    });
  }

  function renderAllLegends() {
    bindLegend('leg_pid_v', pidVSeries);
    bindLegend('leg_pid_yaw', pidYawSeries);
    bindLegend('leg_pid_w', pidWSeries);
    bindLegend('leg_pid_err', pidErrSeries);

    bindLegend('leg_ekf_x', ekfXSeries);
    bindLegend('leg_ekf_y', ekfYSeries);
    bindLegend('leg_ekf_yaw', ekfYawSeries);
    bindLegend('leg_ekf_vx', ekfVxSeries);
    bindLegend('leg_ekf_wz', ekfWzSeries);

    bindLegend('leg_traj', trajData);
    bindLegend('leg_custom', customSeries);
  }
  renderAllLegends();

  function updatePidLabels() {
    const refT = document.getElementById('pidRefTopic').value || '/cmd_vel';
    const fbT  = document.getElementById('pidFbTopic').value || '/odom';
    pidVSeries[0].label = `Đặt: ${refT} [v]`;
    pidVSeries[1].label = `Thực: ${fbT} [v]`;
    pidYawSeries[1].label = `Thực: ${fbT} [yaw]`;
    pidWSeries[0].label = `Đặt: ${refT} [w]`;
    pidWSeries[1].label = `Thực: ${fbT} [w]`;
    renderAllLegends();
  }

  function updateEkfLabels() {
    const odomT = document.getElementById('ekfOdomTopic').value || '/odom';
    const imuT  = document.getElementById('ekfImuTopic').value || '/imu';
    const ekfT  = document.getElementById('ekfFilteredTopic').value || '/odometry/filtered';

    ekfXSeries[0].label = `X Odom (${odomT})`;
    ekfXSeries[1].label = `X EKF (${ekfT})`;
    ekfYSeries[0].label = `Y Odom (${odomT})`;
    ekfYSeries[1].label = `Y EKF (${ekfT})`;
    ekfYawSeries[0].label = `Yaw Odom (${odomT})`;
    ekfYawSeries[1].label = `Yaw EKF (${ekfT})`;
    ekfVxSeries[0].label = `Vx Odom (${odomT})`;
    ekfVxSeries[1].label = `Vx EKF (${ekfT})`;
    ekfWzSeries[0].label = `ωz IMU (${imuT})`;
    ekfWzSeries[1].label = `ωz EKF (${ekfT})`;
    renderAllLegends();
  }

  async function syncYawRef() {
    await fetch('/api/sync_yaw');
  }

  async function refreshTopics() {
    try {
      const res = await fetch('/api/topics');
      const topics = await res.json();
      const selectors = [
        { id: 'pidRefTopic', def: '/cmd_vel' },
        { id: 'pidFbTopic', def: '/odom' },
        { id: 'ekfOdomTopic', def: '/odom' },
        { id: 'ekfImuTopic', def: '/imu' },
        { id: 'ekfFilteredTopic', def: '/odometry/filtered' },
        { id: 'customTopicSelect', def: '/odom' }
      ];
      selectors.forEach(selObj => {
        const sel = document.getElementById(selObj.id);
        if (!sel) return;
        const cur = sel.value || selObj.def;
        sel.innerHTML = '';
        topics.forEach(t => {
          const opt = document.createElement('option');
          opt.value = t.name;
          const icon = t.hz > 0 ? `🟢 (${t.hz.toFixed(1)}Hz)` : `⚪ (0Hz)`;
          opt.text = `${icon} ${t.name} [${t.type.split('/').pop()}]`;
          sel.appendChild(opt);
        });
        if ([...sel.options].some(o => o.value === cur)) sel.value = cur;
      });
      updatePidLabels();
      updateEkfLabels();
    } catch (e) {}
  }

  function addCustomSeries() {
    const topic = document.getElementById('customTopicSelect').value;
    const field = document.getElementById('customFieldSelect').value;
    if (!topic) return;
    const label = `${topic} [${field}]`;
    if (customSeries.some(s => s.label === label)) return;
    const color = palette[customSeries.length % palette.length];
    customSeries.push({ label, topic, field, color, data: new Array(timeHistory.length).fill(0), show: true });
    renderAllLegends();
  }

  function resetCustomSeries() {
    customSeries = [];
    renderAllLegends();
    redrawAll();
  }

  function drawLinePlot(canvasId, seriesArr, dashedFirst=false) {
    const canvas = document.getElementById(canvasId);
    if (!canvas || canvas.clientWidth === 0) return;
    const ctx = canvas.getContext('2d');
    canvas.width = canvas.clientWidth;
    canvas.height = canvas.clientHeight;
    const W = canvas.width, H = canvas.height;
    const padL = 50, padR = 12, padT = 12, padB = 22;
    ctx.clearRect(0, 0, W, H);

    let minY = -0.5, maxY = 0.5;
    let allVals = [];
    seriesArr.forEach(s => { if (s.show) allVals.push(...s.data); });
    if (allVals.length > 0) {
      minY = Math.min(...allVals);
      maxY = Math.max(...allVals);
      if (Math.abs(maxY - minY) < 0.2) { maxY += 0.5; minY -= 0.5; }
      else { const m = (maxY - minY) * 0.12; maxY += m; minY -= m; }
    }

    ctx.strokeStyle = '#313244';
    ctx.fillStyle = '#a6adc8';
    ctx.font = '11px monospace';
    for (let i = 0; i <= 4; i++) {
      const y = padT + (H - padT - padB) * (i / 4);
      const val = maxY - (maxY - minY) * (i / 4);
      ctx.beginPath(); ctx.moveTo(padL, y); ctx.lineTo(W - padR, y); ctx.stroke();
      ctx.fillText(val.toFixed(2), 4, y + 4);
    }

    seriesArr.forEach((s, idx) => {
      if (!s.show || s.data.length < 2) return;
      ctx.strokeStyle = s.color;
      ctx.lineWidth = idx === 0 ? 1.8 : 2.2;
      if (dashedFirst && idx === 0) ctx.setLineDash([6, 3]); else ctx.setLineDash([]);
      ctx.beginPath();
      for (let i = 0; i < s.data.length; i++) {
        const x = padL + ((W - padL - padR) * i) / (maxPts - 1);
        const y = padT + (H - padT - padB) * (1 - (s.data[i] - minY) / (maxY - minY));
        if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
      }
      ctx.stroke();
      ctx.setLineDash([]);
    });
  }

  function drawTrajPlot(canvasId) {
    const canvas = document.getElementById(canvasId);
    if (!canvas || canvas.clientWidth === 0) return;
    const ctx = canvas.getContext('2d');
    canvas.width = canvas.clientWidth;
    canvas.height = canvas.clientHeight;
    const W = canvas.width, H = canvas.height;
    const pad = 36;
    ctx.clearRect(0, 0, W, H);

    let allX = [-2, 2], allY = [-2, 2];
    trajData.forEach(t => { if (t.show) t.pts.forEach(p => { allX.push(p.x); allY.push(p.y); }); });
    const minX = Math.min(...allX) - 0.5, maxX = Math.max(...allX) + 0.5;
    const minY = Math.min(...allY) - 0.5, maxY = Math.max(...allY) + 0.5;

    ctx.strokeStyle = '#313244';
    ctx.fillStyle = '#a6adc8';
    ctx.font = '11px monospace';
    for (let i = 0; i <= 4; i++) {
      const y = pad + (H - 2 * pad) * (i / 4);
      ctx.beginPath(); ctx.moveTo(pad, y); ctx.lineTo(W - pad, y); ctx.stroke();
      ctx.fillText((maxY - (maxY - minY) * (i / 4)).toFixed(1), 4, y + 4);

      const x = pad + (W - 2 * pad) * (i / 4);
      ctx.beginPath(); ctx.moveTo(x, pad); ctx.lineTo(x, H - pad); ctx.stroke();
      ctx.fillText((minX + (maxX - minX) * (i / 4)).toFixed(1), x - 10, H - 8);
    }

    trajData.forEach((t, idx) => {
      if (!t.show || t.pts.length === 0) return;
      ctx.strokeStyle = t.color;
      ctx.lineWidth = 2;
      if (idx === 3) ctx.setLineDash([5, 4]); else ctx.setLineDash([]);
      ctx.beginPath();
      t.pts.forEach((p, i) => {
        const px = pad + ((p.x - minX) / (maxX - minX)) * (W - 2 * pad);
        const py = pad + (1 - (p.y - minY) / (maxY - minY)) * (H - 2 * pad);
        if (i === 0) ctx.moveTo(px, py); else ctx.lineTo(px, py);
      });
      ctx.stroke();
      ctx.setLineDash([]);
    });
  }

  function redrawAll() {
    if (activeTab === 'tab_pid') {
      drawLinePlot('c_pid_v', pidVSeries, true);
      drawLinePlot('c_pid_yaw', pidYawSeries, true);
      drawLinePlot('c_pid_w', pidWSeries, true);
      drawLinePlot('c_pid_err', pidErrSeries, false);
    } else if (activeTab === 'tab_ekf') {
      drawLinePlot('c_ekf_x', ekfXSeries, true);
      drawLinePlot('c_ekf_y', ekfYSeries, true);
      drawLinePlot('c_ekf_yaw', ekfYawSeries, true);
      drawLinePlot('c_ekf_vx', ekfVxSeries, true);
      drawLinePlot('c_ekf_wz', ekfWzSeries, true);
    } else if (activeTab === 'tab_slam') {
      drawTrajPlot('c_traj');
    } else if (activeTab === 'tab_custom') {
      drawLinePlot('c_custom', customSeries, false);
    }
  }

  function pushVal(arr, val) {
    arr.push(val);
    if (arr.length > maxPts) arr.shift();
  }

  async function loop() {
    try {
      const res = await fetch('/api/data');
      const d = await res.json();

      const badge = document.getElementById('connBadge');
      const banner = document.getElementById('alertBanner');

      if (!d.model_connected) {
        badge.className = 'conn-badge conn-lost';
        badge.innerText = `⚠ MẤT KẾT NỐI MÔ HÌNH (0 Hz)`;
        banner.style.display = 'block';
        return;
      }

      badge.className = 'conn-badge conn-ok';
      badge.innerText = `● ĐÃ KẾT NỐI MÔ HÌNH (Odom: ${d.hz.odom.toFixed(1)}Hz | EKF: ${d.hz.ekf.toFixed(1)}Hz)`;
      banner.style.display = 'none';

      const tp = d.topics;
      const defaultObj = {x:0, y:0, yaw:0, v:0, vy:0, w:0, cov_x:0, cov_y:0, cov_yaw:0, cov_vx:0};

      // --- 1. XỬ LÝ DỮ LIỆU TAB PID ---
      const refTopic = document.getElementById('pidRefTopic').value || '/cmd_vel';
      const fbTopic  = document.getElementById('pidFbTopic').value || '/odom';
      const yawMode  = document.getElementById('pidYawRefMode').value;

      const refObj = tp[refTopic] || defaultObj;
      const fbObj  = tp[fbTopic]  || defaultObj;

      let yawRef = d.integrated_yaw_ref;
      if (yawMode === 'goal_pose') yawRef = d.goal_yaw_ref;
      else if (yawMode === 'ref_topic') yawRef = refObj.yaw;

      const ev = refObj.v - fbObj.v;
      const ew = refObj.w - fbObj.w;
      let eyaw = yawRef - fbObj.yaw;
      while (eyaw > 180) eyaw -= 360;
      while (eyaw < -180) eyaw += 360;

      document.getElementById('kpi_pid_v').innerText   = `Đặt: ${refObj.v.toFixed(2)} | Thực: ${fbObj.v.toFixed(2)} m/s`;
      document.getElementById('kpi_pid_yaw').innerText = `Đặt: ${yawRef.toFixed(1)}° | Thực: ${fbObj.yaw.toFixed(1)}°`;
      document.getElementById('kpi_pid_w').innerText   = `Đặt: ${refObj.w.toFixed(2)} | Thực: ${fbObj.w.toFixed(2)} rad/s`;
      document.getElementById('kpi_pid_err').innerText = `e_v: ${ev.toFixed(3)} m/s | e_yaw: ${eyaw.toFixed(1)}°`;

      pushVal(timeHistory, d.time);
      pushVal(pidVSeries[0].data, refObj.v);
      pushVal(pidVSeries[1].data, fbObj.v);
      pushVal(pidYawSeries[0].data, yawRef);
      pushVal(pidYawSeries[1].data, fbObj.yaw);
      pushVal(pidWSeries[0].data, refObj.w);
      pushVal(pidWSeries[1].data, fbObj.w);
      pushVal(pidErrSeries[0].data, ev);
      pushVal(pidErrSeries[1].data, ew);

      // --- 2. XỬ LÝ DỮ LIỆU TAB EKF (5 ĐỒ THỊ) ---
      const ekfOdomT = document.getElementById('ekfOdomTopic').value || '/odom';
      const ekfImuT  = document.getElementById('ekfImuTopic').value || '/imu';
      const ekfFiltT = document.getElementById('ekfFilteredTopic').value || '/odometry/filtered';

      const odomData = tp[ekfOdomT] || defaultObj;
      const imuData  = tp[ekfImuT]  || defaultObj;
      const ekfData  = tp[ekfFiltT] || defaultObj;

      // 1. Vị trí X: X Odom vs X EKF
      pushVal(ekfXSeries[0].data, odomData.x);
      pushVal(ekfXSeries[1].data, ekfData.x);
      // 2. Vị trí Y: Y Odom vs Y EKF
      pushVal(ekfYSeries[0].data, odomData.y);
      pushVal(ekfYSeries[1].data, ekfData.y);
      // 3. Góc Heading/Yaw: Yaw Odom vs Yaw EKF
      pushVal(ekfYawSeries[0].data, odomData.yaw);
      pushVal(ekfYawSeries[1].data, ekfData.yaw);
      // 4. Vận tốc dài: Vx Odom vs Vx EKF
      pushVal(ekfVxSeries[0].data, odomData.v);
      pushVal(ekfVxSeries[1].data, ekfData.v);
      // 5. Vận tốc góc: ωz IMU vs ωz EKF
      pushVal(ekfWzSeries[0].data, imuData.w);
      pushVal(ekfWzSeries[1].data, ekfData.w);

      // Cập nhật KPI của Tab EKF
      document.getElementById('kpi_ekf_pos').innerText =
        `Odom:(${odomData.x.toFixed(2)}, ${odomData.y.toFixed(2)}) | EKF:(${ekfData.x.toFixed(2)}, ${ekfData.y.toFixed(2)})`;
      document.getElementById('kpi_ekf_yaw').innerText =
        `Yaw: ${odomData.yaw.toFixed(1)}° vs ${ekfData.yaw.toFixed(1)}° | Vy: ${ekfData.vy.toFixed(3)} m/s`;
      document.getElementById('kpi_ekf_cov').innerText =
        `cov[0]:${ekfData.cov_x.toExponential(1)} [7]:${ekfData.cov_y.toExponential(1)} [35]:${ekfData.cov_yaw.toExponential(1)}`;
      document.getElementById('kpi_ekf_hz').innerText =
        `Odom: ${d.hz.odom.toFixed(1)}Hz | IMU: ${d.hz.imu.toFixed(1)}Hz | EKF: ${d.hz.ekf.toFixed(1)}Hz`;

      // --- 3. CẬP NHẬT TAB SLAM & CUSTOM ---
      customSeries.forEach(s => {
        const val = (tp[s.topic] && tp[s.topic][s.field] !== undefined) ? tp[s.topic][s.field] : 0;
        pushVal(s.data, val);
      });

      const slam = tp['TF:map->base_footprint'] || defaultObj;
      trajData[0].pts.push({x: odomData.x, y: odomData.y});
      trajData[1].pts.push({x: ekfData.x, y: ekfData.y});
      trajData[2].pts.push({x: slam.x, y: slam.y});
      if (trajData[0].pts.length > 400) {
        trajData[0].pts.shift(); trajData[1].pts.shift(); trajData[2].pts.shift();
      }
      trajData[3].pts = d.plan || [];

      redrawAll();

      // Lưu đầy đủ cả dữ liệu PID và 5 đồ thị EKF vào CSV
      logCSV.push([
        d.time.toFixed(2),
        refObj.v.toFixed(4), odomData.v.toFixed(4), ekfData.v.toFixed(4),
        odomData.x.toFixed(4), ekfData.x.toFixed(4),
        odomData.y.toFixed(4), ekfData.y.toFixed(4),
        yawRef.toFixed(2), odomData.yaw.toFixed(2), ekfData.yaw.toFixed(2),
        refObj.w.toFixed(4), imuData.w.toFixed(4), ekfData.w.toFixed(4),
        ekfData.vy.toFixed(4),
        ekfData.cov_x.toExponential(3), ekfData.cov_y.toExponential(3), ekfData.cov_yaw.toExponential(3), ekfData.cov_vx.toExponential(3)
      ]);
    } catch (e) {
      const badge = document.getElementById('connBadge');
      badge.className = 'conn-badge conn-lost';
      badge.innerText = '⚠ NODE PYTHON ĐÃ DỪNG';
    }
  }

  function clearAll() {
    timeHistory = [];
    logCSV = [];
    [pidVSeries, pidYawSeries, pidWSeries, pidErrSeries,
     ekfXSeries, ekfYSeries, ekfYawSeries, ekfVxSeries, ekfWzSeries, customSeries].forEach(g => g.forEach(s => s.data = []));
    trajData.forEach(t => t.pts = []);
    redrawAll();
  }

  function exportCSV() {
    let csv = "time,v_ref,vx_odom,vx_ekf,x_odom,x_ekf,y_odom,y_ekf,yaw_ref_deg,yaw_odom_deg,yaw_ekf_deg,w_ref,wz_imu,wz_ekf,vy_ekf,pose_cov_0_x,pose_cov_7_y,pose_cov_35_yaw,twist_cov_0_vx\\n";
    logCSV.forEach(r => { csv += r.join(",") + "\\n"; });
    const a = document.createElement('a');
    a.href = URL.createObjectURL(new Blob([csv], { type: 'text/csv' }));
    a.download = 'ros2_pid_ekf_comparison.csv';
    a.click();
  }

  refreshTopics();
  setInterval(refreshTopics, 3000);
  setInterval(loop, 100);
</script>
</body>
</html>
"""

def quat_to_yaw_deg(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))

def empty_topic_dict():
    return {
        'x': 0.0, 'y': 0.0, 'yaw': 0.0,
        'v': 0.0, 'vy': 0.0, 'w': 0.0,
        'cov_x': 0.0, 'cov_y': 0.0, 'cov_yaw': 0.0, 'cov_vx': 0.0
    }

class WebTelemetryNode(Node):
    def __init__(self):
        super().__init__('web_telemetry_node')
        self.start_time = time.time()
        self.subscribed_topics = {}
        self.topic_values = {}
        self.topic_last_time = {}
        self.topic_msg_counts = {}
        self.topic_hz = {}
        self.last_hz_calc = time.time()

        self.integrated_yaw_ref = 0.0
        self.goal_yaw_ref = 0.0
        self.yaw_initialized = False

        for k in ['/cmd_vel', '/odom', '/odometry/filtered', '/imu', 'TF:map->base_footprint']:
            self.topic_values[k] = empty_topic_dict()
            self.topic_last_time[k] = 0.0
            self.topic_msg_counts[k] = 0
            self.topic_hz[k] = 0.0

        self.plan_data = []
        self.create_subscription(Path, '/plan', self.plan_cb, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_timer(0.05, self.update_loop)
        self.create_timer(1.0, self.calc_frequencies)
        self.create_timer(2.0, self.discover_topics)
        self.discover_topics()

    def discover_topics(self):
        for name, types in self.get_topic_names_and_types():
            if name in self.subscribed_topics or not types:
                continue
            msg_type_str = types[0]
            if msg_type_str in TYPE_MAP:
                msg_cls = TYPE_MAP[msg_type_str]
                if name not in self.topic_values:
                    self.topic_values[name] = empty_topic_dict()
                    self.topic_last_time[name] = 0.0
                    self.topic_msg_counts[name] = 0
                    self.topic_hz[name] = 0.0
                self.create_subscription(
                    msg_cls, name,
                    lambda msg, t_name=name, t_str=msg_type_str: self.generic_cb(msg, t_name, t_str),
                    qos_profile_sensor_data
                )
                self.subscribed_topics[name] = msg_type_str

    def generic_cb(self, msg, name, type_str):
        now = time.time()
        self.topic_last_time[name] = now
        self.topic_msg_counts[name] = self.topic_msg_counts.get(name, 0) + 1
        d = self.topic_values[name]

        if type_str == 'nav_msgs/msg/Odometry':
            d['x'] = msg.pose.pose.position.x
            d['y'] = msg.pose.pose.position.y
            d['yaw'] = quat_to_yaw_deg(msg.pose.pose.orientation)
            d['v'] = msg.twist.twist.linear.x
            d['vy'] = msg.twist.twist.linear.y
            d['w'] = msg.twist.twist.angular.z
            d['cov_x'] = float(msg.pose.covariance[0])
            d['cov_y'] = float(msg.pose.covariance[7])
            d['cov_yaw'] = float(msg.pose.covariance[35])
            d['cov_vx'] = float(msg.twist.covariance[0])
            if name == '/odom' and not self.yaw_initialized:
                self.integrated_yaw_ref = d['yaw']
                self.yaw_initialized = True

        elif type_str == 'geometry_msgs/msg/Twist':
            d['v'] = msg.linear.x
            d['vy'] = msg.linear.y
            d['w'] = msg.angular.z

        elif type_str == 'sensor_msgs/msg/Imu':
            d['yaw'] = quat_to_yaw_deg(msg.orientation)
            d['w'] = msg.angular_velocity.z

        elif type_str == 'geometry_msgs/msg/PoseStamped':
            d['x'] = msg.pose.position.x
            d['y'] = msg.pose.position.y
            d['yaw'] = quat_to_yaw_deg(msg.pose.orientation)
            if name == '/goal_pose':
                ox = self.topic_values['/odom']['x']
                oy = self.topic_values['/odom']['y']
                self.goal_yaw_ref = math.degrees(math.atan2(d['y'] - oy, d['x'] - ox))

        elif type_str == 'geometry_msgs/msg/PoseWithCovarianceStamped':
            d['x'] = msg.pose.pose.position.x
            d['y'] = msg.pose.pose.position.y
            d['yaw'] = quat_to_yaw_deg(msg.pose.pose.orientation)
            d['cov_x'] = float(msg.pose.covariance[0])
            d['cov_y'] = float(msg.pose.covariance[7])
            d['cov_yaw'] = float(msg.pose.covariance[35])

    def plan_cb(self, msg):
        step = max(1, len(msg.poses) // 40)
        self.plan_data = [{'x': p.pose.position.x, 'y': p.pose.position.y} for p in msg.poses[::step]]
        if len(msg.poses) > 3:
            ox = self.topic_values['/odom']['x']
            oy = self.topic_values['/odom']['y']
            idx = min(5, len(msg.poses) - 1)
            tx = msg.poses[idx].pose.position.x
            ty = msg.poses[idx].pose.position.y
            self.goal_yaw_ref = math.degrees(math.atan2(ty - oy, tx - ox))

    def update_loop(self):
        now = time.time()
        dt = 0.05
        w_ref = self.topic_values['/cmd_vel']['w']
        if abs(w_ref) > 1e-4:
            self.integrated_yaw_ref += math.degrees(w_ref * dt)
            while self.integrated_yaw_ref > 180.0:
                self.integrated_yaw_ref -= 360.0
            while self.integrated_yaw_ref < -180.0:
                self.integrated_yaw_ref += 360.0

        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            d = self.topic_values['TF:map->base_footprint']
            d['x'] = t.transform.translation.x
            d['y'] = t.transform.translation.y
            d['yaw'] = quat_to_yaw_deg(t.transform.rotation)
            self.topic_last_time['TF:map->base_footprint'] = now
        except Exception:
            pass

    def calc_frequencies(self):
        now = time.time()
        dt = max(0.1, now - self.last_hz_calc)
        for name, cnt in list(self.topic_msg_counts.items()):
            if now - self.topic_last_time.get(name, 0.0) > 1.5:
                self.topic_hz[name] = 0.0
            else:
                self.topic_hz[name] = cnt / dt
            self.topic_msg_counts[name] = 0
        self.last_hz_calc = now

    def is_model_connected(self):
        now = time.time()
        odom_alive = (now - self.topic_last_time.get('/odom', 0.0)) < 1.5
        imu_alive  = (now - self.topic_last_time.get('/imu', 0.0)) < 1.5
        return odom_alive or imu_alive

    def get_available_topics(self):
        res = []
        for k, v in sorted(self.subscribed_topics.items()):
            res.append({'name': k, 'type': v, 'hz': self.topic_hz.get(k, 0.0)})
        res.append({'name': 'TF:map->base_footprint', 'type': 'tf2_msgs/Transform', 'hz': self.topic_hz.get('TF:map->base_footprint', 0.0)})
        return res

ros_node = None

class DashboardHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/api/data':
            now = time.time()
            payload = {
                'time': now - ros_node.start_time,
                'model_connected': ros_node.is_model_connected(),
                'hz': {
                    'odom': ros_node.topic_hz.get('/odom', 0.0),
                    'imu': ros_node.topic_hz.get('/imu', 0.0),
                    'ekf': ros_node.topic_hz.get('/odometry/filtered', 0.0),
                },
                'integrated_yaw_ref': ros_node.integrated_yaw_ref,
                'goal_yaw_ref': ros_node.goal_yaw_ref,
                'topics': ros_node.topic_values,
                'plan': ros_node.plan_data
            }
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(payload).encode('utf-8'))
        elif self.path == '/api/topics':
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(ros_node.get_available_topics()).encode('utf-8'))
        elif self.path == '/api/sync_yaw':
            ros_node.integrated_yaw_ref = ros_node.topic_values['/odom']['yaw']
            self.send_response(200)
            self.end_headers()
            self.wfile.write(b'OK')
        else:
            self.send_response(200)
            self.send_header('Content-type', 'text/html; charset=utf-8')
            self.end_headers()
            self.wfile.write(HTML_PAGE.encode('utf-8'))

    def log_message(self, format, *args):
        return

def main():
    global ros_node
    rclpy.init()
    ros_node = WebTelemetryNode()

    server = HTTPServer(('0.0.0.0', 8080), DashboardHandler)
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()

    print("=====================================================")
    print(" Web Dashboard (PID & 5-Plot EKF): http://localhost:8080")
    print(" Nhấn Ctrl+C để dừng.")
    print("=====================================================")

    try:
        rclpy.spin(ros_node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()