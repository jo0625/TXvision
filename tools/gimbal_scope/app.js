/* Local history and read-only ROS feedback scope. Angles in files are radians. */
(() => {
  'use strict';
  const $ = id => document.getElementById(id);
  const DEG = 180 / Math.PI;
  const KEYS = ['feedback', 'forwarded', 'command_active', 'detector', 'tracker', 'preview'];
  const emptyRows = () => Object.fromEntries(KEYS.map(key => [key, []]));
  const state = {records: [], source: null, axis: 'pitch', range: [13, 17], paused: false,
    live: {rows: emptyRows()}, session: null, cursor: 0, elapsed: 0, generation: 0,
    pollTimer: null, frozen: null, events: [], plot: null};
  const chart = echarts.init($('chart'), null, {renderer: 'canvas'});
  const fmt = (value, digits = 3) => Number.isFinite(value) ? value.toFixed(digits) : '—';
  const isLive = () => state.source === 'live';
  const current = () => isLive() ? (state.frozen || state.live) : state.records.find(r => r.id === state.source);
  const modeName = value => ({0: '0 失能', 3: '3 遥控', 6: '6 视觉'})[value] || `${value}`;
  const inRange = row => row[0] >= state.range[0] && row[0] < state.range[1];
  const last = rows => rows.length ? rows[rows.length - 1] : null;
  let toastTimer;

  function wallTime(record, key, row) {
    const index = record.wall_time_indices?.[key];
    const value = Number.isInteger(index) ? row[index] : null;
    return Number.isFinite(value) && value > 0 && value < 8.64e15 ? value : null;
  }

  function calendar(value) {
    if (!Number.isFinite(value)) return '未记录实际时间';
    return new Date(value + 8 * 3600000).toISOString().slice(0, 23).replace('T', ' ');
  }

  function calendarRange(record) {
    const samples = KEYS.flatMap(key => record.rows[key].filter(inRange)
      .map(row => [row[0], wallTime(record, key, row)]))
      .filter(row => row[1] !== null).sort((a, b) => a[0] - b[0]);
    return samples.length ? `首样本 ${calendar(samples[0][1])}\n末样本 ${calendar(last(samples)[1])}` : '所选区间未记录实际时间';
  }

  const timeTitle = record => ({text: '接收时间 · 北京时间 UTC+08:00',
    subtext: calendarRange(record), left: 12, top: 4,
    textStyle: {fontSize: 11, fontWeight: 'normal', color: '#707980'},
    subtextStyle: {fontSize: 11, lineHeight: 16, color: '#242b31'}, itemGap: 4});

  function toast(message) {
    $('toast').textContent = message;
    $('toast').hidden = false;
    clearTimeout(toastTimer);
    toastTimer = setTimeout(() => { $('toast').hidden = true; }, 4500);
  }

  function normalize(input, metadata) {
    const data = input.data || input;
    if (!data.rows || !Array.isArray(data.rows.feedback)) throw new Error('记录需要包含 rows.feedback 数组');
    const rows = emptyRows();
    for (const key of KEYS) {
      if (data.rows[key] === undefined) continue;
      if (!Array.isArray(data.rows[key])) throw new Error(`${key} 不是数组`);
      rows[key] = data.rows[key].map(row => {
        const minimum = {feedback: 6, forwarded: 3, command_active: 2, detector: 2, tracker: 2, preview: 4}[key];
        if (!Array.isArray(row) || row.length < minimum || !Number.isFinite(row[0]) || row[0] < 0)
          throw new Error(`${key} 含有格式错误的数据`);
        const indices = key === 'feedback' ? [1, 2, 3, 4, 5] :
          (key === 'forwarded' || key === 'preview' ? [1, 2] : []);
        if (indices.some(i => !Number.isFinite(row[i]))) throw new Error(`${key} 含无效数值`);
        if ((key === 'command_active' || key === 'tracker') && typeof row[1] !== 'boolean')
          throw new Error(`${key} 状态必须是布尔值`);
        return row.slice();
      }).sort((a, b) => a[0] - b[0]);
    }
    const meta = {...(data.scope || {}), ...metadata};
    return {...meta, rows, summary: data.summary || {}, duration: Math.max(0.1,
      ...KEYS.map(key => (last(rows[key]) || [0])[0] + 0.001))};
  }

  function valuesFor(record) {
    const index = state.axis === 'pitch' ? 4 : 3;
    let previous = null;
    const feedback = record.rows.feedback.map(row => {
      let value = row[index] * DEG;
      if (state.axis === 'yaw' && previous !== null) value += 360 * Math.round((previous - value) / 360);
      previous = value;
      return [row[0], value, wallTime(record, 'feedback', row)];
    });
    let pos = 0;
    const targets = record.rows.forwarded.map(row => {
      while (pos + 1 < feedback.length && feedback[pos + 1][0] <= row[0]) pos++;
      let value = row[state.axis === 'pitch' ? 2 : 1] * DEG;
      if (state.axis === 'yaw' && feedback.length)
        value += 360 * Math.round((feedback[pos][1] - value) / 360);
      return [row[0], value, wallTime(record, 'forwarded', row)];
    });
    return {feedback, targets};
  }

  function median(values) {
    if (!values.length) return NaN;
    const sorted = values.slice().sort((a, b) => a - b);
    const m = Math.floor(sorted.length / 2);
    return sorted.length % 2 ? sorted[m] : (sorted[m - 1] + sorted[m]) / 2;
  }

  function statistics(points) {
    if (!points.length) return {count: 0};
    const values = points.map(p => p[1]);
    const mean = values.reduce((a, b) => a + b, 0) / values.length;
    const gaps = [], steps = [];
    for (let i = 1; i < points.length; i++) {
      const dt = points[i][0] - points[i - 1][0];
      gaps.push(dt);
      if (dt > 0 && dt <= 0.15) steps.push(Math.abs(points[i][1] - points[i - 1][1]));
    }
    const min = Math.min(...values), max = Math.max(...values);
    const span = points[points.length - 1][0] - points[0][0];
    return {count: points.length, min, max, peak: max - min,
      std: Math.sqrt(values.reduce((sum, v) => sum + (v - mean) ** 2, 0) / values.length),
      step: median(steps), hz: span > 0 ? (points.length - 1) / span : NaN,
      gap: gaps.length ? Math.max(...gaps) * 1000 : NaN};
  }

  function eventsFor(record) {
    const events = [];
    const feedback = record.rows.feedback;
    for (let i = 1; i < feedback.length; i++) {
      if (feedback[i][5] !== feedback[i - 1][5])
        events.push({t: feedback[i][0], text: `模式 → ${modeName(feedback[i][5])}`, kind: 'stop'});
      const gap = feedback[i][0] - feedback[i - 1][0];
      if (gap > 0.15) events.push({t: feedback[i][0], text: `反馈间隔 ${fmt(gap * 1000, 0)} ms`, kind: 'gap'});
    }
    if (record.diagnosed_zero_targets) {
      for (const row of record.rows.forwarded) {
        if (row[1] === 0 && row[2] === 0) events.push({t: row[0], text: '已确认的零值误转发', kind: 'zero', key: 'forwarded'});
      }
    }
    const target = last(record.rows.forwarded);
    if (target && !isLive()) events.push({t: target[0], text: '本记录最后一次转发', kind: 'stop', key: 'forwarded'});
    return events.sort((a, b) => a.t - b.t);
  }

  function breakGaps(points, maxGap) {
    const result = [];
    for (let i = 0; i < points.length; i++) {
      if (i && points[i][0] - points[i - 1][0] > maxGap)
        result.push([(points[i][0] + points[i - 1][0]) / 2, null]);
      result.push(points[i]);
    }
    return result;
  }

  function angleLimits(plot) {
    const points = [...($('show-feedback').checked ? plot.feedback : []),
      ...($('show-target').checked ? plot.targets : [])].filter(inRange);
    if (!points.length) return {min: -1, max: 1};
    const min = Math.min(...points.map(p => p[1])), max = Math.max(...points.map(p => p[1]));
    const pad = Math.max((max - min) * 0.12, 0.05);
    return {min: min - pad, max: max + pad};
  }

  function renderStats(record, plot) {
    const stats = statistics(plot.feedback.filter(inRange));
    $('metric-count').textContent = stats.count;
    $('metric-range').textContent = `${fmt(state.range[0], 1)}–${fmt(state.range[1], 1)} s`;
    for (const key of ['peak', 'std', 'step', 'hz', 'gap'])
      $(`metric-${key}`).textContent = fmt(stats[key], key === 'hz' || key === 'gap' ? 1 : 3);
    $('metric-step').title = '仅计算间隔不超过 150 ms 的相邻反馈，排除断流后的跨段变化';
    $('fact-extents').textContent = stats.count ? `${fmt(stats.min)} ～ ${fmt(stats.max)}°` : '无反馈';
    const modes = [...new Set(record.rows.feedback.filter(inRange).map(row => row[5]))];
    $('fact-mode').textContent = modes.map(modeName).join(' → ') || '无反馈';
    $('fact-commands').textContent = `${record.rows.forwarded.filter(inRange).length} 条`;
    const active = record.rows.command_active.filter(row => row[0] < state.range[1]);
    const before = last(active.filter(row => row[0] <= state.range[0]));
    const states = [...new Set([...(before ? [before[1]] : []), ...active.filter(inRange).map(row => row[1])])];
    $('fact-active').textContent = states.length === 0 ? '未记录' : states.length === 2 ? '有 / 无切换' : states[0] ? '有活动记录' : '无活动记录';
    $('fact-duration').textContent = `${fmt(record.duration || state.elapsed, 1)} s`;
    $('fact-file').textContent = record.filename || 'ROS 实时 · 最近 120 秒';
    $('data-origin').textContent = state.axis === 'yaw' ? '时间轴：主机接收时间 · Yaw 连续展开' : '时间轴：主机接收时间';
    $('range-badge').textContent = `${fmt(state.range[1] - state.range[0], 2)} s`;
    state.events = eventsFor(record);
    $('event-count').textContent = state.events.length;
    $('events').replaceChildren();
    for (const event of state.events.slice(-150)) {
      const button = document.createElement('button');
      if (event.t >= state.range[0] && event.t < state.range[1]) button.className = 'in-range';
      const dot = document.createElement('span'); dot.className = `event-dot ${event.kind}`;
      const label = document.createElement('span'); label.textContent = event.text;
      const time = document.createElement('span'); time.className = 'event-time'; time.textContent = `${fmt(event.t, 2)}s`;
      const eventKey = event.key || 'feedback';
      const eventRow = record.rows[eventKey].find(row => row[0] === event.t);
      const eventWall = eventRow ? wallTime(record, eventKey, eventRow) : null;
      if (eventWall !== null) {
        const stamp = document.createElement('small');
        stamp.className = 'event-calendar'; stamp.textContent = `${calendar(eventWall)} +08:00`;
        label.append(stamp);
      }
      button.append(dot, label, time);
      button.onclick = () => { if (isLive()) setPaused(true); setRange(event.t - 1, event.t + 1, true); };
      $('events').append(button);
    }
    if (!state.events.length) {
      const empty = document.createElement('div'); empty.className = 'no-events'; empty.textContent = '暂无事件'; $('events').append(empty);
    }
  }

  function render() {
    const record = current();
    if (!record) return;
    const plot = valuesFor(record);
    state.plot = plot;
    renderStats(record, plot);
    $('range-start').value = fmt(state.range[0], 3);
    $('range-end').value = fmt(state.range[1], 3);
    $('chart-title').textContent = state.axis === 'pitch' ? 'Pitch 俯仰角' : 'Yaw 偏航角';
    const duration = Math.max(state.range[1], record.duration || state.elapsed || 1);
    const modes = breakGaps(record.rows.feedback.map(row => [row[0], row[5], wallTime(record, 'feedback', row)]), 0.15);
    const active = record.rows.command_active.map(row => [row[0], row[1] ? 1 : 0, wallTime(record, 'command_active', row)]);
    // This state belongs to the host writer, not a controller setpoint or motor state.
    if (active.length) active.push([Math.min(duration, Math.max(last(active)[0], last(record.rows.feedback)?.[0] || 0)), last(active)[1]]);
    const small = $('chart').clientWidth < 500;
    const xAxis = [0, 1, 2].map(i => ({type: 'value', gridIndex: i, min: 0, max: duration,
      axisLabel: {show: i === 2, color: '#79838a', formatter: v => `${+v.toFixed(1)}s`},
      axisLine: {show: false}, axisTick: {show: false}, splitLine: {show: true, lineStyle: {color: '#edf0f2'}},
      axisPointer: {show: true, label: {formatter: p => `${fmt(p.value)} s`}}}));
    const line = (name, data, color, axisIndex, extra = {}) => ({name, type: 'line', data,
      encode: {x: 0, y: 1},
      xAxisIndex: axisIndex, yAxisIndex: axisIndex, showSymbol: false, connectNulls: false,
      symbolSize: 3, lineStyle: {width: 1.5, color}, itemStyle: {color}, animation: false, ...extra});
    chart.setOption({animation: false, backgroundColor: '#ffffff',
      title: timeTitle(record),
      textStyle: {fontFamily: 'Microsoft YaHei, sans-serif'},
      grid: [{left: 60, right: 22, top: 100, height: '33%'},
        {left: 60, right: 22, top: '63%', height: '10%'},
        {left: 60, right: 22, top: '80%', height: '6%'}],
      tooltip: {trigger: 'axis', confine: true, valueFormatter: v => v == null ? '缺失' : fmt(v),
        formatter: entries => entries.map(p => `${p.marker}${p.seriesName}: ${p.value[1] == null ? '缺失' : fmt(p.value[1])}` +
          `<br>${fmt(p.value[0])} s · ${calendar(p.value[2])}${Number.isFinite(p.value[2]) ? ' +08:00' : ''}`).join('<br>'),
        axisPointer: {type: 'cross'}, textStyle: {fontSize: 12}},
      axisPointer: {link: [{xAxisIndex: 'all'}]},
      xAxis, yAxis: [{type: 'value', name: '角度 / °', scale: true, ...angleLimits(plot), nameTextStyle: {color: '#707980'},
        axisLabel: {color: '#707980', formatter: v => +v.toFixed(3)}, splitLine: {lineStyle: {color: '#edf0f2'}}},
        {type: 'value', gridIndex: 1, name: '模式', min: 0, max: 6, interval: 3, nameGap: 8,
          nameTextStyle: {color: '#707980'}, axisLabel: {color: '#707980'}, splitLine: {show: false}},
        {type: 'value', gridIndex: 2, name: '发送', min: 0, max: 1, interval: 1, nameGap: 8,
          nameTextStyle: {color: '#707980'}, axisLabel: {color: '#707980'}, splitLine: {show: false}}],
      dataZoom: [{type: 'inside', xAxisIndex: [0, 1, 2], filterMode: 'none', startValue: state.range[0], endValue: state.range[1]},
        {type: 'slider', xAxisIndex: [0, 1, 2], filterMode: 'none', bottom: 7, height: 18,
          showDetail: !small, startValue: state.range[0], endValue: state.range[1], borderColor: '#dce4e6',
          fillerColor: '#117c7319', handleStyle: {color: '#117c73'}}],
      series: [line('角度反馈 / °', $('show-feedback').checked ? breakGaps(plot.feedback, 0.15) : [], '#117c73', 0,
        {showSymbol: state.range[1] - state.range[0] < 8}),
      line('转发目标 / °', $('show-target').checked ? breakGaps(plot.targets, 0.15) : [], '#bf6b27', 0,
        {showSymbol: true, symbolSize: 3}),
      line('模式', modes, '#527aa7', 1, {step: 'end'}),
      line('发送活动', active, '#9298a0', 2, {step: 'end'})]}, {notMerge: true});
    $('empty-state').hidden = !isLive() || record.rows.feedback.some(inRange);
  }

  function setRange(start, end, custom = false) {
    const record = current();
    const duration = Math.max(0.1, isLive() ? state.elapsed : record.duration);
    start = Math.max(0, Number(start)); end = Math.min(duration, Number(end));
    if (!Number.isFinite(start) || !Number.isFinite(end) || end <= start) { toast('结束时间需要大于开始时间'); return; }
    state.range = [start, end];
    if (custom) $('preset-select').value = 'custom';
    render();
  }

  function preset() {
    const record = current();
    const selected = $('preset-select').value;
    const bounds = record.presets?.[selected];
    if (bounds) setRange(...bounds);
    else if (selected === 'all') setRange(0, record.duration);
  }

  function setPaused(paused) {
    if (paused && !state.paused) state.frozen = {...state.live, duration: state.elapsed,
      rows: Object.fromEntries(KEYS.map(key => [key, state.live.rows[key].slice()]))};
    if (!paused) state.frozen = null;
    state.paused = paused;
    $('pause-button').innerHTML = `<i data-lucide="${paused ? 'play' : 'pause'}"></i>`;
    $('pause-button').title = paused ? '继续曲线' : '暂停曲线';
    $('pause-button').setAttribute('aria-label', $('pause-button').title);
    lucide.createIcons();
    if (!paused && isLive()) { updateLiveRange(); render(); }
  }

  function updateLiveRange() {
    const end = Math.max(state.elapsed, 0.1);
    state.range = [Math.max(0, end - Number($('live-window').value)), end];
  }

  function status(text, kind = '') {
    $('connection-text').textContent = text;
    $('connection').className = `connection ${kind}`;
  }

  async function poll(generation) {
    if (!isLive() || generation !== state.generation) return;
    try {
      const response = await fetch(`/api/live?after=${state.cursor}`, {cache: 'no-store', signal: AbortSignal.timeout(3000)});
      if (!response.ok) throw new Error('实时服务不可用');
      const packet = await response.json();
      if (generation !== state.generation) return;
      if (state.session && state.session !== packet.session) {
        state.cursor = 0; state.live = {rows: emptyRows()}; state.session = packet.session;
        toast('实时服务已重启，开始新的记录');
        state.pollTimer = setTimeout(() => poll(generation), 0); return;
      }
      state.session = packet.session;
      state.elapsed = packet.elapsed;
      state.cursor = packet.cursor;
      state.live.wall_time_indices = packet.wall_time_indices || {};
      state.live.wall_time_source = packet.wall_time_source || null;
      for (const [, key, row] of packet.events) if (state.live.rows[key]) state.live.rows[key].push(row);
      for (const key of KEYS) state.live.rows[key] = state.live.rows[key].filter(row => row[0] >= state.elapsed - 120);
      if (packet.truncated) toast('离开时间较长，仅保留最近 120 秒记录');
      const fresh = packet.feedback_age !== null && packet.feedback_age < 0.5;
      status(!packet.ros_ready ? 'ROS 订阅不可用' : fresh ? (state.paused ? '实时 · 已暂停' : '实时 · 只读') : '等待反馈', fresh ? 'is-live' : 'is-waiting');
      $('empty-detail').textContent = !packet.ros_ready ? `ROS 未就绪：${packet.error || '正在连接'}` : '尚无新反馈';
      if (!state.paused) { updateLiveRange(); render(); }
    } catch (error) {
      if (generation !== state.generation) return;
      status('实时服务未连接', 'is-waiting');
      $('empty-detail').textContent = '实时服务未连接';
    }
    if (generation === state.generation) state.pollTimer = setTimeout(() => poll(generation), 250);
  }

  function chooseSource() {
    clearTimeout(state.pollTimer);
    state.generation++;
    state.source = $('source-select').value;
    $('history-tools').hidden = isLive(); $('live-tools').hidden = !isLive();
    state.paused = false;
    state.frozen = null;
    if (isLive()) {
      status('等待反馈', 'is-waiting');
      updateLiveRange(); render(); setPaused(false); poll(state.generation);
    } else {
      status('历史记录'); $('empty-state').hidden = true;
      const record = current();
      for (const option of $('preset-select').options)
        option.disabled = ['post', 'disabled'].includes(option.value) && !record.presets?.[option.value];
      $('preset-select').value = record.presets?.post ? 'post' : 'all';
      preset();
    }
  }

  function addOption(record) {
    const option = document.createElement('option'); option.value = record.id; option.textContent = record.label;
    $('source-select').append(option);
  }

  function download(blob, filename) {
    const url = URL.createObjectURL(blob); const link = document.createElement('a');
    link.href = url; link.download = filename; link.click(); setTimeout(() => URL.revokeObjectURL(url), 3000);
  }

  $('source-select').onchange = chooseSource;
  $('preset-select').onchange = preset;
  document.querySelectorAll('[data-axis]').forEach(button => {
    button.onclick = () => {
      state.axis = button.dataset.axis;
      document.querySelectorAll('[data-axis]').forEach(b => b.setAttribute('aria-pressed', String(b === button)));
      render();
    };
  });
  $('show-feedback').onchange = render; $('show-target').onchange = render;
  $('apply-range').onclick = () => { if (isLive()) setPaused(true); setRange($('range-start').value, $('range-end').value, true); };
  $('reset-zoom').onclick = () => isLive() ? setPaused(false) : setRange(0, current().duration, true);
  $('live-window').onchange = () => { if (!state.paused) { updateLiveRange(); render(); } };
  $('pause-button').onclick = () => setPaused(!state.paused);
  $('import-button').onclick = () => $('file-input').click();
  $('file-input').onchange = async event => {
    const file = event.target.files[0]; if (!file) return;
    try {
      const record = normalize(JSON.parse(await file.text()), {id: `import-${Date.now()}`, label: file.name, filename: file.name});
      state.records.push(record); addOption(record); $('source-select').value = record.id; chooseSource();
    } catch (error) { toast(`无法打开记录：${error.message}`); }
    event.target.value = '';
  };
  $('save-json').onclick = () => {
    const record = current();
    download(new Blob([JSON.stringify({rows: record.rows, summary: record.summary || {}, scope: {
      presets: record.presets || {}, diagnosed_zero_targets: !!record.diagnosed_zero_targets,
      source: 'gimbal_scope', time_base: 'host_monotonic_receive', live_retention_seconds: isLive() ? 120 : null,
      wall_time_indices: record.wall_time_indices || {}, wall_time_source: record.wall_time_source || null,
      display_timezone: 'Asia/Shanghai'
    }})], {type: 'application/json'}), `gimbal_${isLive() ? 'live' : state.source}.json`);
  };
  $('export-csv').onclick = () => {
    const record = current();
    const lines = ['kind,time_s,yaw_deg,pitch_deg,mode,sequence,mcu_ms,command_active,host_receive_unix_ms,host_receive_time_utc08'];
    const stamp = (key, row) => {
      const value = wallTime(record, key, row);
      return [value ?? '', value === null ? '' : calendar(value).replace(' ', 'T') + '+08:00'];
    };
    for (const row of record.rows.feedback.filter(inRange)) lines.push(['feedback', row[0], row[3] * DEG, row[4] * DEG, row[5], row[1], row[2], '', ...stamp('feedback', row)].join(','));
    for (const row of record.rows.forwarded.filter(inRange)) lines.push(['target', row[0], row[1] * DEG, row[2] * DEG, '', '', '', '', ...stamp('forwarded', row)].join(','));
    for (const row of record.rows.command_active.filter(inRange)) lines.push(['active', row[0], '', '', '', '', '', row[1] ? 1 : 0, ...stamp('command_active', row)].join(','));
    download(new Blob([lines.join('\n')], {type: 'text/csv;charset=utf-8'}), 'gimbal_interval.csv');
  };
  $('export-png').onclick = () => {
    const link = document.createElement('a'); link.href = chart.getDataURL({type: 'png', pixelRatio: 2, backgroundColor: '#fff'});
    link.download = `gimbal_${state.axis}.png`; link.click();
  };
  chart.on('datazoom', event => {
    if (isLive()) setPaused(true);
    const zoom = event.batch?.[0] || event;
    const duration = Math.max(state.range[1], current().duration || state.elapsed || 1);
    state.range = [zoom.startValue ?? zoom.start / 100 * duration, zoom.endValue ?? zoom.end / 100 * duration];
    $('preset-select').value = 'custom';
    $('range-start').value = fmt(state.range[0], 3); $('range-end').value = fmt(state.range[1], 3);
    renderStats(current(), state.plot);
    chart.setOption({title: timeTitle(current()), yAxis: [angleLimits(state.plot), {}, {}]});
  });
  new ResizeObserver(() => chart.resize()).observe($('chart'));
  for (const item of JSON.parse($('embedded-records').textContent).records) {
    const {data, ...meta} = item; const record = normalize(data, meta); state.records.push(record); addOption(record);
  }
  const option = document.createElement('option'); option.value = 'live'; option.textContent = '实时反馈 · 只读';
  option.disabled = location.protocol === 'file:'; $('source-select').append(option);
  if (option.disabled) option.textContent += '（需本地服务）';
  if (state.records.length && !(location.protocol !== 'file:' && new URLSearchParams(location.search).has('live'))) { $('source-select').value = state.records[0].id; chooseSource(); }
  else { $('source-select').value = 'live'; chooseSource(); }
  lucide.createIcons();
})();
