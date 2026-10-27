(() => {
  const $ = (id) => document.getElementById(id);
  const ui = {
    connection: $('connectionStatus'), sensor: $('sensorStatus'), motor: $('motorStatus'),
    banner: $('safetyBanner'), angle: $('angleValue'), turns: $('turnValue'),
    velocity: $('velocityValue'), target: $('targetValue'), targetUnit: $('targetUnit'),
    magnitude: $('magnitudeValue'), rate: $('loopRate'), errors: $('sensorErrors'),
    calibration: $('calibrationChip'), enable: $('enableSwitch'), mode: $('modeSelect'),
    slider: $('targetSlider'), targetInput: $('targetInput'), rangeText: $('targetRangeText'),
    zero: $('zeroTarget'), estop: $('estopButton'), calibrate: $('calibrateButton'),
    save: $('saveButton'), supply: $('supplyInput'), polePairs: $('polePairsInput'),
    voltageLimit: $('voltageLimitInput'), velocityLimit: $('velocityLimitInput'),
    currentLimit: $('currentLimitInput'), applyLimits: $('applyLimits'),
    velP: $('velPInput'), velI: $('velIInput'), velD: $('velDInput'),
    velTf: $('velTfInput'), angP: $('angPInput'), applyPid: $('applyPid'),
    restart: $('restartButton'), ip: $('ipAddress'), profile: $('firmwareProfile'),
    heap: $('heapValue'), updateForm: $('updateForm'), file: $('firmwareFile'),
    fileLabel: $('fileLabel'), progress: $('updateProgress'), chart: $('telemetryChart'),
    toasts: $('toastStack')
  };

  const modes = [
    { unit: 'rad/s', kind: 'velocity' },
    { unit: 'rad', kind: 'angle' },
    { unit: 'rad/s', kind: 'velocity' },
    { unit: 'rad', kind: 'angle' },
    { unit: 'V', kind: 'torque' }
  ];

  let socket;
  let reconnectTimer;
  let state = null;
  let editing = false;
  const velocityHistory = Array(140).fill(0);
  const targetHistory = Array(140).fill(0);

  function setStatus(node, text, level = '') {
    node.className = `status ${level}`.trim();
    node.lastChild.textContent = ` ${text}`;
  }

  function toast(text, ok = true) {
    const item = document.createElement('div');
    item.className = `toast ${ok ? 'ok' : 'bad'}`;
    item.textContent = text;
    ui.toasts.appendChild(item);
    setTimeout(() => item.remove(), 4300);
  }

  function send(action, payload = {}) {
    if (!socket || socket.readyState !== WebSocket.OPEN) {
      toast('Controller is not connected', false);
      return false;
    }
    socket.send(JSON.stringify({ action, ...payload }));
    return true;
  }

  function connect() {
    clearTimeout(reconnectTimer);
    setStatus(ui.connection, 'Connecting', 'warn');
    socket = new WebSocket(`ws://${location.host}/ws`);
    socket.onopen = () => {
      setStatus(ui.connection, 'Connected', 'ok');
      send('getState');
    };
    socket.onclose = () => {
      setStatus(ui.connection, 'Disconnected', 'bad');
      reconnectTimer = setTimeout(connect, 1400);
    };
    socket.onerror = () => socket.close();
    socket.onmessage = ({ data }) => {
      try {
        const message = JSON.parse(data);
        if (message.type === 'state') applyState(message);
        else if (message.type === 'telemetry') applyTelemetry(message);
        else if (message.type === 'msg') toast(message.text, message.ok);
      } catch (_) {
        toast('Invalid controller response', false);
      }
    };
  }

  function setInput(node, value) {
    if (!editing && document.activeElement !== node) node.value = value;
  }

  function applyState(next) {
    state = next;
    setStatus(ui.sensor, next.sensor ? 'AS5048A ready' : 'Encoder missing', next.sensor ? 'ok' : 'bad');

    if (!next.motorCompiled) {
      setStatus(ui.motor, 'Output locked', 'warn');
      ui.banner.className = 'banner warning';
      ui.banner.innerHTML = '<strong>Safe build:</strong> the webpage and encoder are live, but the motor stage is locked. Verify the encoder first, then flash <code>esp32dev-motor</code>.';
    } else if (!next.driver) {
      setStatus(ui.motor, 'Driver failed', 'bad');
      ui.banner.className = 'banner warning';
      ui.banner.innerHTML = '<strong>Driver error:</strong> motor PWM initialisation failed. Keep motor power off and inspect the serial log.';
    } else {
      setStatus(ui.motor, next.enabled ? 'Output enabled' : 'Output disabled', next.enabled ? 'ok' : 'warn');
      ui.banner.className = 'banner ready';
      ui.banner.innerHTML = '<strong>Motor build:</strong> PWM 16/17/5 and enable 4 are active. Use a current-limited 12 V supply and begin with the 0.5 V limit.';
    }

    const motorControls = [ui.enable, ui.mode, ui.slider, ui.targetInput, ui.zero, ui.calibrate];
    motorControls.forEach((control) => { control.disabled = !next.motorCompiled || !next.driver || next.deckConnected; });
    if (next.deckConnected) {
      ui.banner.textContent = 'Windows deck connected. Disconnect it before calibrating or tuning here. E-STOP remains available.';
    }
    [ui.applyLimits, ui.applyPid, ui.save, ui.restart].forEach(control => { control.disabled = Boolean(next.deckConnected); });
    ui.currentLimit.disabled = !next.currentCompiled;
    ui.enable.checked = Boolean(next.enabled);
    ui.mode.value = String(next.mode);
    ui.calibration.textContent = next.calibrated ? 'FOC calibrated' : 'Not calibrated';
    ui.calibration.style.color = next.calibrated ? 'var(--green)' : '';
    ui.errors.textContent = next.sensorErrors;
    ui.magnitude.textContent = next.sensor ? next.magnitude : '—';
    ui.ip.textContent = next.ip || '192.168.4.1';
    ui.profile.textContent = next.motorCompiled
      ? (next.currentCompiled ? 'Motor + current sense' : 'Motor / voltage control')
      : 'Safe / sensor only';
    ui.heap.textContent = `${Math.round((next.heap || 0) / 1024)} KB free heap`;

    setInput(ui.supply, next.supply);
    setInput(ui.polePairs, next.polePairs);
    setInput(ui.voltageLimit, next.voltageLimit);
    setInput(ui.velocityLimit, next.velocityLimit);
    setInput(ui.currentLimit, next.currentLimit);
    setInput(ui.velP, next.velP);
    setInput(ui.velI, next.velI);
    setInput(ui.velD, next.velD);
    setInput(ui.velTf, next.velTf);
    setInput(ui.angP, next.angP);
    setInput(ui.slider, next.target);
    setInput(ui.targetInput, next.target);
    configureTargetRange();
  }

  function applyTelemetry(data) {
    const angle = Number(data.angle || 0);
    const velocity = Number(data.velocity || 0);
    const target = Number(data.target || 0);
    ui.angle.textContent = `${((angle * 180 / Math.PI) % 360).toFixed(2)}°`;
    ui.turns.textContent = `${(angle / (2 * Math.PI)).toFixed(3)} turns`;
    ui.velocity.textContent = velocity.toFixed(2);
    ui.target.textContent = target.toFixed(2);
    ui.rate.textContent = `${Number(data.rate || 0).toLocaleString()} control loops/s`;
    if (data.magnitude) ui.magnitude.textContent = data.magnitude;
    velocityHistory.push(velocity); velocityHistory.shift();
    targetHistory.push(target); targetHistory.shift();
    drawChart();
  }

  function configureTargetRange() {
    if (!state) return;
    const mode = Number(ui.mode.value);
    const info = modes[mode];
    let limit = info.kind === 'angle' ? 6.283 : state.velocityLimit;
    if (info.kind === 'torque') limit = state.currentCompiled ? state.currentLimit : state.voltageLimit;
    limit = Math.max(Number(limit) || 1, 0.1);
    ui.slider.min = -limit;
    ui.slider.max = limit;
    ui.slider.step = info.kind === 'angle' ? 0.01 : 0.1;
    ui.targetInput.min = -limit;
    ui.targetInput.max = limit;
    ui.targetInput.step = ui.slider.step;
    ui.targetUnit.textContent = info.unit;
    ui.rangeText.textContent = `−${limit.toFixed(2)} to +${limit.toFixed(2)} ${info.unit}`;
  }

  function commitTarget(value) {
    const min = Number(ui.slider.min), max = Number(ui.slider.max);
    const safe = Math.min(max, Math.max(min, Number(value) || 0));
    ui.slider.value = safe;
    ui.targetInput.value = safe;
    send('setTarget', { value: safe });
  }

  function drawChart() {
    const canvas = ui.chart;
    const rect = canvas.getBoundingClientRect();
    const ratio = Math.min(window.devicePixelRatio || 1, 2);
    const width = Math.max(1, Math.floor(rect.width * ratio));
    const height = Math.max(1, Math.floor(rect.height * ratio));
    if (canvas.width !== width || canvas.height !== height) {
      canvas.width = width; canvas.height = height;
    }
    const ctx = canvas.getContext('2d');
    ctx.clearRect(0, 0, width, height);
    const maxValue = Math.max(1, ...velocityHistory.map(Math.abs), ...targetHistory.map(Math.abs));
    const mid = height / 2;
    ctx.strokeStyle = 'rgba(148,184,212,.13)';
    ctx.beginPath(); ctx.moveTo(0, mid); ctx.lineTo(width, mid); ctx.stroke();
    const plot = (values, color, lineWidth) => {
      ctx.beginPath();
      values.forEach((value, index) => {
        const x = index * width / (values.length - 1);
        const y = mid - value / maxValue * (height * .41);
        index ? ctx.lineTo(x, y) : ctx.moveTo(x, y);
      });
      ctx.strokeStyle = color; ctx.lineWidth = lineWidth * ratio; ctx.stroke();
    };
    plot(targetHistory, 'rgba(242,184,75,.65)', 1.2);
    plot(velocityHistory, '#48d7e6', 1.8);
  }

  [ui.supply, ui.polePairs, ui.voltageLimit, ui.velocityLimit, ui.currentLimit,
   ui.velP, ui.velI, ui.velD, ui.velTf, ui.angP].forEach((node) => {
    node.addEventListener('focus', () => { editing = true; });
    node.addEventListener('blur', () => { editing = false; });
  });

  ui.enable.addEventListener('change', () => send('setEnable', { on: ui.enable.checked }));
  ui.mode.addEventListener('change', () => {
    configureTargetRange();
    commitTarget(0);
    send('setMode', { mode: Number(ui.mode.value) });
  });
  ui.slider.addEventListener('input', () => { ui.targetInput.value = ui.slider.value; });
  ui.slider.addEventListener('change', () => commitTarget(ui.slider.value));
  ui.targetInput.addEventListener('change', () => commitTarget(ui.targetInput.value));
  ui.zero.addEventListener('click', () => commitTarget(0));
  ui.estop.addEventListener('click', () => { ui.enable.checked = false; commitTarget(0); send('estop'); });
  ui.calibrate.addEventListener('click', () => {
    if (confirm('The motor will move during calibration. Continue?')) send('calibrate');
  });
  ui.save.addEventListener('click', () => send('saveConfig'));
  ui.applyLimits.addEventListener('click', () => {
    send('setSupply', { volts: Number(ui.supply.value) });
    send('setPolePairs', { value: Number(ui.polePairs.value) });
    send('setLimits', {
      voltage: Number(ui.voltageLimit.value), velocity: Number(ui.velocityLimit.value),
      current: Number(ui.currentLimit.value)
    });
  });
  ui.applyPid.addEventListener('click', () => send('setPID', {
    velP: Number(ui.velP.value), velI: Number(ui.velI.value), velD: Number(ui.velD.value),
    velTf: Number(ui.velTf.value), angP: Number(ui.angP.value)
  }));
  ui.restart.addEventListener('click', () => {
    if (confirm('Restart the ESP32 controller?')) send('restart');
  });

  ui.file.addEventListener('change', () => {
    ui.fileLabel.textContent = ui.file.files[0]?.name || 'Choose firmware .bin';
  });
  ui.updateForm.addEventListener('submit', (event) => {
    event.preventDefault();
    const file = ui.file.files[0];
    if (!file || !file.name.toLowerCase().endsWith('.bin')) {
      toast('Select a PlatformIO firmware .bin file', false); return;
    }
    if (!confirm('Motor output will stop and the controller will reboot. Upload firmware?')) return;
    const request = new XMLHttpRequest();
    const form = new FormData(); form.append('firmware', file, file.name);
    ui.progress.classList.remove('hidden');
    request.upload.onprogress = (e) => {
      if (e.lengthComputable) ui.progress.firstElementChild.style.width = `${e.loaded / e.total * 100}%`;
    };
    request.onload = () => toast(request.status === 200 ? 'Firmware accepted; reconnecting shortly' : 'Firmware upload failed', request.status === 200);
    request.onerror = () => toast('Firmware upload failed', false);
    request.open('POST', '/update'); request.send(form);
  });

  window.addEventListener('resize', drawChart);
  drawChart();
  connect();
})();
