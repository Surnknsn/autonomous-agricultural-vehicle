(function () {
  "use strict";

  function safeGet(id) {
    return document.getElementById(id);
  }

  function setStyleMode(enabled) {
    var useStick = !!enabled;
    var g1 = safeGet("joy-grid-wrap");
    var s1 = safeGet("joy-stick-wrap");
    var g2 = safeGet("follow-joy-grid-wrap");
    var s2 = safeGet("follow-joy-stick-wrap");
    if (g1) g1.style.display = useStick ? "none" : "grid";
    if (s1) s1.style.display = useStick ? "flex" : "none";
    if (g2) g2.style.display = useStick ? "none" : "grid";
    if (s2) s2.style.display = useStick ? "flex" : "none";

    var sw1 = safeGet("emer-style-switch");
    var sw2 = safeGet("follow-emer-style-switch");
    if (sw1) sw1.checked = useStick;
    if (sw2) sw2.checked = useStick;

    var l1 = safeGet("emer-style-label");
    var l2 = safeGet("follow-emer-style-label");
    var txt = useStick ? "ON" : "OFF";
    if (l1) {
      l1.textContent = txt;
      l1.className = useStick ? "fw-bold text-success" : "fw-bold text-danger";
    }
    if (l2) {
      l2.textContent = txt;
      l2.className = useStick ? "fw-bold text-success" : "fw-bold text-danger";
    }
  }

  function bindSwitches() {
    var sw1 = safeGet("emer-style-switch");
    var sw2 = safeGet("follow-emer-style-switch");
    if (sw1) sw1.addEventListener("change", function () { setStyleMode(sw1.checked); });
    if (sw2) sw2.addEventListener("change", function () { setStyleMode(sw2.checked); });
    setStyleMode(false);
  }

  function bindStick(baseId, knobId, stateId, options) {
    options = options || {};
    var base = safeGet(baseId);
    var knob = safeGet(knobId);
    var state = safeGet(stateId);
    if (!base || !knob) return;
    if (typeof window.sendManualCmd !== "function") return;

    var active = false;
    var currentCmd = "STOP";
    var holdTimer = null;
    var pointerId = null;

    function clampPwm(v) {
      v = Math.round(Number(v) || 0);
      if (v > 255) return 255;
      if (v < -255) return -255;
      return v;
    }

    function mapLogicalToArduino(l, r) {
      // Must match lawnmower_node.py: rawL=-logicalR, rawR=logicalL.
      return { l: clampPwm(-r), r: clampPwm(l) };
    }

    function describeCmd(cmd) {
      var raw;
      if (cmd.indexOf("PWM,") === 0) {
        var p = cmd.split(",");
        if (p.length === 3) {
          if (options.smoothFromZero) {
            return "PWM L " + Number(p[1]) + ", R " + Number(p[2]) + " / 255";
          }
          raw = mapLogicalToArduino(Number(p[1]), Number(p[2]));
          return "PWM raw L " + raw.l + ", R " + raw.r;
        }
      }
      if (cmd === "FORWARD") raw = mapLogicalToArduino(170, 170);
      else if (cmd === "BACK") raw = mapLogicalToArduino(-170, -170);
      else if (cmd === "LEFT") raw = mapLogicalToArduino(-170, 170);
      else if (cmd === "RIGHT") raw = mapLogicalToArduino(170, -170);
      else if (cmd === "STOP") raw = mapLogicalToArduino(0, 0);
      if (raw) return cmd + " raw L " + raw.l + ", R " + raw.r;
      return cmd;
    }

    function sendCmd(cmd) {
      if (cmd !== currentCmd) {
        currentCmd = cmd;
        window.sendManualCmd(cmd);
      }
      if (state) state.textContent = describeCmd(cmd);
    }

    function startHold() {
      if (holdTimer) return;
      holdTimer = setInterval(function () {
        window.sendManualCmd(currentCmd);
      }, 120);
    }

    function stopHold() {
      if (!holdTimer) return;
      clearInterval(holdTimer);
      holdTimer = null;
    }

    function resetKnob() {
      knob.style.left = "50%";
      knob.style.top = "50%";
      knob.style.transform = "translate(-50%, -50%)";
      sendCmd("STOP");
      stopHold();
    }

    function update(clientX, clientY) {
      var rect = base.getBoundingClientRect();
      var cx = rect.left + rect.width / 2;
      var cy = rect.top + rect.height / 2;
      var dx = clientX - cx;
      var dy = clientY - cy;
      var dist = Math.hypot(dx, dy);
      var maxR = rect.width * 0.36;
      if (dist > maxR && dist > 0) {
        dx = dx * maxR / dist;
        dy = dy * maxR / dist;
        dist = maxR;
      }

      knob.style.left = (50 + (dx / (rect.width / 2)) * 50) + "%";
      knob.style.top = (50 + (dy / (rect.height / 2)) * 50) + "%";
      knob.style.transform = "translate(-50%, -50%)";

      var dead = rect.width * 0.14;
      if (dist < dead) {
        sendCmd("STOP");
        return;
      }

      // Differential mix for diagonal drive:
      // forward = -dy. Stick-left must produce logical LEFT (-,+),
      // because lawnmower_node maps that to Arduino raw (-,-).
      var nX;
      var nY;
      if (options.smoothFromZero) {
        var scale = (dist - dead) / Math.max(1, (maxR - dead));
        if (scale < 0) scale = 0;
        if (scale > 1) scale = 1;
        nX = (dist > 0 ? dx / dist : 0) * scale;   // starts at 0 after deadband
        nY = (dist > 0 ? -dy / dist : 0) * scale;  // starts at 0 after deadband
      } else {
        nX = dx / maxR;   // -1..1 (left..right)
        nY = -dy / maxR;  // -1..1 (back..forward)
      }
      var maxPwm = 255;
      var fwd = nY * maxPwm;
      var turn = -nX * maxPwm;
      // nX < 0 (stick left) => turn > 0 => left wheel slower, right wheel faster.
      var l = Math.round(fwd - turn);
      var r = Math.round(fwd + turn);
      if (l > 255) l = 255;
      if (l < -255) l = -255;
      if (r > 255) r = 255;
      if (r < -255) r = -255;

      sendCmd("PWM," + l + "," + r);
    }

    function down(ev) {
      ev.preventDefault();
      active = true;
      pointerId = (ev.pointerId !== undefined) ? ev.pointerId : null;
      if (base.setPointerCapture && ev.pointerId !== undefined) {
        try { base.setPointerCapture(ev.pointerId); } catch (_) {}
      }
      update(ev.clientX, ev.clientY);
      startHold();
    }

    function move(ev) {
      if (!active) return;
      if (pointerId !== null && ev.pointerId !== undefined && ev.pointerId !== pointerId) return;
      ev.preventDefault();
      update(ev.clientX, ev.clientY);
    }

    function up(ev) {
      if (!active) return;
      ev.preventDefault();
      active = false;
      pointerId = null;
      resetKnob();
    }

    function touchPoint(ev) {
      var t = (ev.touches && ev.touches[0]) || (ev.changedTouches && ev.changedTouches[0]);
      if (!t) return null;
      return { x: t.clientX, y: t.clientY };
    }

    function touchDown(ev) {
      var p = touchPoint(ev);
      if (!p) return;
      ev.preventDefault();
      active = true;
      update(p.x, p.y);
      startHold();
    }

    function touchMove(ev) {
      if (!active) return;
      var p = touchPoint(ev);
      if (!p) return;
      ev.preventDefault();
      update(p.x, p.y);
    }

    function touchUp(ev) {
      if (!active) return;
      ev.preventDefault();
      active = false;
      resetKnob();
    }

    function mouseDown(ev) {
      ev.preventDefault();
      active = true;
      update(ev.clientX, ev.clientY);
      startHold();
    }

    function mouseMove(ev) {
      if (!active) return;
      ev.preventDefault();
      update(ev.clientX, ev.clientY);
    }

    function mouseUp(ev) {
      if (!active) return;
      if (ev) ev.preventDefault();
      active = false;
      resetKnob();
    }

    base.addEventListener("pointerdown", down);
    base.addEventListener("pointermove", move);
    base.addEventListener("pointerup", up);
    base.addEventListener("pointercancel", up);
    base.addEventListener("lostpointercapture", up);
    base.addEventListener("touchstart", touchDown, { passive: false });
    base.addEventListener("touchmove", touchMove, { passive: false });
    base.addEventListener("touchend", touchUp, { passive: false });
    base.addEventListener("touchcancel", touchUp, { passive: false });
    base.addEventListener("mousedown", mouseDown);
    window.addEventListener("mousemove", mouseMove);
    window.addEventListener("mouseup", mouseUp);
  }

  function init() {
    try {
      bindSwitches();
      bindStick("joy-stick-base", "joy-stick-knob", "joy-stick-state");
      bindStick("follow-joy-stick-base", "follow-joy-stick-knob", "follow-joy-stick-state");
      bindStick("assist-joy-stick-base", "assist-joy-stick-knob", "assist-joy-stick-state", { smoothFromZero: true });
    } catch (e) {
      // Hard fallback: keep default button mode.
      setStyleMode(false);
      console.warn("emer_stick disabled:", e);
    }
  }

  if (document.readyState === "loading") {
    document.addEventListener("DOMContentLoaded", init);
  } else {
    init();
  }
})();
