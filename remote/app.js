"use strict";

// Standard Nordic UART Service. TX carries the robot's log output (notify).
const NUS_SERVICE = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
const NUS_TX = "6e400003-b5a3-f393-e0a9-e50e24dcca9e";
const DEVICE_NAME = "LineFollower";
const MAX_LINES = 2000;

const statusEl = document.getElementById("status");
const logEl = document.getElementById("log");
const connectBtn = document.getElementById("connectBtn");
const disconnectBtn = document.getElementById("disconnectBtn");
const clearBtn = document.getElementById("clearBtn");

let device = null;
let txChar = null;
// The TX stream is a byte stream, not one notification per line: a line may
// span notifications and a notification may hold several lines. Decode with
// a streaming decoder and split on \n, keeping any trailing partial.
const decoder = new TextDecoder("utf-8");
let lineBuffer = "";

function setStatus(text, kind) {
  statusEl.textContent = text;
  statusEl.className = kind || "";
}

function appendLine(text) {
  const stick =
    logEl.scrollTop + logEl.clientHeight >= logEl.scrollHeight - 4;
  const el = document.createElement("span");
  el.className = "line";
  el.textContent = text;
  logEl.appendChild(el);
  while (logEl.childElementCount > MAX_LINES) {
    logEl.removeChild(logEl.firstChild);
  }
  if (stick) logEl.scrollTop = logEl.scrollHeight;
}

function onNotification(event) {
  lineBuffer += decoder.decode(event.target.value, { stream: true });
  const parts = lineBuffer.split("\n");
  lineBuffer = parts.pop(); // trailing partial (or "" if it ended on \n)
  for (const line of parts) appendLine(line);
}

function onDisconnected() {
  setStatus("Disconnected");
  connectBtn.disabled = false;
  disconnectBtn.disabled = true;
  txChar = null;
  lineBuffer = "";
}

async function connect() {
  if (!navigator.bluetooth) {
    setStatus("Web Bluetooth not supported in this browser", "error");
    return;
  }
  try {
    setStatus("Requesting device...");
    connectBtn.disabled = true;
    device = await navigator.bluetooth.requestDevice({
      filters: [{ name: DEVICE_NAME }],
      optionalServices: [NUS_SERVICE],
    });
    device.addEventListener("gattserverdisconnected", onDisconnected);

    setStatus("Connecting...");
    const server = await device.gatt.connect();
    const service = await server.getPrimaryService(NUS_SERVICE);
    txChar = await service.getCharacteristic(NUS_TX);
    txChar.addEventListener("characteristicvaluechanged", onNotification);
    await txChar.startNotifications();

    setStatus("Connected to " + (device.name || "device"), "connected");
    disconnectBtn.disabled = false;
  } catch (err) {
    // Includes the user cancelling the chooser (NotFoundError).
    setStatus("Error: " + err.message, "error");
    connectBtn.disabled = false;
    if (device && device.gatt && device.gatt.connected) {
      device.gatt.disconnect();
    }
  }
}

function disconnect() {
  if (device && device.gatt && device.gatt.connected) {
    device.gatt.disconnect(); // fires gattserverdisconnected -> onDisconnected
  } else {
    onDisconnected();
  }
}

connectBtn.addEventListener("click", connect);
disconnectBtn.addEventListener("click", disconnect);
clearBtn.addEventListener("click", () => {
  logEl.replaceChildren();
});

// Register the service worker so the app is installable and works offline.
// Without an active worker controlling the page, Chrome won't offer to
// install it as a PWA.
if ("serviceWorker" in navigator) {
  window.addEventListener("load", () => {
    navigator.serviceWorker.register("sw.js").catch((err) => {
      console.error("Service worker registration failed:", err);
    });
  });
}
