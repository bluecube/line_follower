// Network-first app-shell cache. When online we always try the network so the
// newest code loads without any manual version bump, and we refresh the cached
// copy on the way through. When offline we fall back to the last cached version,
// so the app stays installable and usable with no internet (all robot data
// arrives over BLE, never the network). The first visit must be online to
// populate the cache.
const CACHE = "lf-remote-v1";
const ASSETS = [
  "./",
  "./index.html",
  "./styles.css",
  "./app.js",
  "./manifest.webmanifest",
  "./icon.svg",
];

self.addEventListener("install", (event) => {
  // Pre-cache the shell so even a first-load-then-offline works.
  event.waitUntil(caches.open(CACHE).then((cache) => cache.addAll(ASSETS)));
  self.skipWaiting();
});

self.addEventListener("activate", (event) => {
  event.waitUntil(
    caches
      .keys()
      .then((keys) =>
        Promise.all(keys.filter((k) => k !== CACHE).map((k) => caches.delete(k)))
      )
      .then(() => self.clients.claim())
  );
});

// Network-first for same-origin GETs: try the network, cache a fresh copy, and
// fall back to the cache only when the network is unavailable.
self.addEventListener("fetch", (event) => {
  const req = event.request;
  if (req.method !== "GET" || new URL(req.url).origin !== self.location.origin) {
    return;
  }
  event.respondWith(
    fetch(req)
      .then((res) => {
        // Cache a clone of successful responses for offline use.
        if (res.ok) {
          const copy = res.clone();
          caches.open(CACHE).then((cache) => cache.put(req, copy));
        }
        return res;
      })
      .catch(() => caches.match(req).then((hit) => hit || caches.match("./")))
  );
});
