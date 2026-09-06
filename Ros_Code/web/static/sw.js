const CACHE_VERSION = 'lawnmower-offline-20260531-1';
const APP_SHELL_URLS = [
  '/offline',
  '/manifest.webmanifest',
  '/static/js/emer_stick.js?v=20260510_pwmmap2',
  '/static/icons/logo-120.png',
  '/static/icons/logo-152.png',
  '/static/icons/logo-180.png',
  '/static/icons/logo-192.png',
  '/static/icons/logo-512.png'
];

async function cacheAppShell() {
  const cache = await caches.open(CACHE_VERSION);
  await Promise.all(APP_SHELL_URLS.map(async (url) => {
    try {
      const response = await fetch(url, { cache: 'reload' });
      if (response && response.ok) {
        await cache.put(url, response);
      }
    } catch (err) {
      // Some optional shell assets may be unavailable during install.
    }
  }));
}

self.addEventListener('install', (event) => {
  event.waitUntil(cacheAppShell().then(() => self.skipWaiting()));
});

self.addEventListener('activate', (event) => {
  event.waitUntil((async () => {
    const keys = await caches.keys();
    await Promise.all(keys.map((key) => {
      if (key !== CACHE_VERSION) return caches.delete(key);
      return Promise.resolve();
    }));
    await self.clients.claim();
  })());
});

self.addEventListener('message', (event) => {
  if (event.data && event.data.type === 'SKIP_WAITING') {
    self.skipWaiting();
  }
});

async function navigationFallback(request) {
  const cache = await caches.open(CACHE_VERSION);
  try {
    const response = await fetch(request);
    if (response.status >= 500) {
      throw new Error(`Server error ${response.status}`);
    }
    return response;
  } catch (err) {
    return (await cache.match('/offline')) || (await cache.match('/')) || new Response('Offline', {
      status: 503,
      headers: { 'Content-Type': 'text/plain; charset=utf-8' }
    });
  }
}

async function staticFallback(request) {
  const cached = await caches.match(request);
  if (cached) return cached;
  const response = await fetch(request);
  if (response && response.ok) {
    const cache = await caches.open(CACHE_VERSION);
    await cache.put(request, response.clone());
  }
  return response;
}

self.addEventListener('fetch', (event) => {
  const request = event.request;
  if (request.method !== 'GET') return;

  const url = new URL(request.url);
  if (request.mode === 'navigate') {
    event.respondWith(navigationFallback(request));
    return;
  }

  if (url.origin === self.location.origin) {
    if (url.pathname.startsWith('/api/') ||
        url.pathname === '/status' ||
        url.pathname === '/video_feed' ||
        url.pathname === '/video_frame.jpg' ||
        url.pathname.startsWith('/dataset/')) {
      return;
    }
    event.respondWith(staticFallback(request));
  }
});
