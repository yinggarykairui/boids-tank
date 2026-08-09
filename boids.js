/* boids-tank — flocking simulation core.
 *
 * Classic script (no modules) so index.html and tests.html both work from
 * file:// with zero build step. Everything hangs off the global `Boids`.
 *
 * The maths lives in small pure functions at the top; the stateful bits
 * (World) sit underneath them and allocate nothing per frame.
 */
var Boids = (function () {
  'use strict';

  var TAU = Math.PI * 2;
  var EPS = 1e-9;

  /* ---------- pure helpers ---------- */

  /* Deterministic RNG so a warmed-up flock and the tests are reproducible. */
  function makeRng(seed) {
    var a = (seed >>> 0) || 1;
    return function () {
      a |= 0;
      a = (a + 0x6d2b79f5) | 0;
      var t = Math.imul(a ^ (a >>> 15), 1 | a);
      t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
  }

  /* Shortest signed difference on a circle of circumference `size`. */
  function wrapDelta(d, size) {
    if (!(size > 0) || !isFinite(d)) return 0;
    var half = size / 2;
    d = d % size;
    if (d > half) d -= size;
    else if (d < -half) d += size;
    return d;
  }

  /* Fold a coordinate back into [0, size). */
  function wrapCoord(p, size) {
    if (!(size > 0) || !isFinite(p)) return 0;
    p = p % size;
    if (p < 0) p += size;
    /* p === size can happen for tiny negatives after rounding. */
    if (p >= size) p = 0;
    return p;
  }

  /* Squared toroidal distance between two points. */
  function torusDist2(ax, ay, bx, by, w, h) {
    var dx = wrapDelta(bx - ax, w);
    var dy = wrapDelta(by - ay, h);
    return dx * dx + dy * dy;
  }

  /* Clamp a vector's magnitude, writing into `out` ({x,y}). */
  function limitVec(x, y, max, out) {
    out = out || { x: 0, y: 0 };
    var m2 = x * x + y * y;
    if (!isFinite(m2) || m2 <= EPS) {
      out.x = 0;
      out.y = 0;
      return out;
    }
    if (m2 > max * max) {
      var s = max / Math.sqrt(m2);
      out.x = x * s;
      out.y = y * s;
    } else {
      out.x = x;
      out.y = y;
    }
    return out;
  }

  /* Rescale a vector to exactly `mag`; zero-length input yields zero. */
  function setMag(x, y, mag, out) {
    out = out || { x: 0, y: 0 };
    var m2 = x * x + y * y;
    if (!isFinite(m2) || m2 <= EPS) {
      out.x = 0;
      out.y = 0;
      return out;
    }
    var s = mag / Math.sqrt(m2);
    out.x = x * s;
    out.y = y * s;
    return out;
  }

  function clamp(v, lo, hi) {
    return v < lo ? lo : v > hi ? hi : v;
  }

  /* Tank-size dependent constants, so a phone tank and a desktop tank both
   * flock instead of one being a soup and the other a scatter of loners. */
  function deriveTuning(width, height, count) {
    var w = Math.max(1, width || 1);
    var h = Math.max(1, height || 1);
    var n = Math.max(1, count || 1);
    var spacing = Math.sqrt((w * h) / n);
    var perception = clamp(spacing * 2.2, 38, 120);
    var maxSpeed = clamp(Math.sqrt(w * h) / 8, 58, 150);
    return {
      perception: perception,
      sepRadius: perception * 0.38,
      maxSpeed: maxSpeed,
      minSpeed: maxSpeed * 0.55,
      maxForce: maxSpeed * 2.1
    };
  }

  /* Centre of mass on a torus, via the circular mean of each axis.
   * A plain arithmetic mean lands in the wrong place whenever the flock
   * straddles a seam, which is exactly when scatter looks wrong. */
  function torusCentroid(boids, width, height, out) {
    out = out || { x: 0, y: 0 };
    var n = boids.length;
    if (n === 0 || !(width > 0) || !(height > 0)) {
      out.x = width / 2 || 0;
      out.y = height / 2 || 0;
      return out;
    }
    var cx = 0, sx = 0, cy = 0, sy = 0, used = 0;
    for (var i = 0; i < n; i++) {
      var b = boids[i];
      if (!isFinite(b.x) || !isFinite(b.y)) continue;
      var ax = (wrapCoord(b.x, width) / width) * TAU;
      var ay = (wrapCoord(b.y, height) / height) * TAU;
      cx += Math.cos(ax); sx += Math.sin(ax);
      cy += Math.cos(ay); sy += Math.sin(ay);
      used++;
    }
    if (used === 0) {
      out.x = width / 2;
      out.y = height / 2;
      return out;
    }
    /* Degenerate (perfectly uniform) axes fall back to the tank centre. */
    out.x = (cx * cx + sx * sx) < 1e-8
      ? width / 2
      : wrapCoord(((Math.atan2(-sx / used, -cx / used) + Math.PI) / TAU) * width, width);
    out.y = (cy * cy + sy * sy) < 1e-8
      ? height / 2
      : wrapCoord(((Math.atan2(-sy / used, -cy / used) + Math.PI) / TAU) * height, height);
    return out;
  }

  /* Mean toroidal distance from the flock centre — the dispersion measure. */
  function meanSpread(boids, width, height) {
    var n = boids.length;
    if (n === 0) return 0;
    var c = torusCentroid(boids, width, height);
    var sum = 0;
    for (var i = 0; i < n; i++) {
      sum += Math.sqrt(torusDist2(c.x, c.y, boids[i].x, boids[i].y, width, height));
    }
    return sum / n;
  }

  /* Exponential relaxation of the burst speed cap back to the cruise cap. */
  function relaxCap(cap, base, dt, tau) {
    if (!isFinite(cap)) return base;
    if (!(tau > 0) || !(dt > 0)) return cap;
    return base + (cap - base) * Math.exp(-dt / tau);
  }

  /* ---------- world ---------- */

  var DEFAULT_PARAMS = { cohesion: 0.5, alignment: 0.62, separation: 0.55 };
  var BURST_CAP = 3.2;      /* multiple of maxSpeed allowed right after scatter */
  var BURST_TAU = 1.1;      /* seconds for that headroom to decay away */
  var SEP_GAIN = 1.7;       /* separation needs more authority to be legible */
  var MAX_DT = 1 / 30;      /* never integrate a step bigger than this */

  function World(width, height, count, seed) {
    this.width = Math.max(1, width);
    this.height = Math.max(1, height);
    this.count = Math.max(0, count | 0);
    this.rng = makeRng(seed === undefined ? 0x5eed : seed);
    this.params = {
      cohesion: DEFAULT_PARAMS.cohesion,
      alignment: DEFAULT_PARAMS.alignment,
      separation: DEFAULT_PARAMS.separation
    };
    this.tuning = deriveTuning(this.width, this.height, this.count);
    this.speedCap = this.tuning.maxSpeed;
    this.boids = [];

    var n = this.count;
    for (var i = 0; i < n; i++) {
      var ang = this.rng() * TAU;
      var sp = this.tuning.minSpeed + this.rng() * (this.tuning.maxSpeed - this.tuning.minSpeed);
      this.boids.push({
        x: this.rng() * this.width,
        y: this.rng() * this.height,
        vx: Math.cos(ang) * sp,
        vy: Math.sin(ang) * sp
      });
    }

    /* Scratch buffers — allocated once, reused every step, never grown. */
    this._alloc(n);
    this._c = { x: 0, y: 0 };
    this._t1 = { x: 0, y: 0 };
    this._t2 = { x: 0, y: 0 };
  }

  World.prototype._alloc = function (n) {
    this.cohX = new Float64Array(n);
    this.cohY = new Float64Array(n);
    this.aliX = new Float64Array(n);
    this.aliY = new Float64Array(n);
    this.sepX = new Float64Array(n);
    this.sepY = new Float64Array(n);
    this.nbr = new Float64Array(n);
    this.snbr = new Float64Array(n);
  };

  World.prototype.setParams = function (p) {
    if (!p) return;
    if (typeof p.cohesion === 'number') this.params.cohesion = clamp(p.cohesion, 0, 1);
    if (typeof p.alignment === 'number') this.params.alignment = clamp(p.alignment, 0, 1);
    if (typeof p.separation === 'number') this.params.separation = clamp(p.separation, 0, 1);
  };

  /* Resize the tank, carrying the flock with it in relative coordinates so
   * nothing is ever left outside the field. */
  World.prototype.resize = function (width, height) {
    var w = Math.max(1, width);
    var h = Math.max(1, height);
    var sx = w / this.width;
    var sy = h / this.height;
    if (!isFinite(sx) || !isFinite(sy) || (sx === 1 && sy === 1)) {
      this.width = w;
      this.height = h;
      return;
    }
    for (var i = 0; i < this.boids.length; i++) {
      var b = this.boids[i];
      b.x = wrapCoord(b.x * sx, w);
      b.y = wrapCoord(b.y * sy, h);
    }
    this.width = w;
    this.height = h;
    var prevMax = this.tuning.maxSpeed;
    this.tuning = deriveTuning(w, h, this.count);
    /* Keep speeds in the new tank's units, and keep any burst headroom. */
    var k = prevMax > EPS ? this.tuning.maxSpeed / prevMax : 1;
    if (isFinite(k) && k > 0) {
      for (var j = 0; j < this.boids.length; j++) {
        this.boids[j].vx *= k;
        this.boids[j].vy *= k;
      }
      this.speedCap = clamp(this.speedCap * k, this.tuning.maxSpeed,
        this.tuning.maxSpeed * BURST_CAP);
    } else {
      this.speedCap = this.tuning.maxSpeed;
    }
  };

  /* One fixed integration step. dt is seconds and is clamped internally. */
  World.prototype.step = function (dt) {
    if (!isFinite(dt) || dt <= 0) return;
    if (dt > MAX_DT) dt = MAX_DT;

    var n = this.count;
    var b = this.boids;
    var W = this.width;
    var H = this.height;
    var T = this.tuning;
    var per2 = T.perception * T.perception;
    var sep2 = T.sepRadius * T.sepRadius;

    var cohX = this.cohX, cohY = this.cohY;
    var aliX = this.aliX, aliY = this.aliY;
    var sepX = this.sepX, sepY = this.sepY;
    var nbr = this.nbr, snbr = this.snbr;

    var i, j;
    for (i = 0; i < n; i++) {
      cohX[i] = 0; cohY[i] = 0;
      aliX[i] = 0; aliY[i] = 0;
      sepX[i] = 0; sepY[i] = 0;
      nbr[i] = 0; snbr[i] = 0;
    }

    /* Pairwise once, applied to both members — halves the work at n=140. */
    for (i = 0; i < n; i++) {
      var bi = b[i];
      for (j = i + 1; j < n; j++) {
        var bj = b[j];
        var dx = wrapDelta(bj.x - bi.x, W);
        var dy = wrapDelta(bj.y - bi.y, H);
        var d2 = dx * dx + dy * dy;
        if (d2 >= per2 || d2 <= EPS) continue;   /* guard: coincident pairs */
        nbr[i]++; nbr[j]++;
        cohX[i] += dx; cohY[i] += dy;
        cohX[j] -= dx; cohY[j] -= dy;
        aliX[i] += bj.vx; aliY[i] += bj.vy;
        aliX[j] += bi.vx; aliY[j] += bi.vy;
        if (d2 < sep2) {
          var inv = 1 / d2;                      /* d2 > EPS, safe */
          sepX[i] -= dx * inv; sepY[i] -= dy * inv;
          sepX[j] += dx * inv; sepY[j] += dy * inv;
          snbr[i]++; snbr[j]++;
        }
      }
    }

    var wCoh = this.params.cohesion;
    var wAli = this.params.alignment;
    var wSep = this.params.separation * SEP_GAIN;
    var maxSpeed = T.maxSpeed;
    var maxForce = T.maxForce;
    var t1 = this._t1, t2 = this._t2;

    this.speedCap = relaxCap(this.speedCap, maxSpeed, dt, BURST_TAU);
    var cap = clamp(this.speedCap, maxSpeed, maxSpeed * BURST_CAP);
    this.speedCap = cap;

    for (i = 0; i < n; i++) {
      var p = b[i];
      var ax = 0, ay = 0;
      var cnt = nbr[i];

      if (cnt > 0) {                              /* guard: no neighbours */
        var invCnt = 1 / cnt;
        if (wCoh > 0) {
          setMag(cohX[i] * invCnt, cohY[i] * invCnt, maxSpeed, t1);
          limitVec(t1.x - p.vx, t1.y - p.vy, maxForce, t2);
          ax += t2.x * wCoh; ay += t2.y * wCoh;
        }
        if (wAli > 0) {
          setMag(aliX[i] * invCnt, aliY[i] * invCnt, maxSpeed, t1);
          limitVec(t1.x - p.vx, t1.y - p.vy, maxForce, t2);
          ax += t2.x * wAli; ay += t2.y * wAli;
        }
      }
      if (wSep > 0 && snbr[i] > 0) {              /* guard: nobody too close */
        setMag(sepX[i], sepY[i], maxSpeed, t1);
        limitVec(t1.x - p.vx, t1.y - p.vy, maxForce, t2);
        ax += t2.x * wSep; ay += t2.y * wSep;
      }

      var vx = p.vx + ax * dt;
      var vy = p.vy + ay * dt;
      if (!isFinite(vx) || !isFinite(vy)) { vx = 0; vy = 0; }

      var sp2 = vx * vx + vy * vy;
      if (sp2 <= EPS) {
        /* Never let a boid stall into a divide-by-zero heading. */
        var a0 = i * 2.399963229728653;
        vx = Math.cos(a0) * T.minSpeed;
        vy = Math.sin(a0) * T.minSpeed;
      } else {
        var sp = Math.sqrt(sp2);
        if (sp > cap) { vx = (vx / sp) * cap; vy = (vy / sp) * cap; }
        else if (sp < T.minSpeed) { vx = (vx / sp) * T.minSpeed; vy = (vy / sp) * T.minSpeed; }
      }

      p.vx = vx;
      p.vy = vy;
      p.x = wrapCoord(p.x + vx * dt, W);
      p.y = wrapCoord(p.y + vy * dt, H);
    }
  };

  /* Outward impulse from the flock's centre; the cap opens to let it show,
   * then relaxes so the flock can gather itself again. */
  World.prototype.scatter = function (strength) {
    var s = typeof strength === 'number' ? strength : 2.4;
    var c = torusCentroid(this.boids, this.width, this.height, this._c);
    var push = this.tuning.maxSpeed * s;
    for (var i = 0; i < this.boids.length; i++) {
      var p = this.boids[i];
      var dx = wrapDelta(p.x - c.x, this.width);
      var dy = wrapDelta(p.y - c.y, this.height);
      var d2 = dx * dx + dy * dy;
      var ux, uy;
      if (d2 <= 1e-6) {                            /* guard: boid at centre */
        var a = i * 2.399963229728653;
        ux = Math.cos(a); uy = Math.sin(a);
      } else {
        var inv = 1 / Math.sqrt(d2);
        ux = dx * inv; uy = dy * inv;
      }
      p.vx += ux * push;
      p.vy += uy * push;
    }
    this.speedCap = this.tuning.maxSpeed * BURST_CAP;
  };

  World.prototype.spread = function () {
    return meanSpread(this.boids, this.width, this.height);
  };

  /* Run the sim forward before the first paint so the flock is already a
   * flock when the page loads, rather than a cloud of confetti. */
  World.prototype.warmUp = function (seconds, stepDt) {
    var dt = stepDt || 1 / 60;
    var steps = Math.max(0, Math.round((seconds || 0) / dt));
    for (var i = 0; i < steps; i++) this.step(dt);
  };

  return {
    TAU: TAU,
    DEFAULT_PARAMS: DEFAULT_PARAMS,
    BURST_CAP: BURST_CAP,
    makeRng: makeRng,
    wrapDelta: wrapDelta,
    wrapCoord: wrapCoord,
    torusDist2: torusDist2,
    limitVec: limitVec,
    setMag: setMag,
    clamp: clamp,
    deriveTuning: deriveTuning,
    torusCentroid: torusCentroid,
    meanSpread: meanSpread,
    relaxCap: relaxCap,
    World: World
  };
})();

if (typeof module === 'object' && module.exports) module.exports = Boids;
