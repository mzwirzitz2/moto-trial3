// Moto Trial — core physics + levels. Works in the browser and in node (for tests).
(function (root) {
  'use strict';

  const G = 1200;            // gravity px/s²
  const GAS = 950;           // engine acceleration
  const BRAKE = 1700;        // brake deceleration
  const ROLL = 40;           // rolling resistance
  const AIR = 7e-4;          // air drag coefficient
  const IMPACT_MAX = 1100;   // max normal speed on landing
  const LAND_ANGLE_MAX = 70 * Math.PI / 180;
  const ASSIST = 1.0;
  const TRAMP_V = 1000;      // trampoline launch speed
  const CONV_ACC = 1500, CONV_MAX = 1250;
  const REV_ACC = 650, REV_MAX = 320;   // reverse gear
  const HEAD = 62, WHEEL_R = 14, WHEEL_X = 26;
  const SLING = { mid: 0.8, amp: 0.5, aimSpeed: 1.4, pmin: 500, pmax: 1500, powerTime: 1.2 };
  const AIM = { time: 3.5, turn: 0.7, min: 0.15, max: 1.3 };
  const PI = Math.PI;

  // ---------- path helpers ----------
  // points are [x, y, kind, conveyorDir]; the kind belongs to the segment ending at that point
  function Path(x, y) {
    let kind = 'normal', conv = 0;
    const pts = [[x, y, kind, conv]];
    const step = 7;
    const last = () => pts[pts.length - 1];
    const push = (px, py) => pts.push([px, py, kind, conv]);
    const api = {
      kind(k, c) { kind = k; conv = c || 0; return api; },
      line(x2, y2) {
        const [x1, y1] = last();
        const n = Math.max(1, Math.ceil(Math.hypot(x2 - x1, y2 - y1) / step));
        for (let i = 1; i <= n; i++) push(x1 + (x2 - x1) * i / n, y1 + (y2 - y1) * i / n);
        return api;
      },
      cubic(c1x, c1y, c2x, c2y, x2, y2) {
        const [x0, y0] = last();
        const est = Math.hypot(c1x - x0, c1y - y0) + Math.hypot(c2x - c1x, c2y - c1y) + Math.hypot(x2 - c2x, y2 - c2y);
        const n = Math.max(2, Math.ceil(est / step));
        for (let i = 1; i <= n; i++) {
          const t = i / n, m = 1 - t;
          push(m * m * m * x0 + 3 * m * m * t * c1x + 3 * m * t * t * c2x + t * t * t * x2,
            m * m * m * y0 + 3 * m * m * t * c1y + 3 * m * t * t * c2y + t * t * t * y2);
        }
        return api;
      },
      arc(cx, cy, r, a0, a1) {
        const n = Math.max(2, Math.ceil(Math.abs(a1 - a0) * r / step));
        for (let i = 1; i <= n; i++) {
          const a = a0 + (a1 - a0) * i / n;
          push(cx + r * Math.cos(a), cy + r * Math.sin(a));
        }
        return api;
      },
      fn(x2, f) { // y = f(x) from current x to x2
        const [x1] = last();
        const n = Math.max(2, Math.ceil(Math.abs(x2 - x1) / 6));
        for (let i = 1; i <= n; i++) { const x = x1 + (x2 - x1) * i / n; push(x, f(x)); }
        return api;
      },
      pts,
    };
    return api;
  }

  function fillSegs(g) {
    const segs = [], side = g.side, pts = g.pts;
    for (let i = 0; i < pts.length - 1; i++) {
      const [ax, ay] = pts[i], [bx, by, kind, conv] = pts[i + 1];
      const len = Math.hypot(bx - ax, by - ay);
      if (len < 1e-6) continue;
      const tx = (bx - ax) / len, ty = (by - ay) / len;
      segs.push({ ax, ay, bx, by, len, tx, ty, nx: side * ty, ny: -side * tx, kind, conv });
    }
    g.segs = segs;
  }

  // side: +1 → ground is below the bike while travelling in +x
  function makeGround(pts, side, opts) {
    const g = Object.assign({ pts: pts.map(p => p.slice()), base: pts.map(p => p.slice()), side,
      ox: 0, oy: 0, rot: 0, pivot: [pts[0][0], pts[0][1]], vx: 0, vy: 0 }, opts || {});
    fillSegs(g);
    return g;
  }

  function transformGround(g) {
    const c = Math.cos(g.rot), s = Math.sin(g.rot), [px, py] = g.pivot;
    g.pts = g.base.map(([x, y, k, cv]) => {
      const dx = x - px, dy = y - py;
      return [px + dx * c - dy * s + g.ox, py + dx * s + dy * c + g.oy, k, cv];
    });
    fillSegs(g);
  }

  function groundYAt(g, x) {
    for (const s of g.segs) if ((x - s.ax) * (x - s.bx) <= 0 && Math.abs(s.tx) > 1e-6) return s.ay + (x - s.ax) / s.tx * s.ty;
    return null;
  }

  // shared helpers for level builders
  function ballistic(x, y, vx, vy, spacing, maxN, stopFn, out, drag, gf) {
    let px = x, py = y, acc = spacing * 0.6, n = 0;
    const dt = 1 / 240;
    for (let i = 0; i < 4000 && n < maxN; i++) {
      const nx = px + vx * dt, ny = py + vy * dt;
      vy += G * (gf || 1) * dt;
      if (drag) { const sp = Math.hypot(vx, vy); vx -= AIR * 0.5 * vx * sp * dt; vy -= AIR * 0.5 * vy * sp * dt; }
      acc += Math.hypot(nx - px, ny - py);
      px = nx; py = ny;
      if (stopFn(px, py)) break;
      if (acc >= spacing) { acc = 0; out.push({ x: px, y: py - 32 }); n++; }
    }
  }
  const W = (ax, ay, bx, by) => ({ ax, ay, bx, by });

  // ======================= LEVEL 1 =======================
  function level1() {
    const grounds = [];
    const p0 = Path(-470, 50).line(-270, 50).arc(-270, 400, 350, -PI / 2, -PI / 6).line(380, 825)
      .arc(640, 675, 300, PI * 5 / 6, PI / 2).line(680, 975).cubic(780, 975, 890, 794, 930, 725);
    grounds.push(makeGround(p0.pts, 1));
    const p1 = Path(1437, 1125).cubic(1470, 1260, 1560, 1405, 1775, 1405).line(2400, 1405);
    grounds.push(makeGround(p1.pts, 1));
    const p2 = Path(2600, 1405).line(2600, 1697).arc(2460, 1697, 140, 0, PI / 2).line(2340, 1837)
      .cubic(2300, 1837, 2280, 1780, 2240, 1740);
    grounds.push(makeGround(p2.pts, -1));
    const p3 = Path(2240, 1837).line(1990, 1837).cubic(1960, 1837, 1945, 1795, 1915, 1765);
    grounds.push(makeGround(p3.pts, -1));
    const p4 = Path(1915, 1837).line(1880, 1837).cubic(1850, 1837, 1830, 1772, 1790, 1772)
      .cubic(1750, 1772, 1730, 1837, 1700, 1837).line(1580, 1837).cubic(1545, 1837, 1515, 1815, 1490, 1790);
    grounds.push(makeGround(p4.pts, -1));
    const p5 = Path(1250, 2060).cubic(1238, 2130, 1200, 2200, 1120, 2200).line(560, 2200);
    grounds.push(makeGround(p5.pts, -1));

    const walls = [
      W(930, 733, 930, 975),
      W(1437, 1133, 1437, 1440), W(1437, 1440, 2400, 1440), W(2400, 1411, 2400, 1440),
      W(2240, 1748, 2240, 1837), W(1915, 1773, 1915, 1837),
      W(1490, 1798, 1490, 1870), W(1490, 1870, 2640, 1870),
      W(1250, 2068, 1250, 2320),
    ];

    const coins = [];
    { const s = grounds[0].segs[grounds[0].segs.length - 1], v = 850;
      ballistic(s.bx, s.by, s.tx * v, s.ty * v, 62, 14, (x, y) => y > 1180, coins); }
    for (let x = 1820; x <= 2360; x += 48) coins.push({ x, y: 1405 - 32 });
    ballistic(2400, 1405, 150, 0, 58, 6, (x, y) => y > 1760, coins);
    for (let x = 2440; x >= 2360; x -= 48) coins.push({ x, y: 1837 - 32 });
    { const s = grounds[4].segs[grounds[4].segs.length - 1], v = 440;
      ballistic(s.bx, s.by, s.tx * v, s.ty * v, 52, 8, (x, y) => y > 2090, coins); }
    for (let x = 1100; x >= 960; x -= 48) coins.push({ x, y: 2200 - 32 });

    return {
      name: 'Level 1', start: { gi: 0, x: -420 },
      grounds, walls, coins,
      spikes: [{ x: 2572, y: 1350, w: 56, h: 55, dir: 'up' }],
      gates: [{ x: 2025, w: 50, top: 1445, bottom: 1837, ceiling: 1440 }],
      buttons: [{ x: 2115, w: 95, y: 1837, target: { type: 'gate', i: 0 }, color: '#fdd835' }],
      movers: [], flaps: [], saws: [], fans: [],
      checkpoints: [{ gi: 2, x: 2440, tx: 2560, dir: -1 }],
      finish: { gi: 5, x: 880, dir: -1 },
      deathY: 2450,
      deathRects: [{ x: 930, y: 1460, w: 507, h: 240 }],
      decor: {
        hatch: [[1437, 1405, 963, 35], [2600, 1405, 45, 465], [1490, 1837, 1110, 33], [560, 2200, 690, 120]],
        hatchPolys: [
          { poly: [[1250, 2060], ...p5.pts, [560, 2320], [1250, 2320]], box: [560, 2050, 700, 270] },
          { poly: [...p4.pts.filter(p => p[0] <= 1880 && p[0] >= 1700), [1700, 1837], [1880, 1837]], box: [1690, 1760, 200, 80] },
        ],
        ramps: [
          [[700, 975], ...p0.pts.filter(p => p[0] >= 700), [930, 975]],
          [[1437, 1405], ...p1.pts.filter(p => p[0] <= 1775), [1437, 1405]],
          [[2330, 1837], ...p2.pts.filter(p => p[1] > 1700 && p[0] <= 2330), [2240, 1837]],
          [[1985, 1837], ...p3.pts.filter(p => p[0] <= 1985), [1915, 1837]],
          [[1575, 1837], ...p4.pts.filter(p => p[0] <= 1575), [1490, 1837]],
        ],
        texts: [{ x: -420, y: 0, t: 'START' }],
        warns: [],
        arrows: [{ x: 1995, y: 1610, up: true, untilButton: 0 }],
      },
    };
  }

  // ======================= LEVEL 2 =======================
  function level2() {
    const gr = [];
    const add = (path, side, opts) => { gr.push(makeGround(path.pts, side, opts)); return gr.length - 1; };
    const coins = [];
    const coinRow = (gi, x0, x1, step) => {
      for (let x = x0; step > 0 ? x <= x1 : x >= x1; x += step) { const y = groundYAt(gr[gi], x); if (y != null) coins.push({ x, y: y - 32 }); }
    };

    // ---- Row A (→): waves, conveyor, trampolines, shaft
    const a0 = add(Path(0, 0).line(250, 0).cubic(350, 0, 400, 150, 520, 150).cubic(640, 150, 700, 60, 820, 60).line(900, 60)
      .kind('conv', 1).cubic(1150, 90, 1400, 40, 1650, 40)
      .kind('normal').line(2150, 40).cubic(2300, 40, 2330, 280, 2470, 280).line(2500, 280)
      .kind('tramp').line(2580, 280), 1);
    const a1 = add(Path(3100, 280).line(3600, 280).kind('tramp').line(3680, 280).kind('normal').line(4320, 280), 1);

    // ---- Row B (←): button + elevator, conveyor + magnet loop, spike-bowl jump, folding bridges
    const b0 = add(Path(4640, 1000).line(3700, 1000), -1);
    const bLift = add(Path(3700, 1001).line(3500, 1001), -1, { color: '#ff8a3d' });
    const b1 = add(Path(3500, 1002).line(3420, 1002).kind('conv', 1).line(3150, 1002).kind('normal').line(2950, 1002)
      .kind('magnet').arc(2950, 822, 180, PI / 2, PI * 5 / 2).kind('normal').line(2700, 1002)
      .cubic(2650, 1002, 2635, 980, 2600, 960), -1);
    const b2 = add(Path(2350, 990).cubic(2300, 995, 2250, 1002, 2200, 1002).line(2050, 1002), -1);
    const flapG = [];
    for (let i = 0; i < 3; i++) {
      const x0 = 2050 - i * 200, y = 1003 + i;
      flapG.push(add(Path(x0, y).line(x0 - 200, y), -1, { color: '#8bc34a', pivot: [x0, y] }));
    }
    const b3 = add(Path(1450, 1006).line(1000, 1006), -1);

    // ---- Row C (→): bowl, saws, trampoline over saw, moving platform
    const c0 = add(Path(800, 1006).line(800, 1297).arc(940, 1297, 140, PI, PI / 2).line(2200, 1437)
      .kind('tramp', 750).line(2280, 1437).kind('normal').line(3000, 1437), 1);
    const cMove = add(Path(3000, 1438).line(3260, 1438), 1, { color: '#9c4dcc' });
    const c1 = add(Path(3500, 1439).line(3900, 1439).cubic(4100, 1439, 4250, 1950, 4600, 1950).line(4800, 1950)
      // ---- Row D (→): bumps under a spike ceiling, spiked stairs, conveyor + trampoline to the finish
      .fn(6000, x => 1950 - 16 * (1 - Math.cos(2 * PI * (x - 4800) / 220)))
      .line(6250, 1950), 1);
    let sp = Path(6250, 1950);
    for (let i = 0; i < 6; i++) { const y = 1950 + 70 * (i + 1); sp.line(6250 + 200 * i, y).line(6250 + 200 * (i + 1), y); }
    sp.line(7500, 2370).kind('conv', 1).line(7650, 2370).kind('tramp').line(7730, 2370).kind('normal').line(7750, 2370);
    const d1 = add(sp, 1);
    const d2 = add(Path(8050, 2140).line(9300, 2140), 1);

    const walls = [
      W(2580, 286, 2580, 520), W(3100, 286, 3100, 520),
      W(4320, 286, 4320, 820), W(4640, 0, 4640, 1000),
      W(3700, 1006, 3700, 1350), W(3500, 1008, 3500, 1350),
      W(2600, 968, 2600, 1120), W(2350, 996, 2350, 1120),
      W(2050, 1009, 2050, 1170), W(1450, 1012, 1450, 1170),
      W(3000, 1443, 3000, 1700), W(3500, 1445, 3500, 1700),
      W(8050, 2146, 8050, 2400)
    ];

    const spikes = [
      { x: 2600, y: 455, w: 480, h: 50, dir: 'up' },          // pit under trampoline 1
      { x: 3760, y: 160, w: 60, h: 120, dir: 'up' },          // spike block between trampolines
      { x: 4600, y: 380, w: 40, h: 520, dir: 'left' },        // shaft wall
      { x: 2360, y: 1065, w: 230, h: 50, dir: 'up' },         // spike bowl
      { x: 1460, y: 1105, w: 580, h: 45, dir: 'up' },         // under folding bridges
      { x: 770, y: 950, w: 60, h: 56, dir: 'up' },            // top of bowl wall
      { x: 4850, y: 1800, w: 1100, h: 45, dir: 'down' },      // spike ceiling
      { x: 8020, y: 2170, w: 30, h: 200, dir: 'left' },       // finish wall
    ];
    for (let i = 0; i < 6; i++) spikes.push({ x: 6250 + 200 * i, y: 1950 + 70 * (i + 1) - 26, w: 60, h: 26, dir: 'up' });

    // coins
    coinRow(a0, 150, 850, 70);
    coinRow(a0, 950, 1600, 75);
    ballistic(2540, 280, 560, -TRAMP_V, 70, 9, (x, y) => y > 270 && x > 2700, coins, true);
    ballistic(3640, 280, 300, -TRAMP_V, 70, 9, (x, y) => y > 270 && x > 3700, coins, true);
    ballistic(4320, 280, 170, 0, 70, 7, (x, y) => y > 900, coins, true);
    coinRow(b0, 4400, 3900, -80);
    for (let a = PI * 0.75; a < PI * 2.25; a += PI / 6) coins.push({ x: 2950 + 140 * Math.cos(a), y: 822 + 140 * Math.sin(a) });
    { const s = gr[b1].segs[gr[b1].segs.length - 1]; ballistic(s.bx, s.by, s.tx * 560, s.ty * 560, 60, 5, (x, y) => y > 1000, coins, true); }
    coinRow(flapG[0], 2000, 1460, -90);
    coinRow(c0, 1000, 2150, 110);
    ballistic(2240, 1437, 420, -750, 70, 9, (x, y) => y > 1420 && x > 2400, coins, true);
    coins.push({ x: 3130, y: 1405 }, { x: 3250, y: 1405 }, { x: 3370, y: 1405 });
    coinRow(c1, 4900, 5950, 110);
    for (let i = 0; i < 6; i++) coins.push({ x: 6250 + 200 * i + 130, y: 1950 + 70 * (i + 1) - 32 });
    ballistic(7690, 2370, 520, -TRAMP_V, 80, 9, (x, y) => y > 2110 && x > 8050, coins, true);
    coinRow(d2, 8300, 8700, 80);

    return {
      name: 'Level 2', start: { gi: a0, x: 60 },
      grounds: gr, walls, coins, spikes,
      gates: [],
      buttons: [{ x: 3780, w: 90, y: 1000, target: { type: 'mover', i: 0 }, color: '#ff8a3d' }],
      movers: [
        { gi: bLift, from: [0, 320], to: [0, 0], speed: 240, mode: 'button' },
        { gi: cMove, from: [0, 0], to: [240, 0], speed: 120, mode: 'loop', dwell: 1.2 },
      ],
      flaps: flapG.map(gi => ({ gi, dir: 1, delay: 0.4 })),
      saws: [
        { cx: 1550, cy: 1290, orbit: 85, r: 36, speed: 1.8, phase: 0 },
        { cx: 1850, cy: 1290, orbit: 85, r: 36, speed: 1.8, phase: PI },
        { cx: 2500, cy: 1392, orbit: 0, r: 45, speed: 0, phase: 0 },
      ],
      fans: [{ x: 4320, y: 600, w: 320, h: 400, vt: 180 }],
      checkpoints: [
        { gi: b0, x: 4450, tx: 4600, dir: -1 },
        { gi: c0, x: 1000, tx: 900, dir: 1 },
        { gi: c1, x: 3600, tx: 3550, dir: 1 },
        { gi: d1, x: 7300, tx: 7250, dir: 1 },
      ],
      finish: { gi: d2, x: 8500, dir: 1 },
      deathY: 2700,
      deathRects: [{ x: 3500, y: 1080, w: 200, h: 260 }, { x: 3000, y: 1520, w: 500, h: 300 }],
      decor: {
        hatch: [[2560, 280, 20, 240], [3100, 280, 20, 240], [2560, 505, 560, 25],
          [4640, -40, 40, 1070], [4280, 286, 40, 534],
          [3700, 1000, 940, 30], [3700, 1000, 25, 350], [3475, 1002, 25, 350], [3475, 1350, 250, 15],
          [2350, 1115, 250, 20], [1450, 1150, 600, 22], [1000, 1006, 450, 30], [2050, 1002, 300, 30],
          [760, 1006, 40, 461], [940, 1437, 2060, 30], [3500, 1439, 400, 30],
          [4850, 1775, 1100, 30], [8050, 2140, 1250, 260]],
        hatchPolys: [],
        ramps: [[[2700, 1002], ...gr[b1].pts.filter(p => p[0] <= 2700), [2600, 1002]]],
        texts: [{ x: 60, y: -60, t: 'START' }],
        warns: [{ x: 4200, y: 200 }, { x: 1700, y: 1150 }, { x: 4700, y: 1880 }, { x: 6200, y: 1890 }],
        arrows: [],
      },
    };
  }

  // ======================= SECTION BUILDER (levels 3+) =======================
  // Every section travels to the right (+x). Each method appends to the current path or
  // starts a new one, and records hints (ideal speeds) used by the test bot.
  function Builder(name) {
    const B = {
      name, gr: [], walls: [], spikes: [], coins: [], saws: [], fans: [], flaps: [], movers: [], buttons: [],
      portals: [], waters: [], checkpoints: [], hints: [], deathRects: [], finish: null, start: null,
      decor: { hatch: [], hatchPolys: [], ramps: [], texts: [], warns: [], arrows: [], skirt: true },
      p: null, pending: [], deathY: 0,
    };
    const last = () => B.p.pts[B.p.pts.length - 1];
    const X = () => last()[0], Y = () => last()[1];
    const later = f => B.pending.push(f);
    const hint = h => { if (B.guard !== undefined) h.x0 = Math.max(h.x0, B.guard); B.hints.push(h); later(gi => { if (h.gi === undefined) h.gi = gi; }); return h; };
    const coinsFlat = (x0, x1, y, step) => { for (let x = x0 + step / 2; x < x1; x += step) B.coins.push({ x, y: y - 32 }); };
    const pit = (x0, x1, y, depth, lava) => {
      B.walls.push(W(x0, y + 8, x0, y + depth), W(x1, y + 8, x1, y + depth));
      if (lava || B.lavaPits) B.spikes.push({ x: x0, y: y + depth - 70, w: x1 - x0, h: 70, dir: 'lava' });
      else B.spikes.push({ x: x0 + 6, y: y + depth - 45, w: x1 - x0 - 12, h: 45, dir: 'up' });
      B.decor.hatch.push([x0 - 25, y, 25, depth + 25], [x1, y, 25, depth + 25], [x0 - 25, y + depth, x1 - x0 + 50, 25]);
    };
    // ideal launch speed so that the ballistic path (with drag) lands at targetX on height landY
    const idealSpeed = (x, y, dx, dy, landY, targetX, fixedVy, gf) => {
      const landsAt = v => {
        let px = x, py = y, vx = fixedVy !== undefined ? v : dx * v, vy = fixedVy !== undefined ? fixedVy : dy * v;
        for (let i = 0; i < 4000; i++) {
          const dt = 1 / 240; vy += G * (gf || 1) * dt; const sp = Math.hypot(vx, vy);
          vx -= AIR * 0.5 * vx * sp * dt; vy -= AIR * 0.5 * vy * sp * dt;
          px += vx * dt; py += vy * dt;
          if (vy > 0 && py >= landY) return px;
        }
        return px;
      };
      let lo = 50, hi = 3000;
      for (let i = 0; i < 40; i++) { const m = (lo + hi) / 2; if (landsAt(m) < targetX) lo = m; else hi = m; }
      return lo;
    };

    B.begin = (x, y) => { B.p = Path(x, y); return B; };
    B.end = () => {
      if (B.p && B.p.pts.length > 1) {
        B.gr.push(makeGround(B.p.pts, 1));
        const gi = B.gr.length - 1;
        for (const f of B.pending) f(gi);
        B.deathY = Math.max(B.deathY, ...B.p.pts.map(p => p[1] + 700));
      }
      B.p = null; B.pending = [];
      return B.gr.length - 1;
    };
    B.startHere = text => { const x = X(), y = Y(); B.start = { x: x + 60 }; later(gi => (B.start.gi = gi)); B.decor.texts.push({ x: x + 60, y: y - 70, t: text || 'START' }); return B; };
    B.flat = (len, coinStep) => { const x = X(), y = Y(); B.p.line(x + len, y); if (coinStep) coinsFlat(x, x + len, y, coinStep); return B; };
    B.slope = (dx, dy) => { const x = X(), y = Y(); B.p.cubic(x + dx * 0.45, y, x + dx * 0.55, y + dy, x + dx, y + dy); return B; };
    B.waves = (len, amp, wl) => {
      const x = X(), y = Y();
      B.p.fn(x + len, xx => y - amp * (1 - Math.cos(2 * PI * (xx - x) / wl)) / 2);
      for (let c = x + wl / 2; c < x + len; c += wl) B.coins.push({ x: c, y: y - amp - 34 });
      return B;
    };
    B.conv = len => { const x = X(), y = Y(); B.p.kind('conv', 1).line(x + len, y).kind('normal'); hint({ x0: x - 100, x1: x + len, max: true }); return B; };
    B.tramp = (len, power) => { const x = X(), y = Y(); B.p.kind('tramp', power || 0).line(x + (len || 80), y).kind('normal'); return B; };
    B.loop = (r, magnet) => {
      const x = X(), y = Y();
      hint({ x0: x - 500, x1: x + r + 10, max: true });
      B.p.kind(magnet ? 'magnet' : 'loop').arc(x, y - r, r, PI / 2, PI / 2 - 2 * PI).kind('normal');
      for (let a = -PI * 0.75; a < PI * 0.8; a += PI / 5) B.coins.push({ x: x + (r - 45) * Math.cos(a - PI / 2), y: y - r + (r - 45) * Math.sin(a - PI / 2) });
      if (B.special) B.coins.push({ x, y: y - 2 * r + 45, v: 5 });
      return B;
    };
    B.kickerGap = o => {
      o = Object.assign({ h: 50, angle: 40, gap: 260, drop: 0, landLen: 320, depth: 220, g: 1 }, o);
      const x = X(), y = Y(), A = o.angle * PI / 180, Lk = 130;
      const tx = x + Lk, ty = y - o.h;
      B.p.cubic(x + Lk * 0.4, y, tx - 40 * Math.cos(A), ty + 40 * Math.sin(A), tx, ty);
      const kick = B.p.pts.filter(p => p[0] >= x);
      B.decor.ramps.push([[x + 20, y], ...kick, [tx, y]]);
      const s = { tx: Math.cos(A), ty: -Math.sin(A) };
      let lx = tx + o.gap, ly = y + o.drop;
      let v = idealSpeed(tx, ty, s.tx, s.ty, ly, lx + o.landLen * 0.3, undefined, o.g);
      // keep the jump reachable: shrink the gap until the needed run-up speed is realistic
      const appr = vv => Math.sqrt(vv * vv + 2 * G * o.h + 2 * (ROLL + AIR * vv * vv) * Lk * 1.2) * 1.02;
      while (B.maxApproach && appr(v) > B.maxApproach && o.gap > 200) {
        o.gap -= 20; lx = tx + o.gap; v = idealSpeed(tx, ty, s.tx, s.ty, ly, lx + o.landLen * 0.3, undefined, o.g);
      }
      hint({ x0: x - 1000, x1: x - 5, kick: true, v: Math.sqrt(v * v + 2 * G * o.h + 2 * (ROLL + AIR * v * v) * Lk * 1.2) * 1.02 });
      hint({ x0: x - 5, x1: tx + 1, coast: true });
      const arc = [];
      ballistic(tx, ty, s.tx * v, s.ty * v, o.g < 1 ? 90 : 60, 24, (px, py) => py > ly - 5 && px > lx, arc, true, o.g);
      if (B.special && arc.length) { const top = arc.reduce((a, c) => (c.y < a.y ? c : a)); top.v = 5; }
      B.coins.push(...arc);
      if (o.ceiling !== false) { // spike ceiling just above the ideal flight path: too much speed = too high
        const margin = B.ceilMargin !== undefined ? B.ceilMargin : 28;
        let peak = ty, px = tx, vx = s.tx * v, vy = s.ty * v;
        for (let i = 0; i < 4000 && vy < 0; i++) { const d = 1 / 240; vy += G * o.g * d; const sp = Math.hypot(vx, vy); vx -= AIR * 0.5 * vx * sp * d; vy -= AIR * 0.5 * vy * sp * d; px += vx * d; peak += vy * d; }
        const cy = peak - HEAD - margin - 70 * (1 - o.g);
        B.spikes.push({ x: tx + 30, y: cy - 45, w: o.gap + 120, h: 45, dir: 'down' });
        B.decor.hatch.push([tx + 30, cy - 75, o.gap + 120, 30]);
      }
      if (o.g < 1) B.lowG.push({ x: tx - 20, y: Math.min(...arc.map(c => c.y)) - 400, w: o.gap + o.landLen * 0.8, h: 2000, g: o.g });
      B.end();
      B.walls.push(W(tx, ty + 8, tx, y + o.depth));
      pit(tx, lx, Math.max(y, ly), o.depth, o.lava);
      B.begin(lx, ly).flat(o.landLen);
      return B;
    };
    B.trampGap = o => {
      o = Object.assign({ gap: 420, power: 900, landLen: 380, depth: 220, g: 1 }, o);
      B.flat(60);
      const x = X(), y = Y();
      B.tramp(80, o.power);
      const ex = X(), lx = ex + o.gap;
      const v = idealSpeed(x + 2, y, 0, 0, y, lx + o.landLen * 0.35, -o.power, o.g);
      hint({ x0: x - 400, x1: ex, v });
      const arc = [];
      ballistic(x + 2, y, v, -o.power, o.g < 1 ? 95 : 70, 24, (px, py) => py > y - 5 && px > lx, arc, true, o.g);
      B.coins.push(...arc);
      if (o.g < 1) B.lowG.push({ x: x - 10, y: Math.min(...arc.map(c => c.y)) - 400, w: o.gap + 140 + o.landLen * 0.8, h: 2000, g: o.g });
      B.end();
      pit(ex, lx, y, o.depth, o.lava);
      B.begin(lx, y).flat(o.landLen);
      return B;
    };
    B.saws = (len, n, style) => {
      const x = X(), y = Y(), idx = [];
      for (let i = 0; i < n; i++) {
        idx.push(B.saws_.length);
        B.saws_.push({ cx: x + len * (i + 1) / (n + 1), cy: y - 147, orbit: 85, r: 36, speed: 1.8, phase: i * PI, style });
      }
      hint({ x0: x - 200, x1: x + len, saw: idx, v: 300 });
      B.decor.warns.push({ x: x - 60, y: y - 120 });
      return B.flat(len, 110);
    };
    B.spikeWheel = len => {
      const x = X(), y = Y(), i = B.saws_.length;
      B.saws_.push({ cx: x + len / 2, cy: y - 175, orbit: 95, r: 52, speed: 1.5, phase: 0, style: 'spike' });
      hint({ x0: x - 200, x1: x + len, saw: [i], v: 300 });
      return B.flat(len, 120);
    };
    B.flapsBridge = (n, w) => {
      n = n || 3; w = w || 200;
      const x = X(), y = Y();
      hint({ x0: x - 350, x1: x, v: 650 });
      B.end();
      for (let i = 0; i < n; i++) {
        const x0 = x + i * w, yy = y + 1 + i;
        B.gr.push(makeGround(Path(x0, yy).line(x0 + w, yy).pts, 1, { color: '#8bc34a', pivot: [x0, yy] }));
        const gi = B.gr.length - 1;
        B.flaps.push({ gi, dir: 1, delay: 0.4 });
        B.hints.push({ gi, x0, x1: x0 + w, v: 650 });
        coinsFlat(x0, x0 + w, yy, 100);
      }
      pit(x, x + n * w, y, 160);
      B.begin(x + n * w, y + n + 1).flat(250);
      return B;
    };
    B.stairs = (n, w, h) => {
      n = n || 5; w = w || 200; h = h || 70;
      const x = X(), y = Y();
      hint({ x0: x - 250, x1: x + n * w, v: 420 });
      B.decor.warns.push({ x: x - 50, y: y - 60 });
      for (let i = 0; i < n; i++) {
        const yy = y + h * (i + 1);
        B.p.line(x + w * i, yy).line(x + w * (i + 1), yy);
        B.spikes.push({ x: x + w * i, y: yy - 26, w: 60, h: 26, dir: 'up' });
        B.coins.push({ x: x + w * i + 130, y: yy - 32 });
      }
      return B;
    };
    B.ceilBumps = len => {
      const x = X(), y = Y();
      hint({ x0: x - 200, x1: x + len, v: 420 });
      B.waves(len, 32, 220);
      B.spikes.push({ x: x + 50, y: y - 32 - 62 - 58, w: len - 100, h: 45, dir: 'down' });
      B.decor.hatch.push([x + 50, y - 32 - 62 - 88, len - 100, 30]);
      return B;
    };
    B.pool = (len, depth, withTramp) => {
      if (withTramp) {
        B.flat(60);
        const tx = X(), ty = Y();
        hint({ x0: tx - 350, x1: tx, v: idealSpeed(tx + 2, ty, 0, 0, ty, tx + 80 + len + 150, -800) });
        B.tramp(80, 800);
      }
      const x = X(), y = Y();
      const run = Math.max(210, depth * 2.2);
      B.p.cubic(x + run * 0.5, y, x + run * 0.5, y + depth, x + run, y + depth).line(x + len - run, y + depth).cubic(x + len - run * 0.5, y + depth, x + len - run * 0.5, y, x + len, y);
      B.waters.push({ x: x + 10, y: y - 6, w: len - 20, h: depth + 6, poly: [[x, y - 6], ...B.p.pts.filter(q => q[0] >= x && q[0] <= x + len), [x + len, y - 6]] });
      if (!withTramp) hint({ x0: x - 100, x1: x + len, max: true });
      coinsFlat(x + run, x + len - run, y + depth, 90);
      return B;
    };
    B.elevator = h => {
      h = h || 420;
      const x = X(), y = Y();
      B.end();
      const pw = 200;
      B.gr.push(makeGround(Path(x - 8, y + 1).line(x + 4 + pw, y + 1).pts, 1, { color: '#ff8a3d' }));
      const gi = B.gr.length - 1, mi = B.movers.length;
      B.movers.push({ gi, from: [0, 0], to: [0, -h], speed: 150, mode: 'loop', dwell: 1.4 });
      B.hints.push({ gi, x0: x, x1: x + pw + 10, mover: mi, ride: true });
      B.hints.push({ gi: B.gr.length - 2, x0: x - 300, x1: x + 5, mover: mi, board: x });
      B.walls.push(W(x + pw + 8, y - h + 8, x + pw + 8, y + 200));
      B.decor.hatch.push([x + pw + 8, y - h, 30, h + 220], [x - 25, y, 25, 220]);
      B.spikes.push({ x: x + 10, y: y + 160, w: pw - 10, h: 40, dir: 'up' });
      B.decor.hatch.push([x, y + 200, pw + 30, 20]);
      B.begin(x + pw + 8, y - h + 2).flat(250);
      return B;
    };
    B.moverPit = () => {
      const x = X(), y = Y();
      B.end();
      const pw = 260, gap = 500;
      B.gr.push(makeGround(Path(x - 8, y + 1).line(x + pw, y + 1).pts, 1, { color: '#9c4dcc' }));
      const gi = B.gr.length - 1, mi = B.movers.length;
      B.movers.push({ gi, from: [0, 0], to: [gap - pw, 0], speed: 120, mode: 'loop', dwell: 1.2 });
      B.hints.push({ gi, x0: x, x1: x + gap, mover: mi, ride: true });
      B.hints.push({ gi: B.gr.length - 2, x0: x - 300, x1: x + 5, mover: mi, board: x });
      B.deathRects.push({ x, y: y + 80, w: gap, h: 400 });
      B.decor.hatch.push([x - 25, y, 25, 200], [x + gap, y, 25, 200]);
      coinsFlat(x + 60, x + gap - 60, y, 120);
      B.begin(x + gap, y + 2).flat(250);
      return B;
    };
    B.shaftDown = depth => {
      depth = depth || 650;
      const x = X(), y = Y(), w = 320;
      hint({ x0: x - 300, x1: x, v: 150 });
      B.decor.warns.push({ x: x - 120, y: y - 80 });
      B.end();
      B.walls.push(W(x + w, y - 250, x + w, y + depth - 210), W(x, y + 8, x, y + depth));
      B.spikes.push({ x: x + w - 40, y: y - 150, w: 40, h: depth - 120, dir: 'left' });
      B.fans.push({ x, y: y + depth - 400, w, h: 400, vt: 180 });
      B.decor.hatch.push([x + w, y - 250, 40, depth + 40], [x - 40, y + 6, 40, depth]);
      ballistic(x, y, 140, 0, 70, 7, (px, py) => py > y + depth - 100, B.coins, true);
      B.begin(x, y + depth).flat(w + 300);
      return B;
    };
    B.drop = depth => {
      depth = depth || 480;
      const x = X(), y = Y();
      hint({ x0: x - 300, x1: x, v: 200 });
      B.end();
      B.decor.hatch.push([x - 30, y, 30, depth]);
      ballistic(x, y, 200, 0, 70, 6, (px, py) => py > y + depth - 60, B.coins, true);
      B.begin(x + 20, y + depth); B.p.kind('soft').line(x + 380, y + depth).kind('normal'); B.flat(200);
      return B;
    };
    B.fanUp = h => {
      h = h || 380;
      const x = X(), y = Y(), w = 300;
      hint({ x0: x - 300, x1: x + w, v: 140 });
      B.end();
      B.fans.push({ x, y: y - h - 130, w, h: h + 380, vt: -330, up: true });
      B.walls.push(W(x + w, y - h + 8, x + w, y + 260));
      B.spikes.push({ x: x + 5, y: y + 215, w: w - 10, h: 45, dir: 'up' });
      B.decor.hatch.push([x + w, y - h, 40, h + 300], [x - 25, y, 25, 300], [x, y + 260, w, 25]);
      for (let k = 0; k < 5; k++) B.coins.push({ x: x + 60 + k * 40, y: y - 60 - k * (h / 5) });
      B.begin(x + w, y - h + 2).flat(300);
      return B;
    };
    B.portalTo = (nx, ny, color) => {
      const x = X(), y = Y();
      B.flat(260);
      B.portals.push({ ax: x + 160, ay: y - 38, bx: nx + 70, by: ny - 2, color });
      B.walls.push(W(x + 262, y - 200, x + 262, y));
      B.decor.hatch.push([x + 262, y - 200, 25, 225]);
      B.end();
      B.begin(nx, ny).flat(60);
      B.decor.hatch.push([nx - 25, ny - 200, 25, 225]);
      return B;
    };
    B.magnetHump = (h, len) => {
      const x = X(), y = Y();
      hint({ x0: x - 200, x1: x + len, v: 650 });
      B.p.kind('magnet').cubic(x + len * 0.3, y, x + len * 0.25, y - h, x + len * 0.5, y - h)
        .cubic(x + len * 0.75, y - h, x + len * 0.7, y, x + len, y).kind('normal');
      B.coins.push({ x: x + len * 0.5, y: y - h - 34 }, { x: x + len * 0.35, y: y - h * 0.7 - 34 }, { x: x + len * 0.65, y: y - h * 0.7 - 34 });
      return B;
    };
    B.magnetWall = h => {
      const x = X(), y = Y(), r1 = 110, r2 = 60;
      hint({ x0: x - 500, x1: x + r1 + 5, max: true });
      B.p.kind('magnet').arc(x, y - r1, r1, PI / 2, 0).line(x + r1, y - r1 - h).arc(x + r1 + r2, y - r1 - h, r2, PI, PI * 1.5).kind('normal');
      for (let k = 1; k < 5; k++) B.coins.push({ x: x + r1 - 34, y: y - r1 - h * k / 5 });
      B.decor.hatch.push([x + r1, y - r1 - h, 60, r1 + h + 25]);
      return B;
    };
    B.ice = (len, coinStep) => {
      const x = X(), y = Y();
      B.p.kind('ice').line(x + len, y).kind('normal');
      if (coinStep) coinsFlat(x, x + len, y, coinStep);
      return B;
    };
    B.reverseConv = len => {
      const x = X(), y = Y();
      B.p.kind('conv', -1).line(x + len, y).kind('normal');
      hint({ x0: x - 100, x1: x + len, max: true });
      coinsFlat(x, x + len, y, 100);
      return B;
    };
    B.crushers = (len, n) => {
      const x = X(), y = Y(), idx = [];
      for (let i = 0; i < n; i++) {
        idx.push(B.crush.length);
        // "green wave": at ~300 px/s every crusher is up when you reach it
        B.crush.push({ x: x + len * (i + 1) / (n + 1) - 45, w: 90, ground: y, speed: 2.0, phase: -2.0 * (len * (i + 1) / (n + 1)) / 300 });
      }
      hint({ x0: x - 200, x1: x + len, crush: idx, v: 300 });
      B.decor.warns.push({ x: x - 60, y: y - 120 });
      return B.flat(len, 120);
    };
    const timed = (x0, x1, v) => hint({ x0, x1, timed: true, v: v || 300, wait: x0 + 80 });
    B.birds = (len, n, speed) => {
      const x = X(), y = Y();
      speed = speed || 1.6;
      for (let i = 0; i < n; i++) {
        const cx = x + len * (i + 1) / (n + 1);
        // figure-eight flight that dips down to head height; phases form a "green wave" at ~320 px/s
        B.birds_.push({ cx, cy: y - 115, ax: Math.min(110, len / (n + 1) * 0.4), ay: 48, speed, phase: -speed * (cx - x) / 320 });
      }
      timed(x - 220, x + len);
      B.decor.warns.push({ x: x - 60, y: y - 130 });
      return B.flat(len, 110);
    };
    B.balloonGap = (gap, lava) => {
      const x = X(), y = Y(), rise = Math.min(200, gap * 0.22);
      B.flat(80);
      B.floaters.push({ x: x + 60, y: y - 45, kind: 'balloon' });
      hint({ x0: x - 300, x1: x + 80, v: 300 });
      B.end();
      const ex = x + 80, lx = ex + gap;
      B.hints.push({ balloon: true, landX: lx + 120 });
      pit(ex, lx, y, 260, lava);
      B.walls.push(W(lx, y - rise + 8, lx, y));
      B.decor.hatch.push([lx, y - rise, 30, rise]);
      for (let k = 1; k < 7; k++) B.coins.push({ x: ex + gap * k / 7, y: y - 60 - rise * k / 6 });
      B.begin(lx, y - rise).flat(450);
      return B;
    };
    B.bubbleLift = h => {
      h = h || 280;
      const x = X(), y = Y();
      B.flat(60);
      B.floaters.push({ x: x + 60, y: y - 45, kind: 'bubble' });
      hint({ x0: x - 300, x1: x + 60, v: 200 });
      B.flat(150);
      const wx = x + 60 + 200 - 4;
      B.p.line(wx, y);
      B.end();
      B.walls.push(W(wx + 4, y - h + 8, wx + 4, y));
      B.decor.hatch.push([wx + 4, y - h, 40, h + 25]);
      for (let k = 1; k < 5; k++) B.coins.push({ x: x + 60 + k * 30, y: y - 45 - k * (h / 4) });
      B.begin(wx + 4, y - h).flat(450);
      return B;
    };
    B.ballPit = depth => {
      depth = depth || 420;
      const x = X(), y = Y();
      hint({ x0: x - 300, x1: x, v: 250 });
      B.end();
      const len = 520, run = depth * 2.4;
      B.begin(x, y + depth); B.p.kind('soft').line(x + len, y + depth).kind('conv', 1);
      B.p.cubic(x + len + run * 0.5, y + depth, x + len + run * 0.5, y, x + len + run, y).kind('normal');
      B.waters.push({ x, y: y + depth - 190, w: len + 40, h: 190, style: 'balls' });
      B.decor.hatch.push([x - 30, y, 30, depth + 20]);
      hint({ x0: x, x1: x + len + run, max: true });
      ballistic(x, y, 250, 0, 70, 6, (px, py) => py > y + depth - 150, B.coins, true);
      return B.flat(250);
    };
    B.cannon = (gap, lava) => {
      const x = X(), y = Y(), A = 45 * PI / 180;
      B.flat(120);
      const cx = X();
      B.flat(60);
      const ex = X(), lx = cx + gap;
      const mx = cx + Math.cos(A) * 80, my = y - 46 - Math.sin(A) * 80;
      const power = idealSpeed(mx, my, Math.cos(A), -Math.sin(A), y, lx + 180);
      B.cannons_.push({ x: cx, y, angle: A, power });
      hint({ x0: x - 300, x1: ex, v: 250 });
      const arc = [];
      ballistic(mx, my, Math.cos(A) * power, -Math.sin(A) * power, 110, 30, (px, py) => py > y - 10 && px > lx, arc, true);
      if (B.special && arc.length) { const top = arc.reduce((a, c) => (c.y < a.y ? c : a)); top.v = 5; }
      B.coins.push(...arc);
      B.end();
      pit(ex, lx, y, 260, lava);
      B.begin(lx, y); B.p.kind('soft').line(lx + 420, y).kind('normal');
      return B.flat(250);
    };
    B.lavaHops = n => {
      n = n || 4;
      const x = X(), y = Y();
      hint({ x0: x - 350, x1: x + n * 300, v: 520 });
      B.end();
      let cx = x, cy = y;
      for (let i = 0; i < n; i++) {
        const gx = cx + 110, ny = cy + 45;
        B.spikes.push({ x: cx, y: cy + 150, w: 110, h: 70, dir: 'lava' });
        B.walls.push(W(gx, ny + 8, gx, ny + 220));
        B.decor.hatch.push([gx, ny, 25, 230]);
        B.begin(gx, ny).flat(190, 95);
        hint({ x0: gx - 10, x1: gx + 200, v: 520 });
        cx = X(); cy = Y();
        if (i < n - 1) { B.end(); }
      }
      B.deathRects.push({ x: x, y: y + 200, w: cx - x, h: 300 });
      return B.flat(150);
    };
    B.doorRoom = () => {
      const x = X(), y = Y();
      hint({ x0: x - 300, x1: x, v: 160 });
      B.end();
      const fy = y + 150, di = B.doors_.length;
      B.decor.hatch.push([x - 300, y, 300, 40]);
      B.begin(x - 360, fy - 50); B.p.cubic(x - 330, fy, x - 320, fy, x - 290, fy);
      B.flat(x + 520 - (x - 290));
      B.buttons_.push({ x: x - 230, w: 90, y: fy, target: { type: 'door', i: di }, color: '#ff8a3d' });
      B.doors_.push({ x: x + 500, top: y - 160, bottom: fy, open: 0 });
      B.decor.hatch.push([x + 490, y - 200, 40, 40]);
      B.decor.arrows.push({ x: x - 185, y: fy - 80, up: false, back: true });
      const dh = { door: di, btnX: x - 185, x0: x - 400, x1: x + 520 };
      B.hints.push(dh); later(gi => (dh.gi = gi));
      coinsFlat(x - 260, x - 120, fy, 60);
      return B.flat(300, 100);
    };
    B.piranhaPool = (len, n) => {
      const x = X(), y = Y(), depth = 120;
      B.pool(len, depth);
      const run = Math.max(210, depth * 2.2);
      for (let i = 0; i < n; i++) {
        const px = x + run * 0.7 + (len - 1.4 * run) * (i + 0.5) / n;
        B.piranhas_.push({ x: px, surface: y, base: depth * 0.6, h: depth * 0.6 + 130, speed: 2.0, phase: -2.0 * (px - x) / 280 });
      }
      B.hints.pop();
      timed(x - 200, x + len, 300);
      return B;
    };
    B.hailZone = (len, n, period) => {
      const x = X(), y = Y();
      for (let i = 0; i < n; i++) B.hail_.push({ x: x + len * (i + 0.5) / n, top: y - 520, bottom: y, period: period || 1.1, phase: (i * 0.37) % 1 });
      B.decor.clouds.push({ x, y: y - 560, w: len });
      timed(x - 200, x + len);
      return B.flat(len, 110);
    };
    B.pendulums = (len, n, speed) => {
      const x = X(), y = Y();
      speed = speed || 1.3;
      for (let i = 0; i < n; i++) { const px = x + len * (i + 1) / (n + 1); B.pend_.push({ px, py: y - 300, L: 235, amp: 0.9, speed, phase: -speed * (px - x) / 450, r: 30 }); }
      B.decor.hatch.push([x, y - 330, len, 30]);
      timed(x - 220, x + len, 450);
      return B.flat(len, 110);
    };
    B.dropSpikes = (len, n) => {
      const x = X(), y = Y();
      for (let i = 0; i < n; i++) { const dx = x + len * (i + 1) / (n + 1); B.drops_.push({ x: dx, y0: y - 300, y: y - 300, ground: y, trigger: 230, state: 'hang', vy: 0 }); }
      B.decor.hatch.push([x, y - 360, len, 40]);
      hint({ x0: x - 300, x1: x + len, v: 170 });
      return B.flat(len, 110);
    };
    B.cableCar = (gap, rise, lava) => {
      const x = X(), y = Y();
      B.end();
      const pw = 200;
      B.gr.push(makeGround(Path(x - 8, y + 1).line(x + 4 + pw, y + 1).pts, 1, { color: '#ff8a3d', cable: true }));
      const gi = B.gr.length - 1, mi = B.movers.length;
      B.movers.push({ gi, from: [0, 0], to: [gap - pw, -rise], speed: 140, mode: 'loop', dwell: 1.4, cable: true });
      B.hints.push({ gi, x0: x, x1: x + gap + 10, mover: mi, ride: true });
      B.hints.push({ gi: B.gr.length - 2, x0: x - 300, x1: x + 5, mover: mi, board: x });
      pit(x, x + gap + 4, y, 260, lava);
      B.walls.pop(); B.walls.pop();
      B.walls.push(W(x, y + 8, x, y + 260));
      B.walls.push(W(x + gap + 4, y - rise + 8, x + gap + 4, y + 260));
      B.decor.hatch.push([x + gap + 4, y - rise, 30, rise]);
      B.decor.cables.push([x - 20, y - 175, x + gap + 30, y - rise - 175]);
      coinsFlat(x + 120, x + gap - 80, y - 30, 130);
      B.begin(x + gap + 4, y - rise + 2).flat(300);
      return B;
    };
    // ---- extreme sections (levels 51–60) ----
    B.balanceLift = (h, amp) => { // elevator whose floor is a belt pushing back and forth: keep your balance
      h = h || 450; amp = amp || 650;
      const x = X(), y = Y();
      B.end();
      const pw = 300;
      B.gr.push(makeGround(Path(x - 8, y + 1).kind('conv', 1).line(x + 4 + pw, y + 1).pts, 1, { convWave: { amp, speed: 1.7, phase: 0 } }));
      const gi = B.gr.length - 1, mi = B.movers.length;
      B.movers.push({ gi, from: [0, 0], to: [0, -h], speed: 110, mode: 'loop', dwell: 1.6 });
      B.hints.push({ gi, x0: x - 20, x1: x + pw + 10, mover: mi, ride: true, balance: x + 4 + pw / 2 });
      B.hints.push({ gi: B.gr.length - 2, x0: x - 300, x1: x + 5, mover: mi, board: x });
      B.walls.push(W(x + pw + 8, y - h + 8, x + pw + 8, y + 200));
      B.decor.hatch.push([x + pw + 8, y - h, 30, h + 220], [x - 25, y, 25, 220], [x, y + 200, pw + 30, 20]);
      B.spikes.push({ x: x + 10, y: y + 160, w: pw - 10, h: 40, dir: 'up' });
      B.decor.warns.push({ x: x - 80, y: y - 90 });
      B.begin(x + pw + 8, y - h + 2).flat(300);
      return B;
    };
    B.seesaw = () => { // tipping plank over a pit, spikes above its raised end: slow down to let it tip
      const x = X(), y = Y(), half = 350, a = 0.32;
      hint({ x0: x - 300, x1: x, v: 220 });
      B.end();
      const px = x + half * Math.cos(a), py = y - half * Math.sin(a) + 1;
      B.gr.push(makeGround(Path(px - half, py).line(px + half, py).pts, 1, { color: '#8d6e63', pivot: [px, py] }));
      const gi = B.gr.length - 1;
      B.seesaws_.push({ gi, maxA: a, rot: -a, omega: 0 });
      B.hints.push({ gi, x0: px - half - 20, x1: px + half + 20, seesaw: B.seesaws_.length - 1, pivot: px });
      const rx = x + 2 * half * Math.cos(a);
      pit(x, rx + 2, y, 240);
      B.spikes.push({ x: px + 110, y: py - half * Math.sin(a) * 0.9 - 62 - 55, w: 260, h: 40, dir: 'down' });
      B.decor.hatch.push([px + 110, py - half * Math.sin(a) * 0.9 - 62 - 85, 260, 30]);
      B.decor.pivots.push([px, py]);
      B.begin(rx + 2, y + 2).flat(300);
      return B;
    };
    B.forkTrap = () => { // two ways: the right one needs the right speed, the easy-looking one ends in a trap
      const x = X(), y = Y(), A = 40 * PI / 180, Lk = 130, hK = 60, gap = 380, land = 420;
      const tx = x + Lk, ty = y - hK;
      B.p.cubic(x + Lk * 0.4, y, tx - 40 * Math.cos(A), ty + 40 * Math.sin(A), tx, ty);
      B.decor.ramps.push([[x + 20, y], ...B.p.pts.filter(q => q[0] >= x), [tx, y]]);
      const lx = tx + gap, ly = y - 40;
      const v = idealSpeed(tx, ty, Math.cos(A), -Math.sin(A), ly, lx + land * 0.3);
      hint({ x0: x - 1000, x1: x - 5, kick: true, v: Math.sqrt(v * v + 2 * G * hK + 2 * (ROLL + AIR * v * v) * Lk * 1.2) * 1.02 });
      hint({ x0: x - 5, x1: tx + 1, coast: true });
      const arc = []; ballistic(tx, ty, Math.cos(A) * v, -Math.sin(A) * v, 70, 10, (px, py) => py > ly - 5 && px > lx, arc, true);
      B.coins.push(...arc);
      B.end();
      // the wrong way: a comfortable road below with bait coins, then a hidden trapdoor over lava
      const by = y + 180, trapX = lx + 520;
      B.walls.push(W(tx, ty + 8, tx, by));
      B.gr.push(makeGround(Path(tx - 10, by).line(trapX, by).pts, 1));
      for (let k = 0; k < 2; k++) {
        const x0 = trapX + k * 160;
        B.gr.push(makeGround(Path(x0, by + 1 + k).line(x0 + 160, by + 1 + k).pts, 1, { pivot: [x0, by + 1 + k] }));
        B.flaps.push({ gi: B.gr.length - 1, dir: 1, delay: 0.05, hidden: true });
      }
      B.spikes.push({ x: trapX, y: by + 200, w: 320, h: 70, dir: 'lava' });
      B.walls.push(W(trapX + 320, by - 150, trapX + 320, by + 270), W(trapX, by + 8, trapX, by + 270));
      B.decor.hatch.push([trapX + 320, by - 150, 30, 420], [tx - 30, by, lx + 900 - tx, 0]);
      coinsFlat(tx + 60, trapX + 300, by, 70);
      B.decor.warns.push({ x: x - 60, y: y - 120, q: true });
      // the right way, up high
      B.walls.push(W(lx, ly + 8, lx, by - 70));
      B.decor.hatch.push([lx, ly, 30, by - 70 - ly], [lx, ly, land, 30]);
      B.begin(lx, ly).flat(land);
      return B;
    };
    B.flight = (len, centers, kind) => { // wings: fly through gates, gas = up, brake = down
      const x = X(), y = Y();
      B.flat(80);
      B.floaters.push({ x: x + 50, y: y - 45, kind: kind || 'wings' });
      hint({ x0: x - 300, x1: x + 80, v: 300 });
      B.end();
      const sx = x + 80, ex = sx + len, top = y - 460, floor = y + 260;
      B.spikes.push({ x: sx, y: floor - 70, w: len, h: 70, dir: 'lava' }, { x: sx, y: top, w: len, h: 40, dir: 'down' });
      B.decor.hatch.push([sx, top - 30, len, 30]);
      const gates = [];
      const n = centers.length;
      for (let i = 0; i < n; i++) {
        const gx = sx + len * (i + 1) / (n + 1), cy = y + centers[i], gh = kind === 'hot' ? 250 : 210;
        gates.push([gx, cy]);
        B.spikes.push({ x: gx - 25, y: top + 40, w: 50, h: cy - gh / 2 - top - 40, dir: 'pillarTop' });
        B.spikes.push({ x: gx - 25, y: cy + gh / 2, w: 50, h: floor - 70 - cy - gh / 2, dir: 'pillarBottom' });
        B.coins.push({ x: gx, y: cy });
      }
      B.hints.push({ flight: true, hot: kind === 'hot', gates, landX: ex + 40, landY: y });
      B.walls.push(W(ex, y + 8, ex, floor));
      B.decor.hatch.push([ex, y, 30, floor - y]);
      B.begin(ex, y); B.p.kind('soft').line(ex + 400, y).kind('normal');
      return B.flat(200);
    };
    B.wheelTunnel = len => { // low ceiling: only small wheels fit
      const x = X(), y = Y();
      B.wheelSw_.push({ x: x + 40, y, size: 0.6 });
      B.flat(160);
      B.spikes.push({ x: x + 160, y: y - 88, w: len, h: 32, dir: 'down' });
      B.decor.hatch.push([x + 160, y - 118, len, 30]);
      hint({ x0: x - 200, x1: x + len + 300, v: 320 });
      B.flat(len, 90);
      B.wheelSw_.push({ x: X() + 60, y, size: 1 });
      return B.flat(160);
    };
    B.spikeCarpet = len => { // low spikes on the ground: big wheels roll over them
      const x = X(), y = Y();
      B.wheelSw_.push({ x: x + 40, y, size: 1.8 });
      B.flat(160);
      B.spikes.push({ x: x + 160, y: y - 22, w: len, h: 22, dir: 'up', low: true });
      hint({ x0: x - 200, x1: x + len + 300, v: 320 });
      B.flat(len, 110);
      B.wheelSw_.push({ x: X() + 140, y, size: 1 }); // after the rear wheel has left the carpet
      return B.flat(240);
    };
    B.fortuneWheel = () => {
      const x = X(), y = Y();
      B.fortunes_.push({ x: x + 180, y, used: false });
      return B.flat(380);
    };
    B.secretBelt = () => { // a belt far too strong to beat — a hidden button behind you turns it around
      const x = X(), y = Y();
      hint({ x0: x - 300, x1: x, v: 160 });
      B.end();
      const fy = y + 150, bi = B.buttons_.length;
      B.decor.hatch.push([x - 300, y, 300, 40]);
      B.begin(x - 360, fy - 50); B.p.cubic(x - 330, fy, x - 320, fy, x - 290, fy);
      B.flat(x + 300 - (x - 290));
      B.p.kind('conv', -2.2).line(x + 800, fy).kind('normal');
      B.flat(150);
      B.p.cubic(X() + 150, fy, X() + 150, y, X() + 300, y);
      const btn = { x: x - 240, w: 70, y: fy, target: { type: 'flip' }, color: '#9e9e9e', hidden: true };
      B.buttons_.push(btn);
      later(gi => (btn.target.gi = gi));
      const fh = { secret: bi, btnX: x - 205, x0: x - 400, x1: x + 1300 };
      B.hints.push(fh); later(gi => (fh.gi = gi));
      coinsFlat(x + 320, x + 780, fy, 80);
      return B.flat(250);
    };
    B.slingshot = () => { // drive in, aim, shoot through the bullseye — everything else is lava or wall
      B.flat(140);
      const x = X(), y = Y(), sx = x;
      B.flat(60);
      B.end();
      const tx = sx + 760, ty = y - 230, r = 60, board = 130;
      const exit = { x: tx + 110, y: ty + 40 };
      const i = B.slings_.length;
      B.slings_.push({ x: sx, y, target: { x: tx, y: ty, r, board }, exit });
      // find an angle/power the bot (and a careful player) can use
      const lx = sx + 30, ly = y - 40, cross = (a, p) => {
        let px = lx, py = ly, vx = Math.cos(a) * p, vy = -Math.sin(a) * p;
        for (let k = 0; k < 2400; k++) { const dt = 1 / 240; vy += G * dt; const sp = Math.hypot(vx, vy); vx -= AIR * 0.5 * vx * sp * dt; vy -= AIR * 0.5 * vy * sp * dt; const nx = px + vx * dt; if (px < tx && nx >= tx) return py - 30; px = nx; py += vy * dt; if (vx <= 0) break; }
        return Infinity;
      };
      let best = null;
      for (let a = SLING.mid - SLING.amp + 0.05; a < SLING.mid + SLING.amp; a += 0.01) {
        let lo = SLING.pmin, hi = SLING.pmax;
        for (let k = 0; k < 40; k++) { const m = (lo + hi) / 2; if (cross(a, m) > ty) lo = m; else hi = m; } // more power → higher at the board
        const p = (lo + hi) / 2;
        if (Math.abs(cross(a, p) - ty) < 3 && (!best || Math.abs(a - 0.85) < Math.abs(best.a - 0.85))) best = { a, p };
      }
      B.hints.push({ sling: i, angle: best.a, power: best.p });
      // lava below, walls above and below the board
      B.spikes.push({ x: sx + 60, y: y + 190, w: tx + 70 - sx - 60, h: 70, dir: 'lava' });
      B.walls.push(W(sx + 60, y + 8, sx + 60, y + 260));
      B.decor.hatch.push([sx + 30, y, 30, 260], [tx - 8, ty + board, 16, y + 260 - ty - board]);
      B.decor.warns.push({ x: sx - 80, y: y - 90 });
      for (let k = 1; k < 6; k++) { const ccx = lx + (tx - lx) * k / 6; B.coins.push({ x: ccx, y: ty + 20 - Math.sin(k / 6 * PI) * 120 }); }
      B.begin(tx + 40, ty + 70).flat(450);
      return B;
    };
    // ---- world 2 sections (levels 61–70) ----
    B.vineJump = (n, len) => { // drive off the edge, grab the vines, swing (gas) and let go (brake) at the right moment
      n = n || 2; len = len || 320;
      const x = X(), y = Y();
      hint({ x0: x - 350, x1: x, v: 330 });
      B.end();
      // the first vine end hangs right on the path of a bike leaving the edge at ~330 px/s
      const gap = 560, first = x + 110, endY = y + 15, landX = first + (n - 1) * gap + 500;
      for (let i = 0; i < n; i++) {
        const px = first + i * gap, idx = B.vines_.length;
        B.vines_.push({ px, py: endY - len, len });
        B.hints.push({ vine: idx, next: i < n - 1 ? idx + 1 : null, landX, landY: y });
        B.coins.push({ x: px + 150, y: endY - 60 }, { x: px + 280, y: endY - 150 });
      }
      B.spikes.push({ x: x, y: y + 230, w: landX - x, h: 70, dir: 'lava' });
      B.walls.push(W(x, y + 8, x, y + 300), W(landX, y + 8, landX, y + 300));
      B.decor.hatch.push([x - 30, y, 30, 300], [landX, y, 30, 300], [x, endY - len - 30, landX - x, 30]);
      B.begin(landX, y).flat(400);
      return B;
    };
    B.aimCannonShot = dist => { // aim the barrel yourself (gas up / brake down), it fires when the countdown runs out
      dist = dist || 1100;
      B.flat(120);
      const cx = X(), y = Y();
      B.flat(60);
      const ex = X(), pad = 280, lx = cx + dist - pad / 2;
      // power so that a medium angle lands in the middle of the pad
      const A0 = 0.75, mx = cx + Math.cos(A0) * 80, my = y - 46 - Math.sin(A0) * 80;
      const power = idealSpeed(mx, my, Math.cos(A0), -Math.sin(A0), y, cx + dist);
      const i = B.aimCannons_.length;
      B.aimCannons_.push({ x: cx, y, power });
      B.hints.push({ aim: i, angle: A0 });
      hint({ x0: cx - 300, x1: ex, v: 250 });
      const arc = []; ballistic(mx, my, Math.cos(A0) * power, -Math.sin(A0) * power, 120, 30, (px, py) => py > y - 10 && px > lx, arc, true);
      if (B.special && arc.length) { const top = arc.reduce((a, c) => (c.y < a.y ? c : a)); top.v = 5; }
      B.coins.push(...arc.filter((c, k) => k % 2 === 0));
      B.end();
      pit(ex, lx, y, 260, true);
      B.begin(lx, y); B.p.kind('soft').line(lx + pad, y).kind('normal');
      return B.flat(250);
    };
    B.hotBalloonRide = (len, centers) => B.flight(len, centers, 'hot');
    B.sharkLake = (len, n) => { // a deep lake with sharks patrolling — cross when the way is clear
      const x = X(), y = Y(), depth = 170;
      B.pool(len, depth);
      B.hints.pop();
      // sharks patrol the whole lake near the surface: safe on the bottom, dangerous at the shores
      const run = Math.max(210, depth * 2.2), ax = len / 2 - run * 0.35;
      for (let i = 0; i < n; i++) B.sharks_.push({ cx: x + len / 2, y: y + 58, ax, speed: 0.8 + 0.15 * i, phase: i * 2.4 });
      timed(x - 200, x + len, 300);
      B.decor.warns.push({ x: x - 60, y: y - 100 });
      return B;
    };
    B.dragonCave = (len, gaps) => { // an angry dragon at the end of the cave spits fireballs: JUMP over them
      const x = X(), y = Y();
      const i = B.dragons_.length;
      B.dragons_.push({ x: x + len - 200, y, x0: x + 20, speed: 400, gaps: gaps || [1.8, 2.2, 1.9], phase: 0.5, body: x + len - 110 });
      B.jumpZones_.push({ x0: x - 100, x1: x + len + 80, y0: y - 400, y1: y + 40 });
      B.decor.caves.push({ x, y, w: len });
      B.hints.push({ dragon: i, x0: x - 200, x1: x + len + 60, v: 300 });
      const dh = B.hints[B.hints.length - 1];
      later(gi => (dh.gi = gi));
      B.decor.hatch.push([x, y - 300, len + 40, 40]);
      B.spikes.push({ x: x + 10, y: y - 260, w: len, h: 30, dir: 'down' });
      return B.flat(len + 60, 160);
    };
    B.mark = () => { B.guard = X(); return B; };
    B.x = X; B.y = Y; B.bounds = () => { let lo = Infinity, hi = -Infinity; for (const g of B.gr) for (const q of g.pts) { lo = Math.min(lo, q[1]); hi = Math.max(hi, q[1]); } if (B.p) for (const q of B.p.pts) { lo = Math.min(lo, q[1]); hi = Math.max(hi, q[1]); } return [lo, hi]; };
    B.checkpoint = () => {
      const x = X();
      const cp = { x: x + 70, tx: x + 20, dir: 1 };
      B.checkpoints.push(cp); later(gi => (cp.gi = gi));
      return B;
    };
    B.finishLine = (len, bonus) => {
      const x = X();
      B.flat(len || 500);
      B.finish = { x: x + 200, dir: 1, bonus: !!bonus };
      later(gi => (B.finish.gi = gi));
      if (bonus) B.bonusDrop();
      B.end();
      return B;
    };
    // bonus after the finish: over a ramp into a deep sky shaft, steer with gas (right) / brake (left)
    B.bonusDrop = () => {
      B.flat(250);
      const x0 = X(), y0 = Y();
      B.p.cubic(x0 + 80, y0, x0 + 140, y0 + 40, x0 + 220, y0 + 120); // ramp down and off
      const ex = X(), ey = Y();
      B.end();
      const zx = ex - 150, zw = 1300, depth = 3600, padY = ey + depth;
      B.bonusZone = { x: zx, y: ey - 200, w: zw, h: depth + 200, padY };
      // coins in the air: mostly 1, some 3 and 5, rarely 20 (fixed pattern, no level randomness used)
      const h = k => { const v = Math.sin(k * 91.73 + 7.1) * 43758.5453; return v - Math.floor(v); };
      for (let y = ey + 220, k = 0; y < padY - 180; y += 70, k++) {
        const n = 1 + Math.floor(h(k) * 3);
        for (let j = 0; j < n; j++) {
          const r = h(k * 7 + j * 3.3), q = h(k * 13 + j * 5.1);
          const v = q < 0.03 ? 20 : q < 0.15 ? 5 : q < 0.4 ? 3 : 1;
          B.coins.push({ x: zx + 90 + r * (zw - 180), y: y + j * 25, v });
        }
      }
      B.decor.bonus = B.bonusZone;
      B.begin(zx, padY); B.p.kind('soft').line(zx + zw, padY).kind('normal');
      later(gi => (B.bonusEnd = gi));
      return B;
    };
    B.sections = []; B.saws_ = []; B.crush = []; B.lowG = []; B.birds_ = []; B.floaters = []; B.cannons_ = []; B.doors_ = []; B.buttons_ = [];
    B.piranhas_ = []; B.hail_ = []; B.pend_ = []; B.drops_ = []; B.decor.clouds = []; B.decor.cables = [];
    B.seesaws_ = []; B.fortunes_ = []; B.wheelSw_ = []; B.decor.pivots = []; B.slings_ = [];
    B.vines_ = []; B.aimCannons_ = []; B.sharks_ = []; B.dragons_ = []; B.jumpZones_ = []; B.decor.caves = [];
    B.build = () => ({
      name: B.name, start: B.start, grounds: B.gr, walls: B.walls, coins: B.coins, spikes: B.spikes,
      gates: [], buttons: B.buttons.concat(B.buttons_), movers: B.movers, flaps: B.flaps, saws: B.saws_, fans: B.fans,
      portals: B.portals, waters: B.waters, crushers: B.crush, lowG: B.lowG, theme: B.theme, birds: B.birds_, floaters: B.floaters,
      cannons: B.cannons_, doors: B.doors_, piranhas: B.piranhas_, hail: B.hail_, pendulums: B.pend_, dropSpikes: B.drops_, special: B.special,
      seesaws: B.seesaws_, fortunes: B.fortunes_, wheelSwitches: B.wheelSw_, slings: B.slings_,
      vines: B.vines_, aimCannons: B.aimCannons_, sharks: B.sharks_, dragons: B.dragons_, jumpZones: B.jumpZones_, checkpoints: B.checkpoints, finish: B.finish,
      deathY: B.deathY, deathRects: B.deathRects, decor: B.decor, hints: B.hints, sections: B.sections, bonusZone: B.bonusZone, bonusEnd: B.bonusEnd,
    });
    return B;
  }

  // Level 3 — "Level 1" sketch: hills, loop, saw valley, pool between trampolines, magnet hump, folding bridges
  function level3() {
    const B = Builder('Level 3');
    B.begin(0, 0).flat(120).startHere().flat(300, 80).slope(400, 150).waves(660, 40, 330).flat(200)
      .conv(350).flat(80).loop(130).flat(300, 80)
      .kickerGap({ h: 45, gap: 240 }).flat(150)
      .ceilBumps(880).flat(250)
      .checkpoint().flat(250)
      .saws(700, 2).flat(200)
      .pool(560, 110, true).flat(300, 90)
      .conv(250).flat(250).stairs(5).flat(300)
      .checkpoint().flat(250)
      .magnetHump(170, 440).flat(250)
      .flapsBridge(3, 200).flat(150)
      .kickerGap({ h: 55, gap: 300, drop: 120 }).flat(100)
      .checkpoint().flat(250)
      .conv(300).flat(80).loop(140, true).flat(300, 90)
      .saws(600, 1).flat(200)
      .finishLine(600);
    return B.build();
  }

  // Level 4 — spike wheel, portals, trampolines & water, elevator, trampoline chain, cushion drop, magnet wall
  function level4() {
    const B = Builder('Level 4');
    B.begin(0, 0).flat(120).startHere().flat(300, 80).slope(500, 200).flat(150)
      .spikeWheel(600).flat(200)
      .portalTo(0, 1300, '#2e7d32').flat(250)
      .magnetHump(150, 400).flat(250)
      .kickerGap({ h: 50, gap: 280 }).flat(150)
      .pool(560, 110, true).flat(300)
      .checkpoint().flat(300)
      .elevator(420).flat(150)
      .flapsBridge(3, 200).flat(200)
      .trampGap({ gap: 380 }).flat(200).trampGap({ gap: 420, power: 950 }).flat(150)
      .checkpoint().flat(200)
      .portalTo(0, 2700, '#f2c200').flat(250)
      .drop(480).flat(150)
      .pool(620, 140).flat(250)
      .conv(300).flat(60).magnetWall(260).flat(300, 90)
      .conv(250).flat(60).loop(130).flat(250)
      .kickerGap({ h: 60, gap: 340, drop: 60 }).flat(100)
      .finishLine(600);
    return B.build();
  }

  // Level 5 — the hard one: shaft, triple loop, big jump, portals, water, fan lift, cushion, chains
  function level5() {
    const B = Builder('Level 5');
    B.begin(0, 0).flat(120).startHere().flat(250, 80).slope(400, 150).flat(300)
      .shaftDown(650).flat(150)
      .conv(450).flat(60).loop(130).conv(200).flat(60).loop(130).conv(200).flat(60).loop(130).flat(300, 90)
      .checkpoint().flat(300).conv(300).flat(300)
      .kickerGap({ h: 70, gap: 420, drop: 80 }).flat(150)
      .portalTo(0, 2200, '#7b1fa2').flat(250)
      .pool(700, 150).flat(250)
      .fanUp(380).flat(100)
      .checkpoint().flat(200)
      .stairs(6).flat(250)
      .saws(700, 2).flat(200)
      .moverPit().flat(100)
      .drop(500).flat(100)
      .spikeWheel(600).flat(200)
      .checkpoint().flat(200)
      .portalTo(0, 3800, '#212121').flat(250)
      .trampGap({ gap: 420 }).flat(200).trampGap({ gap: 450, power: 950 }).flat(150)
      .conv(300).flat(60).magnetWall(300).flat(250)
      .flapsBridge(4, 180).flat(150)
      .conv(300).flat(60).loop(140, true).flat(300, 90)
      .kickerGap({ h: 80, gap: 450, drop: 100 }).flat(100)
      .finishLine(600, true);
    return B.build();
  }

  // Level 6 — "Eisbahn": ice everywhere, gas and brakes barely grip
  function level6() {
    const B = Builder('Level 6 · Eisbahn');
    B.begin(0, 0).flat(120).startHere().flat(200, 80).ice(500, 90).slope(400, 160).ice(900, 100)
      .kickerGap({ h: 50, gap: 260 }).ice(400, 100)
      .checkpoint().flat(200).ice(700, 100)
      .shaftDown(600).flat(100).ice(600, 100)
      .trampGap({ gap: 400 }).ice(300)
      .checkpoint().flat(200).ice(500, 100)
      .stairs(5).flat(150).ice(400)
      .conv(400).flat(60).loop(130).ice(500, 100)
      .drop(450).flat(100).ice(500, 100)
      .checkpoint().flat(200).ice(400)
      .saws(600, 2).flat(300).conv(300).ice(700, 100)
      .kickerGap({ h: 60, gap: 320, drop: 60 }).ice(200)
      .finishLine(600);
    return B.build();
  }

  // Level 7 — "Mondlandung": low gravity zones, huge jumps, at night
  function level7() {
    const B = Builder('Level 7 · Mondlandung');
    B.theme = 'night';
    B.begin(0, 0).flat(120).startHere().flat(300, 80).slope(300, 80).flat(500)
      .kickerGap({ h: 60, gap: 800, g: 0.35, landLen: 450 }).flat(250)
      .trampGap({ gap: 900, power: 650, g: 0.35, landLen: 450 }).flat(200)
      .checkpoint().flat(250)
      .spikeWheel(600).flat(400)
      .kickerGap({ h: 80, gap: 1000, drop: 150, g: 0.3, landLen: 450 }).flat(200)
      .checkpoint().flat(300)
      .trampGap({ gap: 1100, power: 700, g: 0.3, landLen: 450 }).flat(250)
      .flapsBridge(3, 200).flat(400)
      .kickerGap({ h: 50, gap: 700, drop: 100, g: 0.4, landLen: 450 }).flat(150)
      .finishLine(600);
    return B.build();
  }

  // Level 8 — "Stampfwerk": a factory with crushers, belts both ways, saws and machines
  function level8() {
    const B = Builder('Level 8 · Stampfwerk');
    B.begin(0, 0).flat(120).startHere().flat(300, 80).conv(400).flat(250)
      .crushers(600, 2).flat(200)
      .reverseConv(450).flat(200)
      .checkpoint().flat(250)
      .elevator(400).flat(150)
      .saws(600, 2).flat(150)
      .crushers(450, 1).flat(200)
      .checkpoint().flat(250)
      .moverPit().flat(150)
      .fanUp(380).flat(150)
      .conv(300).flat(60).magnetWall(260).flat(300, 90)
      .checkpoint().flat(200)
      .crushers(800, 3).flat(150)
      .reverseConv(300).flat(150)
      .trampGap({ gap: 400 }).flat(100)
      .finishLine(600);
    return B.build();
  }

  // Level 9 — "Achterbahn": loops, magnets and portals like a roller coaster
  function level9() {
    const B = Builder('Level 9 · Achterbahn');
    B.begin(0, 0).flat(120).startHere().flat(250, 80).conv(500).flat(60).loop(130).conv(250).flat(60).loop(150, true).flat(250, 90)
      .magnetHump(180, 450).flat(200)
      .checkpoint().flat(200)
      .conv(350).flat(60).magnetWall(300).flat(500)
      .kickerGap({ h: 60, gap: 340 }).flat(150)
      .portalTo(0, 2000, '#e91e63').flat(250)
      .conv(450).flat(60).loop(140).conv(250).flat(60).loop(140).flat(250, 90)
      .checkpoint().flat(200)
      .magnetHump(200, 500).flat(250)
      .trampGap({ gap: 420 }).flat(150)
      .conv(350).flat(60).loop(150, true).conv(250).flat(60).loop(130).flat(200)
      .finishLine(600);
    return B.build();
  }

  // Level 10 — "Finale": a bit of everything
  function level10() {
    const B = Builder('Level 10 · ⭐ Finale');
    B.special = true;
    B.begin(0, 0).flat(120).startHere('FINALE').flat(250, 80).slope(400, 150).ice(500, 100)
      .kickerGap({ h: 50, gap: 280 }).flat(150)
      .crushers(500, 2).flat(150)
      .conv(400).flat(60).loop(130).flat(250)
      .checkpoint().flat(250)
      .saws(600, 2).flat(150)
      .pool(560, 110, true).flat(250)
      .elevator(400).flat(150)
      .portalTo(0, 1800, '#00bcd4').flat(400).conv(300).flat(400)
      .kickerGap({ h: 70, gap: 900, g: 0.35, landLen: 450 }).flat(150)
      .checkpoint().flat(200)
      .fanUp(380).flat(150)
      .flapsBridge(3, 200).flat(150)
      .ice(400).shaftDown(600).flat(150)
      .reverseConv(350).flat(150)
      .checkpoint().flat(200)
      .spikeWheel(600).flat(450)
      .moverPit().flat(150)
      .stairs(5).flat(250)
      .conv(350).flat(60).magnetWall(280).flat(250)
      .portalTo(0, 3800, '#ffc107').flat(250)
      .checkpoint().flat(200)
      .trampGap({ gap: 1000, power: 700, g: 0.33, landLen: 450 }).flat(150)
      .crushers(600, 2).flat(150)
      .conv(400).flat(60).loop(150, true).flat(550)
      .drop(450).flat(150).conv(300).flat(400)
      .kickerGap({ h: 80, gap: 450, drop: 100 }).flat(100)
      .finishLine(700);
    return B.build();
  }


  // ======================= LEVEL GENERATOR (levels 11–50) =======================
  function rng(seed) { // mulberry32
    return () => { seed |= 0; seed = seed + 0x6D2B79F5 | 0; let t = Math.imul(seed ^ seed >>> 15, 1 | seed); t = t + Math.imul(t ^ t >>> 7, 61 | t) ^ t; return ((t ^ t >>> 14) >>> 0) / 4294967296; };
  }
  const INTRO = { 11: 'birds', 12: 'balloon', 13: 'ballpit', 14: 'cannon', 15: 'lava', 16: 'door', 17: 'piranha', 18: 'hail', 19: 'pendulum', 21: 'cable', 22: 'bubble', 23: 'drops' };
  const GEN_NAMES = {
    11: 'Vogelflug', 12: 'Luftballon', 13: 'Bällebad', 14: 'Kanonenkugel', 15: 'Lavaland', 16: 'Geheimtür', 17: 'Piranha-Teich', 18: 'Eisregen', 19: 'Pendelhalle',
    20: '⭐ Extra: Feuerprobe', 21: 'Seilbahn', 22: 'Seifenblasen', 23: 'Stachelregen', 24: 'Mitternacht', 25: 'Vulkan', 26: 'Wolkenkratzer', 27: 'Gletscher',
    28: 'Zirkus', 29: 'Dschungel', 30: '⭐ Extra: Sturm', 31: 'Fabrikhalle', 32: 'Sternenstaub', 33: 'Wasserwelt', 34: 'Kanonade', 35: 'Lava-Lauf',
    36: 'Geisterbahn', 37: 'Frosthölle', 38: 'Vogelschwarm', 39: 'Labyrinth', 40: '⭐ Extra: Albtraum', 41: 'Turbo', 42: 'Himmelsleiter', 43: 'Unterwelt',
    44: 'Eiszeit', 45: 'Feuerwerk', 46: 'Chaos', 47: 'Präzision', 48: 'Höllenritt', 49: 'Der letzte Weg', 50: '⭐ Extra: Endgegner',
    51: '☠ Wüstensturm', 52: '☠ Mondbasis', 53: '☠ Sonnenglut', 54: '☠ Tiefsee', 55: '☠ Strandparty', 56: '☠ Dschungeltempel',
    57: '☠ Gletscherspalte', 58: '☠ Neonstadt', 59: '☠ Vulkanherz', 60: '⭐☠ Extra: Weltall',
    61: '🐉 Lianenwald', 62: '🐉 Drachenhöhle', 63: '🐉 Haifischbucht', 64: '🐉 Kanonenfels', 65: '🐉 Ballonfahrt',
    66: '🐉 Feuerschlucht', 67: '🐉 Tiefer See', 68: '🐉 Urwaldriesen', 69: '🐉 Drachenhort', 70: '⭐🐉 Extra: Drachenkönig',
  };
  const FOCUS = {
    51: ['seesaw', 'fork', 'lava'], 52: ['flight', 'moon', 'moonTramp'], 53: ['lift', 'lava', 'cannon'], 54: ['tunnel', 'carpet', 'piranha'],
    55: ['fortune', 'fork', 'tramp', 'balloon'], 56: ['secret', 'fork', 'drops', 'birds'], 57: ['seesaw', 'tunnel', 'carpet', 'hail'],
    58: ['flight', 'lift', 'crush'], 59: ['fork', 'secret', 'lava', 'lift', 'seesaw'], 60: ['lift', 'seesaw', 'fork', 'flight', 'tunnel', 'carpet', 'secret', 'fortune'],
    61: ['vine'], 62: ['dragon'], 63: ['shark'], 64: ['aim'], 65: ['hot'], 66: ['dragon', 'vine'], 67: ['shark', 'hot'], 68: ['vine', 'aim'],
    69: ['dragon', 'shark', 'aim'], 70: ['vine', 'aim', 'hot', 'shark', 'dragon'],
    24: ['saws', 'birds', 'drops'], 25: ['lava', 'cannon'], 26: ['elev', 'fanUp', 'bubble'], 27: ['kick', 'stairs'], 28: ['tramp', 'cannon', 'balloon', 'ballpit'],
    29: ['birds', 'piranha', 'pendulum'], 31: ['crush', 'rev', 'saws'], 32: ['moon', 'moonTramp'], 33: ['pool', 'piranha', 'ballpit'], 34: ['cannon'],
    35: ['lava', 'kick'], 36: ['pendulum', 'drops', 'wheel'], 37: ['hail', 'kick'], 38: ['birds'], 39: ['door', 'mover', 'elev'], 41: ['loop', 'wall', 'hump'],
    42: ['bubble', 'balloon', 'elev', 'cable'], 43: ['lava', 'shaft', 'drops'], 44: ['hail', 'kick', 'stairs'], 45: ['cannon', 'balloon'], 47: ['kick', 'tramp', 'moon'],
    48: ['lava', 'crush', 'saws'], 49: ['pendulum', 'hail', 'birds', 'drops'],
  };
  // every generated level gets its own background (themes are purely visual)
  const THEME_CYCLE = ['desert', 'beach', 'forest', 'snow', 'sunset', 'city', 'underwater', 'volcano', 'moon', 'space', 'sun', 'candy'];
  const HARD_THEME = { 51: 'desert', 52: 'moon', 53: 'sun', 54: 'underwater', 55: 'beach', 56: 'forest', 57: 'snow', 58: 'city', 59: 'volcano', 60: 'space',
    61: 'jungle', 62: 'cave', 63: 'beach', 64: 'jungle', 65: 'sunset', 66: 'cave', 67: 'underwater', 68: 'jungle', 69: 'cave', 70: 'volcano' };
  function genLevel(n, seed) {
    const hard = n > 50, world2 = n > 60;
    const rnd = rng(seed), d = world2 ? 1.45 + (n - 61) * 0.02 + (n === 70 ? 0.05 : 0) : hard ? Math.min(1.45, 1.15 + (n - 51) * 0.03 + (n % 10 === 0 ? 0.1 : 0)) : Math.min(1.15, (n - 11) / 39 + (n % 10 === 0 ? 0.25 : 0));
    const special = n % 10 === 0;
    const B = Builder(`Level ${n} · ${GEN_NAMES[n]}`);
    B.special = special;
    B.ceilMargin = Math.max(16, 28 - 12 * d);
    B.maxApproach = 960;
    B.theme = hard ? HARD_THEME[n] : n % 6 === 0 ? 'night' : THEME_CYCLE[(n - 11) % THEME_CYCLE.length];
    const icy = world2 ? false : hard ? n === 57 : n % 8 === 3 || n === 44 || n === 37;
    const lavaOn = n >= 15, lavaLevel = n >= 15 && (n % 5 === 0 || rnd() < 0.2);
    if (lavaLevel) B.lavaPits = true;
    const pick = a => a[Math.floor(rnd() * a.length)];
    const R = (a, b) => a + rnd() * (b - a);
    const I = (a, b) => Math.floor(R(a, b + 1));
    const lava = () => lavaOn && rnd() < 0.5;
    const run = len => (icy ? B.ice(len, 100) : B.flat(len, 100));
    const SECT = {
      kick: () => { run(icy ? 900 : 650); if (icy) B.maxApproach = 780; B.kickerGap({ h: R(45, 80), gap: R(240, 300) + d * 240, drop: rnd() < 0.4 ? R(0, 120) : 0, lava: lava() }); B.maxApproach = 960; },
      tramp: () => { run(450); B.trampGap({ gap: 380 + d * 220 + R(0, 60), power: R(900, 1000), lava: lava() }); },
      saws: () => B.saws(R(500, 800), I(1, 1 + Math.round(2 * d))),
      crush: () => B.crushers(R(500, 850), I(1, 1 + Math.round(2 * d))),
      flaps: () => { run(350); B.flapsBridge(I(3, 3 + Math.round(2 * d)), 200); },
      stairs: () => B.stairs(I(4, 7)),
      bumps: () => B.ceilBumps(R(600, 1100)),
      loop: () => { B.conv(400).flat(60).loop(R(125, 150), rnd() < 0.4); },
      hump: () => B.magnetHump(R(150, 200), 440),
      wall: () => { B.conv(350).flat(60).magnetWall(R(240, 320)); },
      pool: () => B.pool(560, 110, rnd() < 0.5),
      elev: () => B.elevator(R(380, 480)),
      mover: () => B.moverPit(),
      shaft: () => B.shaftDown(R(600, 750)),
      drop: () => B.drop(R(420, 480)),
      fanUp: () => B.fanUp(R(350, 430)),
      moon: () => { B.flat(200).conv(300); run(400); B.kickerGap({ h: R(55, 80), gap: 700 + d * 400, g: R(0.3, 0.4), landLen: 450, lava: lava() }); },
      moonTramp: () => { run(450); B.trampGap({ gap: 850 + d * 300, power: R(650, 750), g: R(0.3, 0.38), landLen: 450, lava: lava() }); },
      wheel: () => B.spikeWheel(R(550, 700)),
      rev: () => B.reverseConv(R(300, 500)),
      birds: () => B.birds(R(700, 1200), I(2, 2 + Math.round(2 * d)), 1.5 + d * 0.8),
      balloon: () => B.balloonGap(800 + d * 450 + R(0, 100), lava()),
      ballpit: () => B.ballPit(R(380, 480)),
      cannon: () => B.cannon(900 + d * 500 + R(0, 150), lava()),
      lava: () => B.lavaHops(I(3, 3 + Math.round(3 * d))),
      door: () => B.doorRoom(),
      piranha: () => B.piranhaPool(R(700, 1000), I(2, 2 + Math.round(2 * d))),
      hail: () => B.hailZone(R(700, 1100), I(3, 3 + Math.round(3 * d)), 1.2 - d * 0.25),
      pendulum: () => B.pendulums(R(1000, 1600), I(2, 2 + Math.round(2 * d)), 1.3 + d * 0.4),
      cable: () => B.cableCar(900 + d * 400, R(100, 250), lava()),
      bubble: () => B.bubbleLift(R(250, 310)),
      drops: () => B.dropSpikes(R(700, 1100), I(2, 2 + Math.round(3 * d))),
      lift: () => B.balanceLift(R(420, 520), 600 + d * 150),
      seesaw: () => B.seesaw(),
      fork: () => { run(icy ? 1400 : 650); B.forkTrap(); },
      flight: () => {
        const k = I(4, 7), c = []; let v = R(-150, 30);
        for (let i = 0; i < k; i++) { c.push(Math.round(v)); v = Math.max(-200, Math.min(60, v + R(-140, 140))); }
        B.flight(420 * (k + 1), c);
      },
      tunnel: () => B.wheelTunnel(R(500, 800)),
      carpet: () => B.spikeCarpet(R(500, 800)),
      fortune: () => B.fortuneWheel(),
      secret: () => B.secretBelt(),
      vine: () => B.vineJump(I(2, 4), R(330, 370)),
      aim: () => B.aimCannonShot(R(1000, 1500)),
      hot: () => {
        const k = I(4, 6), c = []; let v = R(-120, 20);
        for (let i = 0; i < k; i++) { c.push(Math.round(v)); v = Math.max(-180, Math.min(50, v + R(-110, 110))); }
        B.hotBalloonRide(520 * (k + 1), c);
      },
      shark: () => B.sharkLake(R(1000, 1300), I(1, 2)),
      dragon: () => B.dragonCave(R(1300, 1800), [R(1.8, 2.4), R(1.8, 2.4), R(1.8, 2.4)]),
    };
    const base = ['kick', 'tramp', 'saws', 'crush', 'flaps', 'stairs', 'bumps', 'loop', 'hump', 'wall', 'pool', 'elev', 'mover', 'shaft', 'drop', 'fanUp', 'moon', 'moonTramp', 'wheel', 'rev'];
    const unlocked = Object.keys(INTRO).filter(k => Number(k) <= n).map(k => INTRO[k]);
    const intro = INTRO[n];
    const focusList = FOCUS[n] || (intro ? [intro] : null);
    const focusPick = () => focusList ? pick(focusList) : null;
    const focus = focusPick();
    const extreme = ['lift', 'seesaw', 'fork', 'flight', 'tunnel', 'carpet', 'secret'];
    const w2 = ['vine', 'aim', 'hot', 'shark', 'dragon'];
    const pool = world2 ? base.concat(unlocked, extreme, w2, w2, w2) : hard ? base.concat(unlocked, extreme, extreme) : base.concat(unlocked);
    const count = hard ? 15 + (special ? 4 : 0) : 7 + Math.round(7 * Math.min(1, d)) + (special ? 3 : 0);
    const cpEvery = d < 0.5 ? 3 : 4;
    B.begin(0, 0).flat(120).startHere(special ? 'EXTRA' : 'START').flat(250, 80);
    let lastKey = null, row = 0;
    const colors = ['#2e7d32', '#e91e63', '#7b1fa2', '#00bcd4', '#ff9800', '#3f51b5'];
    for (let i = 0; i < count; i++) {
      if (i > 0 && i % cpEvery === 0) { B.flat(250).checkpoint(); }
      let key = i === 0 && focus ? focus : (focusList && rnd() < 0.35 ? focusPick() : pick(pool));
      if (key === lastKey) key = pick(pool);
      lastKey = key;
      B.mark(); run(R(380, 480));
      const sec = { key, g0: B.gr.length, x0: B.x() };
      SECT[key]();
      B.flat(150);
      sec.g1 = B.gr.length; sec.x1 = B.x(); B.sections.push(sec);
      if (B.x() > 7000 && i < count - 1) { // new row through a portal
        const [, hi] = B.bounds();
        B.flat(200).portalTo(0, hi + 1100, colors[row++ % colors.length]).flat(200);
      }
    }
    if ([53, 56, 59, 60].includes(n)) { // the slingshot guards the finish (no random numbers used, so the rest stays the same)
      B.mark(); run(450); B.checkpoint(); B.flat(300); B.mark();
      const sec = { key: 'sling', g0: B.gr.length, x0: B.x() };
      B.slingshot(); B.flat(150);
      sec.g1 = B.gr.length; sec.x1 = B.x(); B.sections.push(sec);
    }
    B.mark(); run(450);
    B.finishLine(700, n % 10 === 5);
    if (special) B.coins.forEach((c, i) => { if (i % 6 === 3) c.v = 5; });
    return B.build();
  }
  // seeds picked by the offline validator (every level is solvable by the test bot)
  const SEEDS = { 11: 11000, 12: 12000, 13: 13000, 14: 14000, 15: 15000, 16: 16000, 17: 17000, 18: 18001, 19: 19000, 20: 20000, 21: 21000, 22: 22000, 23: 23000, 24: 24000, 25: 25000, 26: 26005, 27: 27007, 28: 28000, 29: 29000, 30: 30000, 31: 31000, 32: 32001, 33: 33000, 34: 34000, 35: 35007, 36: 36000, 37: 37004, 38: 38000, 39: 39002, 40: 40000, 41: 41000, 42: 42000, 43: 43000, 44: 44010, 45: 45000, 46: 46001, 47: 47000, 48: 48000, 49: 49001, 50: 50000, 51: 51000, 52: 52011, 53: 53003, 54: 54004, 55: 55000, 56: 56002, 57: 57017, 58: 58004, 59: 59000, 60: 60000, 61: 61001, 62: 62123, 63: 63000, 64: 64011, 65: 65000, 66: 66001, 67: 67001, 68: 68000, 69: 69006, 70: 70004 };
  const LEVELS = [level1, level2, level3, level4, level5, level6, level7, level8, level9, level10];
  for (let n = 11; n <= 70; n++) LEVELS.push(() => genLevel(n, SEEDS[n] !== undefined ? SEEDS[n] : n * 1000));

  // ---------- geometry ----------
  function segIntersect(px, py, qx, qy, s) {
    const rx = qx - px, ry = qy - py, sx = s.bx - s.ax, sy = s.by - s.ay;
    const den = rx * sy - ry * sx;
    if (Math.abs(den) < 1e-9) return -1;
    const ux = s.ax - px, uy = s.ay - py;
    const t = (ux * sy - uy * sx) / den;
    const u = (ux * ry - uy * rx) / den;
    if (t >= 0 && t <= 1 && u >= 0 && u <= 1) return t;
    return -1;
  }
  function angDiff(a, b) { let d = a - b; while (d > PI) d -= 2 * PI; while (d < -PI) d += 2 * PI; return d; }

  // ---------- game ----------
  function createGame(levelIndex) {
    const L = Object.assign({ gates: [], buttons: [], movers: [], flaps: [], saws: [], fans: [], portals: [], waters: [], crushers: [], lowG: [], birds: [], floaters: [], cannons: [], doors: [], piranhas: [], hail: [], pendulums: [], dropSpikes: [], seesaws: [], fortunes: [], wheelSwitches: [], slings: [], vines: [], aimCannons: [], sharks: [], dragons: [], jumpZones: [], deathRects: [], walls: [] }, typeof levelIndex === 'function' ? levelIndex() : LEVELS[levelIndex || 0]());
    const coins = L.coins.map(c => ({ x: c.x, y: c.y, v: c.v || 1, taken: false }));
    L.gates.forEach(g => { g.open = 0; g.opening = false; });

    const S = {
      level: L, levelIndex: typeof levelIndex === 'number' ? levelIndex : 0, coins, time: 0, clock: 0, deaths: 0, status: 'ready',
      checkpoint: null, msg: '', events: [],
    };

    function placeOnGround(gi, x) {
      const g = L.grounds[gi];
      let si = 0;
      for (let i = 0; i < g.segs.length; i++) {
        const s = g.segs[i];
        if ((x - s.ax) * (x - s.bx) <= 0) { si = i; break; }
      }
      const s = g.segs[si];
      const u = Math.min(s.len, Math.max(0, (x - s.ax) / (Math.abs(s.tx) > 1e-6 ? s.tx : 1)));
      Object.assign(S, { mode: 'ground', gi, si, u, v: 0, facing: g.side, wheel: 0, airTime: 0, portalCd: 0 });
      syncFromGround();
    }

    function syncFromGround() {
      const g = L.grounds[S.gi], s = g.segs[S.si];
      S.x = s.ax + s.tx * S.u; S.y = s.ay + s.ty * S.u;
      S.theta = Math.atan2(s.nx, -s.ny);
      S.facing = g.side;
      S.vx = s.tx * S.v + g.vx; S.vy = s.ty * S.v + g.vy;
    }

    function resetDynamic() {
      L.gates.forEach(g => { g.open = 0; g.opening = false; });
      L.buttons.forEach(b => (b.pressed = false));
      L.movers.forEach(m => { m.t = 0; m.active = m.mode === 'loop'; applyMover(m, 0); });
      L.flaps.forEach(f => { f.state = 'idle'; f.timer = 0; const g = L.grounds[f.gi]; g.rot = 0; transformGround(g); });
      L.doors.forEach(d => { d.open = 0; d.opening = false; });
      L.floaters.forEach(f => (f.taken = false));
      L.dropSpikes.forEach(d => { d.state = 'hang'; d.y = d.y0; d.vy = 0; });
      S.float = null; S.fly = null; S.cannonCd = 0; S.wheels = 1; S.bonus = false; S.finishTime = undefined; S.sling = null; S.slingShot = undefined; S.slingCd = 0;
      S.vine = null; S.vineCd = {}; S.aim = null; S.hot = null; S.jumpHeld = false;
      L.seesaws.forEach(w => { w.rot = -w.maxA; w.omega = 0; const g = L.grounds[w.gi]; g.rot = w.rot; transformGround(g); });
      L.fortunes.forEach(f => (f.used = false));
      L.grounds.forEach(g => { if (g.flipped) { g.base.forEach(q => (q[3] = -q[3])); g.flipped = false; transformGround(g); } });
    }

    function reset(full) {
      if (full) {
        S.checkpoint = null; S.time = 0; S.deaths = 0;
        coins.forEach(c => (c.taken = false));
      }
      resetDynamic();
      if (S.checkpoint) {
        placeOnGround(S.checkpoint.gi, S.checkpoint.x);
        coins.forEach(c => { if (c.cp) c.taken = false; });
      } else placeOnGround(L.start.gi, L.start.x);
      S.status = full ? 'ready' : 'run';
      S.msg = '';
      S.events = [];
    }

    function crash(msg) {
      if (S.status !== 'run') return;
      S.status = 'crash'; S.msg = msg; S.deaths++;
      S.events.push('crash');
    }

    function bikePoints() {
      const ux = Math.sin(S.theta), uy = -Math.cos(S.theta);
      const fx = S.facing * Math.cos(S.theta), fy = S.facing * Math.sin(S.theta);
      // wheel size switch: big wheels lift the bike, small wheels duck the rider
      const ws = S.wheels || 1, wr = WHEEL_R * ws, lift = wr - WHEEL_R;
      const head = ws < 1 ? 48 : HEAD + lift * 1.2;
      return {
        up: [ux, uy], fwd: [fx, fy],
        pos: [S.x, S.y],
        center: [S.x + ux * (30 + lift), S.y + uy * (30 + lift)],
        head: [S.x + ux * head, S.y + uy * head],
        front: [S.x + fx * (WHEEL_X + wr) + ux * wr, S.y + fy * (WHEEL_X + wr) + uy * wr],
        rear: [S.x - fx * (WHEEL_X + wr) + ux * wr, S.y - fy * (WHEEL_X + wr) + uy * wr],
      };
    }

    function takeoff(vx, vy) {
      const g = L.grounds[S.gi];
      S.mode = 'air'; S.vx = vx + g.vx; S.vy = vy + g.vy; S.airTime = 0; S.leftGi = S.gi;
    }

    function bounce(s) {
      const vt = S.vx * s.tx + S.vy * s.ty;
      S.theta = Math.atan2(s.nx, -s.ny);
      S.mode = 'air'; S.airTime = 0; S.leftGi = S.gi;
      const pw = s.conv || TRAMP_V;
      S.vx = s.tx * vt + s.nx * pw; S.vy = s.ty * vt + s.ny * pw;
      S.x += s.nx * 2; S.y += s.ny * 2;
      S.events.push('tramp');
    }

    function stepGround(dt, inp) {
      const g = L.grounds[S.gi];
      let s = g.segs[S.si];
      let a = G * s.ty;
      const ice = s.kind === 'ice';
      if (inp.up) a += GAS * (ice ? 0.45 : 1);
      const br = BRAKE * (ice ? 0.22 : 1);
      if (inp.down) {
        // brake while rolling forward; once stopped, holding the brake drives backwards
        if (S.v > 0.5) a -= br;
        else if (S.v > -REV_MAX) a -= REV_ACC * (ice ? 0.45 : 1);
        else a += br;
      }
      if (g.convWave) a += g.convWave.amp * Math.sin(g.convWave.phase + g.convWave.speed * S.clock); // balance lift: belt pushes back and forth
      else if (s.kind === 'conv' && S.v * s.conv < CONV_MAX) a += s.conv * CONV_ACC * (s.conv < 0 ? 0.47 : 1); // backward belts are beatable with full gas (strong ones are not)
      if (inWater()) a -= S.v * 2.2;
      a -= Math.sign(S.v) * (ice ? 0 : ROLL) + AIR * S.v * Math.abs(S.v);
      const v0 = S.v;
      S.v += a * dt;
      if (inp.down && v0 > 0.5 && S.v < 0 && Math.abs(G * s.ty) < br) S.v = 0; // stop first, reverse on the next tick
      if (Math.abs(S.v) < 3 && !inp.up && !inp.down && Math.abs(G * s.ty) < ROLL + 5 && s.kind !== 'conv') S.v = 0;
      // overhang / wall: no normal force → fall off (magnets hold you)
      if (s.kind !== 'magnet') {
        // normal force = gravity part + centripetal part on concave curves (loops)
        const nx = g.segs[S.si + 1] || g.segs[S.si - 1] || s;
        const turnIn = nx.tx * s.nx + nx.ty * s.ny > 0 && nx !== s;
        const kappa = turnIn ? Math.acos(Math.min(1, s.tx * nx.tx + s.ty * nx.ty)) / ((s.len + nx.len) / 2) : 0;
        if (-G * s.ny + S.v * S.v * kappa < -30) { syncFromGround(); takeoff(s.tx * S.v, s.ty * S.v); return; }
      }
      for (const f of L.fans) if (f.vt < 0 && inRectP(S.x, S.y - 30, f)) { syncFromGround(); takeoff(s.tx * S.v, s.ty * S.v - 60); return; }

      let dist = S.v * dt;
      let guard = 0;
      while (Math.abs(dist) > 1e-9 && guard++ < 200) {
        s = g.segs[S.si];
        if (dist > 0) {
          const rem = s.len - S.u;
          if (dist <= rem) { S.u += dist; dist = 0; break; }
          dist -= rem;
          if (S.si + 1 >= g.segs.length) {
            S.u = s.len; syncFromGround(); takeoff(s.tx * S.v, s.ty * S.v);
            S.x += S.vx * (dist / Math.abs(S.v)); S.y += S.vy * (dist / Math.abs(S.v));
            return;
          }
          const n = g.segs[S.si + 1];
          if (vertexDetach(s, n)) { S.u = s.len; syncFromGround(); takeoff(s.tx * S.v, s.ty * S.v); return; }
          S.v *= Math.max(0, s.tx * n.tx + s.ty * n.ty);
          S.si++; S.u = 0;
        } else {
          if (-dist <= S.u) { S.u += dist; dist = 0; break; }
          dist += S.u;
          if (S.si === 0) {
            S.u = 0; syncFromGround(); takeoff(s.tx * S.v, s.ty * S.v); return;
          }
          const n = g.segs[S.si - 1];
          if (vertexDetach(n, s)) { S.u = 0; syncFromGround(); takeoff(s.tx * S.v, s.ty * S.v); return; }
          S.v *= Math.max(0, s.tx * n.tx + s.ty * n.ty);
          S.si--; S.u = n.len;
        }
      }
      syncFromGround();
      S.wheel += S.v * dt / WHEEL_R;
      s = g.segs[S.si];
      if (s.kind === 'tramp') bounce(s);
    }

    // a: segment before vertex, b: segment after (in polyline order)
    function vertexDetach(a, b) {
      if (a.kind === 'magnet' || b.kind === 'magnet') return false;
      const dot = a.tx * b.tx + a.ty * b.ty;
      const turnAway = b.tx * a.nx + b.ty * a.ny < -1e-6;
      if (!turnAway) return false;
      const ang = Math.acos(Math.min(1, Math.max(-1, dot)));
      const kappa = ang / ((a.len + b.len) / 2);
      const ny = (a.ny + b.ny) / 2;
      const N = -G * ny - S.v * S.v * kappa;
      return N < -30;
    }

    function stepAir(dt, inp) {
      S.airTime += dt;
      // nose slowly follows the trajectory
      let target = Math.atan2(S.facing * S.vy, S.facing * S.vx);
      target = Math.max(-0.8, Math.min(0.8, target));
      S.theta += angDiff(target, S.theta) * ASSIST * dt;
      S.wheel += (inp.up ? 30 : 0) * dt;

      let gf = 1;
      for (const z of L.lowG) if (inRectP(S.x, S.y, z)) gf = z.g;
      if (S.hot) { // hot-air balloon: gas = burner (up), brake = vent (down), the wind pushes you on
        S.hot.t -= dt;
        S.vx += (170 - S.vx) * Math.min(1, 2 * dt);
        S.vy += (inp.up ? -560 : inp.down ? 480 : 150) * dt;
        S.vy *= Math.exp(-0.6 * dt); S.vy = Math.max(-280, Math.min(280, S.vy)) - G * gf * dt;
        if (S.hot.t <= 0) S.hot = null;
      }
      for (let i = 0; i < L.vines.length; i++) { // grab a vine when you fly past its end
        if (S.vineCd[i] > 0) continue;
        const vn = L.vines[i], th = vineIdle(vn), ex = vn.px + vn.len * Math.sin(th), ey = vn.py + vn.len * Math.cos(th);
        if (Math.hypot(S.x - ex, S.y - 30 - ey) < 70) {
          const a = Math.atan2(S.x - vn.px, S.y - 30 - vn.py);
          S.mode = 'vine'; S.vine = { i, th: a, om: (S.vx * Math.cos(a) - S.vy * Math.sin(a)) / vn.len, held: inp.down };
          S.float = null; S.fly = null; S.hot = null; S.events.push('vine');
          return;
        }
      }
      const bz = L.bonusZone;
      if (S.bonus && bz && S.x > bz.x - 50 && S.x < bz.x + bz.w + 50 && S.y > bz.y) { // bonus sky dive: slow fall, steer left/right
        if (inp.up) S.vx += 1200 * dt;
        if (inp.down) S.vx -= 1200 * dt;
        S.vx *= Math.exp(-1.5 * dt); S.vx = Math.max(-420, Math.min(420, S.vx));
        S.vy += (230 - S.vy) * Math.min(1, 2 * dt) - G * gf * dt;
        if (S.x < bz.x + 40) { S.x = bz.x + 40; S.vx = Math.abs(S.vx) * 0.3; }
        if (S.x > bz.x + bz.w - 40) { S.x = bz.x + bz.w - 40; S.vx = -Math.abs(S.vx) * 0.3; }
      }
      if (S.fly) { // flight mode: steady forward speed, gas = up, brake = down
        S.fly.t -= dt;
        S.vx += (360 - S.vx) * Math.min(1, 3 * dt);
        if (inp.up) S.vy -= 1300 * dt;
        if (inp.down) S.vy += 1300 * dt;
        S.vy *= Math.exp(-1.6 * dt);
        S.vy = Math.max(-400, Math.min(400, S.vy)) - G * gf * dt;
        if (S.fly.t <= 0) { S.fly = null; S.events.push('pop'); }
      }
      if (S.float) {
        S.float.t -= dt;
        if (S.float.kind === 'balloon') {
          // balloon: rises gently, gas/brake steer left/right
          S.vy += (-130 - S.vy) * Math.min(1, 3 * dt) - G * gf * dt;
          if (inp.up) S.vx += 450 * dt; if (inp.down) S.vx -= 450 * dt;
          S.vx = Math.max(-350, Math.min(380, S.vx));
        } else {
          S.vy += (-200 - S.vy) * Math.min(1, 3 * dt) - G * gf * dt;
          S.vx += (80 - S.vx) * Math.min(1, 4 * dt);
        }
        if (S.float.t <= 0) { S.float = null; S.events.push('pop'); }
      }
      S.vy += G * gf * dt;
      if (S.slingShot !== undefined) { // flying from the slingshot: judged when crossing the target board
        const sl = L.slings[S.slingShot], tg = sl.target;
        const nx = S.x + S.vx * dt;
        if (S.x < tg.x && nx >= tg.x) {
          const cy = S.y - 30 + S.vy * dt * (tg.x - S.x) / (nx - S.x), dd = Math.abs(cy - tg.y);
          if (dd < tg.r) {
            S.slingShot = undefined; S.x = sl.exit.x; S.y = sl.exit.y; S.vx = 260; S.vy = 0; S.theta = 0; S.airTime = 0; S.leftGi = -1;
            S.events.push('bullseye'); return;
          }
          crash(dd < tg.board ? 'Daneben! Triff ins Schwarze.' : 'Vorbei! Triff ins Schwarze.'); return; // over or under the board counts as a miss too
        }
      }
      for (const f of L.fans) if (inRectP(S.x, S.y - 30, f)) S.vy += (f.vt - S.vy) * Math.min(1, 5 * dt) - G * gf * dt; // fan carries you: vertical speed goes to vt
      const wz = inWater();
      if (wz) { if (wz.style !== 'balls') S.vy -= G * 0.6 * dt; const k = Math.exp(-(wz.style === 'balls' ? 3 : 2) * dt); S.vx *= k; S.vy *= k; }
      const sp = Math.hypot(S.vx, S.vy);
      S.vx -= AIR * 0.5 * S.vx * sp * dt; S.vy -= AIR * 0.5 * S.vy * sp * dt;
      const px = S.x, py = S.y;
      const qx = px + S.vx * dt, qy = py + S.vy * dt;

      let best = null;
      for (let gi = 0; gi < L.grounds.length; gi++) {
        const gnd = L.grounds[gi], segs = gnd.segs;
        for (let si = 0; si < segs.length; si++) {
          const s = segs[si];
          if ((S.vx - gnd.vx) * s.nx + (S.vy - gnd.vy) * s.ny >= -1) continue;
          const t = segIntersect(px, py, qx, qy, s);
          if (t < 0) continue;
          if (S.airTime < 0.05 && gi === S.leftGi && t < 1e-3) continue;
          if (!best || t < best.t) best = { t, gi, si, s };
        }
      }
      if (best) {
        const s = best.s, gnd = L.grounds[best.gi];
        S.x = px + (qx - px) * best.t; S.y = py + (qy - py) * best.t;
        const ux = Math.sin(S.theta), uy = -Math.cos(S.theta);
        const ang = Math.acos(Math.max(-1, Math.min(1, ux * s.nx + uy * s.ny)));
        const rvx = S.vx - gnd.vx, rvy = S.vy - gnd.vy;
        let vn = -(rvx * s.nx + rvy * s.ny);
        S.gi = best.gi; S.si = best.si;
        if (s.kind === 'tramp') { bounce(s); return; }
        if (s.kind !== 'magnet') {
          if (s.kind === 'soft') vn = 0;
          if (ang > LAND_ANGLE_MAX) { crash('Falsch gelandet! Achte auf dein Tempo.'); return; }
          if (vn > IMPACT_MAX) { crash('Zu harte Landung! Weniger Tempo.'); return; }
        }
        S.mode = 'ground'; S.float = null; S.fly = null; S.hot = null; S.slingShot = undefined;
        S.u = Math.hypot(S.x - s.ax, S.y - s.ay);
        S.v = rvx * s.tx + rvy * s.ty;
        S.events.push('land');
        syncFromGround();
        return;
      }
      S.x = qx; S.y = qy;
    }

    function inRectP(x, y, r) { return x >= r.x && x <= r.x + r.w && y >= r.y && y <= r.y + r.h; }
    function inWater() { for (const w of L.waters) if (inRectP(S.x, S.y - 20, w)) return w; return null; }

    // every moving hazard as a shape at time t: {c: [x, y, r]} circle or {r: rect}
    function timedShapes(t) {
      const out = [];
      for (const sw of L.saws) { const a = sw.phase + sw.speed * t; out.push({ c: [sw.cx + sw.orbit * Math.cos(a), sw.cy + sw.orbit * Math.sin(a), sw.r + 6], msg: sw.style === 'spike' ? 'Vom Stachelrad erwischt!' : 'Von der Säge erwischt!' }); }
      for (const c of L.crushers) out.push({ r: crusherRect(c, t), msg: 'Zerquetscht!' });
      for (const b of L.birds) { const p = birdPos(b, t); out.push({ c: [p[0], p[1], 24], msg: 'Vom Vogel erwischt!' }); }
      for (const pd of L.pendulums) { const p = pendulumPos(pd, t); out.push({ c: [p[0], p[1], pd.r], msg: 'Vom Pendel getroffen!' }); }
      for (const f of L.piranhas) { const p = piranhaPos(f, t); out.push({ c: [p[0], p[1], 22], msg: 'Piranha-Biss!' }); }
      for (const h of L.hail) { const p = hailPos(h, t); if (p) out.push({ c: [p[0], p[1], 15], msg: 'Vom Eisregen getroffen!' }); }
      for (const sh of L.sharks) { const p = sharkPos(sh, t); out.push({ c: [p[0] - 14, p[1], 24], msg: 'Vom Hai gefressen!' }, { c: [p[0] + 18, p[1], 22], msg: 'Vom Hai gefressen!' }); }
      for (const dg of L.dragons) for (const fb of fireballs(dg, t)) out.push({ c: [fb[0], fb[1], 22], msg: 'Vom Feuerball des Drachen erwischt!' });
      return out;
    }
    function birdPos(b, t) { const a = b.phase + b.speed * t; return [b.cx + b.ax * Math.sin(a), b.cy + b.ay * Math.sin(2 * a), Math.cos(a)]; }
    function pendulumPos(p, t) { const th = p.amp * Math.sin(p.phase + p.speed * t); return [p.px + p.L * Math.sin(th), p.py + p.L * Math.cos(th), th]; }
    function piranhaPos(f, t) { const s = Math.sin(f.phase + f.speed * t); return [f.x, f.surface + (f.base !== undefined ? f.base : 25) - Math.max(0, s) * f.h, s]; }
    function hailPos(h, t) {
      const fr = (((t / h.period + h.phase) % 1) + 1) % 1, y = h.top + fr * (h.bottom - h.top + 60) - 30;
      return y >= h.top && y <= h.bottom ? [h.x, y] : null;
    }
    function sharkPos(sh, t) { const a = sh.phase + sh.speed * t; return [sh.cx + sh.ax * Math.sin(a), sh.y + 6 * Math.sin(a * 3), Math.cos(a)]; }
    // the dragon spits fireballs that roll along the cave floor towards you
    function fireballs(dg, t) {
      const out = [], gaps = dg.gaps, cyc = gaps.reduce((a, b) => a + b, 0);
      const life = (dg.x - dg.x0) / dg.speed;
      const base = Math.floor((t - dg.phase) / cyc);
      for (let c = base - 1; c <= base; c++) {
        let st = dg.phase + c * cyc;
        for (const gp of gaps) {
          const age = t - st;
          if (age >= 0 && age < life) out.push([dg.x - dg.speed * age, dg.y - 26, age]);
          st += gp;
        }
      }
      return out;
    }
    function vineIdle(vn) { return 0.12 * Math.sin(1.3 * S.clock + vn.px * 0.01); }
    function crusherRect(c, t) {
      if (t === undefined) t = S.clock;
      const yb = c.ground - 85 + 85 * Math.sin(c.phase + c.speed * t); // bottom edge of the block
      return { x: c.x, y: yb - 140, w: c.w, h: 140 };
    }
    // slingshot: the arrow swings until you press gas (direction), then the power swings until you let go
    // vine: swing like a pendulum, gas pumps the swing, brake lets go
    function stepVine(dt, inp) {
      const vg = S.vine, vn = L.vines[vg.i];
      if (!inp.down) vg.held = false;
      vg.om += (-(G / vn.len) * Math.sin(vg.th) + (inp.up ? 2.4 * (vg.om >= 0 ? 1 : -1) : 0)) * dt;
      vg.om *= Math.exp(-0.04 * dt);
      vg.th += vg.om * dt;
      if (Math.abs(vg.th) > 1.4) { vg.th = Math.sign(vg.th) * 1.4; vg.om = 0; }
      S.x = vn.px + vn.len * Math.sin(vg.th); S.y = vn.py + vn.len * Math.cos(vg.th) + 30; S.theta = -vg.th * 0.5; S.facing = 1;
      if (inp.down && !vg.held) {
        S.mode = 'air'; S.airTime = 0; S.leftGi = -1;
        S.vx = vn.len * vg.om * Math.cos(vg.th); S.vy = -vn.len * vg.om * Math.sin(vg.th);
        S.vineCd[vg.i] = 0.8; S.vine = null; S.events.push('vineOff');
      }
    }
    // aimable cannon: gas raises the barrel, brake lowers it, fires when the countdown ends
    function stepAim(dt, inp) {
      const am = S.aim, c = L.aimCannons[am.i];
      if (inp.up) am.angle += AIM.turn * dt;
      if (inp.down) am.angle -= AIM.turn * dt;
      am.angle = Math.max(AIM.min, Math.min(AIM.max, am.angle));
      S.theta = -am.angle;
      am.t -= dt;
      if (am.t <= 0) {
        const ca = Math.cos(am.angle), sa = Math.sin(am.angle);
        S.mode = 'air'; S.airTime = 0; S.leftGi = -1; S.cannonCd = 1.2;
        S.x = c.x + ca * 80; S.y = c.y - 46 - sa * 80; S.vx = ca * c.power; S.vy = -sa * c.power;
        S.aim = null; S.events.push('cannon');
      }
    }
    function stepSling(dt, inp) {
      const sg = S.sling;
      if (!inp.up) sg.armed = true;
      if (!sg.charging) {
        sg.t += dt;
        sg.angle = SLING.mid + SLING.amp * Math.sin(SLING.aimSpeed * sg.t);
        S.theta = -sg.angle;
        if (inp.up && sg.armed) { sg.charging = true; sg.ct = 0; S.events.push('slingAim'); }
      } else if (inp.down) {
        sg.charging = false; sg.armed = false;
      } else if (inp.up) {
        sg.ct += dt;
        const ph = (sg.ct / SLING.powerTime) % 2;
        sg.power = SLING.pmin + (SLING.pmax - SLING.pmin) * (ph < 1 ? ph : 2 - ph);
      } else { // let go: fire
        const sl = L.slings[sg.i];
        S.mode = 'air'; S.airTime = 0; S.leftGi = -1; S.slingShot = sg.i; S.slingCd = 1.5;
        S.vx = Math.cos(sg.angle) * sg.power; S.vy = -Math.sin(sg.angle) * sg.power;
        S.x = sl.x + 30; S.y = sl.y - 40; S.theta = -sg.angle;
        S.sling = null; S.events.push('slingShot');
      }
    }
    function sawPos(sw) {
      const a = sw.phase + sw.speed * S.clock;
      return [sw.cx + sw.orbit * Math.cos(a), sw.cy + sw.orbit * Math.sin(a)];
    }

    function hazards(prev) {
      const P = bikePoints();
      const pts = [P.pos, P.head, P.front, P.rear, P.center];
      for (const g of L.grounds) for (const s of g.segs) {
        if (s.kind !== 'magnet' && s.kind !== 'loop' && segIntersect(prev.head[0], prev.head[1], P.head[0], P.head[1], s) >= 0) { crash('Kopfüber! Autsch.'); return; }
      }
      const keys = ['pos', 'head', 'front'];
      for (const w of L.walls) for (const k of keys) {
        if (segIntersect(prev[k][0], prev[k][1], P[k][0], P[k][1], w) >= 0) { crash('Gegen die Wand gekracht!'); return; }
      }
      const inRect = (p, r) => p[0] >= r.x && p[0] <= r.x + r.w && p[1] >= r.y && p[1] <= r.y + r.h;
      for (const sp of L.spikes) if (!(sp.low && S.wheels > 1.2)) for (const p of pts) if (inRect(p, sp)) { crash(sp.dir === 'lava' ? 'In die Lava gefallen!' : 'Stacheln berührt!'); return; }
      for (const gt of L.gates) {
        const top = Math.max(gt.ceiling, gt.top - gt.open), bottom = gt.bottom - gt.open;
        if (bottom > top) {
          const r = { x: gt.x, y: top, w: gt.w, h: bottom - top };
          for (const p of pts) if (inRect(p, r)) { crash('Stacheln berührt! Drück zuerst den Knopf.'); return; }
        }
      }
      for (const h of timedShapes(S.clock)) {
        for (const p of pts) {
          if (h.c ? Math.hypot(p[0] - h.c[0], p[1] - h.c[1]) < h.c[2] : inRect(p, h.r)) { crash(h.msg); return; }
        }
      }
      for (const d of L.dropSpikes) {
        if (d.state === 'gone') continue;
        const r = { x: d.x - 16, y: d.y - 46, w: 32, h: 46 };
        for (const p of pts) if (inRect(p, r)) { crash('Von herabfallenden Stacheln getroffen!'); return; }
      }
      // floaters: balloons and soap bubbles carry you
      for (const f of L.floaters) {
        if (f.taken || Math.hypot(P.center[0] - f.x, P.center[1] - f.y) > 50) continue;
        f.taken = true; S.events.push(f.kind);
        if (f.kind === 'wings') { S.fly = { t: 30 }; S.float = null; }
        else if (f.kind === 'hot') { S.hot = { t: 60 }; S.float = null; }
        else S.float = { kind: f.kind, t: f.kind === 'balloon' ? 4.2 : 2.8 };
        if (S.mode === 'ground') { syncFromGround(); takeoff(S.vx - L.grounds[S.gi].vx, -80); }
      }
      // cannons
      if (S.cannonCd > 0) S.cannonCd -= 1 / 240;
      else for (const c of L.cannons) {
        if (S.mode === 'ground' && Math.abs(S.x - c.x) < 26 && Math.abs(S.y - c.y) < 20) {
          const ca = Math.cos(c.angle), sa = Math.sin(c.angle);
          S.mode = 'air'; S.airTime = 0; S.leftGi = -1; S.float = null;
          S.x = c.x + ca * 80; S.y = c.y - 46 - sa * 80; S.vx = ca * c.power; S.vy = -sa * c.power;
          S.theta = -c.angle; S.facing = 1; S.cannonCd = 1; S.events.push('cannon');
          return;
        }
      }
      for (const r of L.deathRects) if (inRect(P.pos, r)) { crash('Abgestürzt!'); return; }
      if (S.y > L.deathY) { crash('Abgestürzt!'); return; }
      // buttons
      for (const b of L.buttons) {
        if (b.pressed) continue;
        for (const p of [P.pos, P.front, P.rear]) {
          if (p[0] >= b.x && p[0] <= b.x + b.w && p[1] >= b.y - 14 && p[1] <= b.y + 4) {
            b.pressed = true; S.events.push('button');
            if (b.target.type === 'gate') L.gates[b.target.i].opening = true;
            else if (b.target.type === 'door') L.doors[b.target.i].opening = true;
            else if (b.target.type === 'flip') { const g = L.grounds[b.target.gi]; g.base.forEach(q => (q[3] = -q[3])); g.flipped = !g.flipped; transformGround(g); }
            else L.movers[b.target.i].active = true;
            break;
          }
        }
      }
      // portals
      if (S.portalCd > 0) S.portalCd -= 1 / 240;
      else for (const pt of L.portals) {
        if (Math.hypot(P.center[0] - pt.ax, P.center[1] - pt.ay) < 48) {
          const sp = Math.max(150, Math.hypot(S.vx, S.vy));
          S.mode = 'air'; S.airTime = 0; S.leftGi = -1;
          S.x = pt.bx; S.y = pt.by; S.vx = sp; S.vy = 0; S.theta = 0; S.facing = 1;
          S.portalCd = 0.5; S.events.push('portal');
          return;
        }
      }
      for (const k in S.vineCd) if (S.vineCd[k] > 0) S.vineCd[k] -= 1 / 240;
      if (S.mode === 'ground' && S.cannonCd <= 0) for (let i = 0; i < L.aimCannons.length; i++) {
        const c = L.aimCannons[i];
        if (Math.abs(S.x - c.x) < 24 && Math.abs(S.y - c.y) < 20) {
          S.mode = 'aim'; S.v = 0; S.vx = 0; S.vy = 0; S.x = c.x; S.y = c.y - 46; S.theta = -0.6;
          S.aim = { i, angle: 0.6, t: AIM.time };
          S.events.push('aimIn');
          return;
        }
      }
      if (S.slingCd > 0) S.slingCd -= 1 / 240;
      if (S.mode === 'ground' && S.slingCd <= 0) for (let i = 0; i < L.slings.length; i++) {
        const sl = L.slings[i];
        if (Math.abs(S.x - sl.x) < 20 && Math.abs(S.y - sl.y) < 20) {
          S.mode = 'sling'; S.v = 0; S.vx = 0; S.vy = 0; S.x = sl.x + 30; S.y = sl.y - 40; S.theta = -SLING.mid; S.facing = 1; // sits in the pocket
          S.sling = { i, t: 0, angle: SLING.mid, charging: false, armed: false, ct: 0, power: SLING.pmin };
          S.events.push('slingIn');
          return;
        }
      }
      if (S.mode === 'ground') {
        for (const f of L.fortunes) if (!f.used && Math.abs(S.x - f.x) < 30 && Math.abs(S.y - f.y) < 30) { f.used = true; S.fortuneHit = f; S.events.push('fortune'); }
        for (const w of L.wheelSwitches) if (Math.abs(S.x - w.x) < 22 && Math.abs(S.y - w.y) < 20 && S.wheels !== w.size) { S.wheels = w.size; S.events.push('wheels'); }
      }
      // coins
      for (const c of coins) {
        if (c.taken) continue;
        if (Math.hypot(c.x - P.center[0], c.y - P.center[1]) < 42) {
          c.taken = true; c.cp = !!S.checkpoint; S.lastCoin = c; S.events.push(c.v > 1 ? 'star' : 'coin');
        }
      }
      // checkpoints
      if (S.mode === 'ground') {
        for (const cp of L.checkpoints) {
          if (S.gi === cp.gi && (S.x - cp.tx) * cp.dir >= 0 && S.checkpoint !== cp &&
              L.checkpoints.indexOf(cp) > L.checkpoints.indexOf(S.checkpoint)) {
            S.checkpoint = cp; S.events.push('checkpoint');
            coins.forEach(c => (c.cp = false));
          }
        }
        const f = L.finish;
        if (S.gi === f.gi && (S.x - f.x) * f.dir >= 0 && !S.bonus) {
          if (f.bonus) { S.bonus = true; S.finishTime = S.time; S.events.push('bonus'); } // time stops here, the bonus is extra
          else { S.status = 'finish'; S.events.push('finish'); }
        }
        if (S.bonus && S.gi === L.bonusEnd) { S.status = 'finish'; S.events.push('finish'); }
      }
    }

    function applyMover(m, dt) {
      const g = L.grounds[m.gi];
      const len = Math.hypot(m.to[0] - m.from[0], m.to[1] - m.from[1]);
      const prevT = m.t;
      if (m.mode === 'button') { if (m.active) m.t = Math.min(1, m.t + m.speed * dt / len); }
      else {
        const travel = len / m.speed, dwell = m.dwell || 0, cyc = 2 * (travel + dwell);
        const p = ((S.clock % cyc) + cyc) % cyc;
        m.t = p < dwell ? 0 : p < dwell + travel ? (p - dwell) / travel : p < 2 * dwell + travel ? 1 : 1 - (p - 2 * dwell - travel) / travel;
      }
      g.ox = m.from[0] + (m.to[0] - m.from[0]) * m.t;
      g.oy = m.from[1] + (m.to[1] - m.from[1]) * m.t;
      g.vx = dt > 0 ? (m.to[0] - m.from[0]) * (m.t - prevT) / dt : 0;
      g.vy = dt > 0 ? (m.to[1] - m.from[1]) * (m.t - prevT) / dt : 0;
      transformGround(g);
    }

    function stepFlaps(dt) {
      for (const f of L.flaps) {
        const g = L.grounds[f.gi];
        if (f.state === 'idle' && S.mode === 'ground' && S.gi === f.gi) { f.state = 'arming'; f.timer = 0; S.events.push('flap'); }
        if (f.state === 'arming') { f.timer += dt; if (f.timer >= f.delay) f.state = 'falling'; }
        if (f.state === 'falling') {
          // fold the free end downwards
          const target = f.dir * g.side * PI / 2;
          g.rot += Math.sign(target) * 7 * dt;
          if (Math.abs(g.rot) >= PI / 2) { g.rot = target; f.state = 'down'; }
          transformGround(g);
          if (S.mode === 'ground' && S.gi === f.gi && S.status === 'run') { syncFromGround(); takeoff(S.vx - g.vx, S.vy - g.vy); }
        }
      }
    }

    function step(dt, inp) {
      S.events = S.events || [];
      for (const gt of L.gates) if (gt.opening) gt.open = Math.min(gt.bottom - gt.top + 10, gt.open + 600 * dt);
      if (S.status !== 'run') return;
      S.time += dt; S.clock += dt;
      for (const m of L.movers) applyMover(m, dt);
      for (const w of L.seesaws) { // the plank tips towards the side the bike is on
        const g = L.grounds[w.gi];
        const on = S.mode === 'ground' && S.gi === w.gi;
        w.omega += (on ? (S.x - g.pivot[0]) * 0.012 : 0) * dt;
        w.omega *= Math.exp(-2.5 * dt);
        w.rot += w.omega * dt;
        if (w.rot > w.maxA) { w.rot = w.maxA; w.omega = 0; } else if (w.rot < -w.maxA) { w.rot = -w.maxA; w.omega = 0; }
        const vy0 = g.rot; g.rot = w.rot; transformGround(g); g.vx = 0; g.vy = 0;
      }
      for (const g of L.grounds) if (g.convWave) { const sg = Math.sign(Math.sin(g.convWave.phase + g.convWave.speed * S.clock)) || 1; g.base.forEach(q => { if (q[2] === 'conv') q[3] = sg; }); }
      stepFlaps(dt);
      for (const d of L.doors) if (d.opening) d.open = Math.min(d.bottom - d.top, d.open + 320 * dt);
      for (const d of L.dropSpikes) {
        if (d.state === 'hang' && S.x > d.x - d.trigger && S.x < d.x) d.state = 'fall';
        if (d.state === 'fall') { d.vy += 1600 * dt; d.y += d.vy * dt; if (d.y >= d.ground) { d.state = 'gone'; S.events.push('shatter'); } }
      }
      if (S.mode === 'sling') { stepSling(dt, inp); return; }
      if (S.mode === 'aim') { stepAim(dt, inp); return; }
      if (S.mode === 'vine') { const pv = bikePoints(); stepVine(dt, inp); if (S.status === 'run' && S.mode === 'vine') hazards(pv); return; }
      // jump button (only inside dragon caves)
      if (!inp.jump) S.jumpHeld = false;
      if (inp.jump && !S.jumpHeld && S.mode === 'ground' && L.jumpZones.some(z => S.x > z.x0 && S.x < z.x1 && S.y > z.y0 && S.y < z.y1)) {
        S.jumpHeld = true; syncFromGround(); const sg = L.grounds[S.gi].segs[S.si];
        takeoff(S.vx - L.grounds[S.gi].vx + sg.nx * 520, S.vy - L.grounds[S.gi].vy + sg.ny * 520); S.events.push('jump');
      }
      const prev = bikePoints();
      const pre = { mode: S.mode, gi: S.gi, si: S.si, u: S.u, x: S.x, y: S.y };
      if (S.mode === 'ground') { syncFromGround(); stepGround(dt, inp); } else stepAir(dt, inp);
      // closed doors block the way (you bump, you don't crash)
      for (const d of L.doors) {
        if (d.open >= d.bottom - d.top - 75) continue;
        const P = bikePoints(), bottom = d.bottom - d.open;
        for (const k of ['front', 'head', 'pos']) {
          if (prev[k][0] <= d.x && P[k][0] > d.x && P[k][1] > d.top - 5 && P[k][1] < bottom) {
            if (pre.mode === 'ground' && S.mode === 'ground') { S.gi = pre.gi; S.si = pre.si; S.u = pre.u; S.v = 0; syncFromGround(); }
            else { S.x = pre.x; S.y = pre.y; S.vx = -Math.abs(S.vx) * 0.2; }
            if (!S.bumpCd || S.clock - S.bumpCd > 0.4) { S.events.push('bump'); S.bumpCd = S.clock; }
            break;
          }
        }
      }
      if (S.status === 'run') hazards(prev);
    }

    reset(true);
    return { S, step, reset, bikePoints, coins, L, sawPos, crusherRect, timedShapes, sharkPos, fireballs, vineIdle, birdPos, pendulumPos, piranhaPos, hailPos, placeOnGround, consts: { HEAD, WHEEL_R, WHEEL_X } };
  }

  const api = { createGame, levelCount: LEVELS.length, Builder, genLevel, SEEDS, SLING, AIM };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  else root.MotoTrial = api;
})(this);
