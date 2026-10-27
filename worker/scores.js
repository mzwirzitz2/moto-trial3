// Shared leaderboard on /api/scores, stored in Cloudflare KV (binding "KV", see wrangler.jsonc).
//
// GET  /api/scores?level=3&me=<clientId>  → top 10 of level 3 (+ your own rank)
// POST /api/scores {level, id, name, score, total, time, deaths}  → save, returns your rank
//
// One key per level ("lb:<level>") holds the best entry per player, sorted.

const LEVELS = 70;   // levels 0..69
const KEEP = 50;     // entries stored per level
const SHOW = 10;     // entries returned per level

const json = (data, status = 200) => new Response(JSON.stringify(data), {
  status, headers: { 'content-type': 'application/json; charset=utf-8', 'cache-control': 'no-store' },
});
const cleanName = n => String(n || '').replace(/[\u0000-\u001f\u007f<>"&]/g, '').replace(/\s+/g, ' ').trim().slice(0, 16) || 'Spieler';
const better = (a, b) => a.score > b.score || (a.score === b.score && a.time < b.time);
const levelOf = v => { const n = Number(v); return Number.isInteger(n) && n >= 0 && n < LEVELS ? n : null; };

async function load(env, level) {
  return (await env.KV.get('lb:' + level, 'json')) || [];
}

export async function onRequestGet({ request, env }) {
  if (!env.KV) return json({ error: 'KV binding "KV" fehlt' }, 503);
  const url = new URL(request.url);
  const level = levelOf(url.searchParams.get('level'));
  if (level === null) return json({ error: 'level' }, 400);
  const me = url.searchParams.get('me') || '';
  const list = await load(env, level);
  const rank = me ? list.findIndex(e => e.id === me) + 1 : 0;
  return json({
    level,
    players: list.length,
    scores: list.slice(0, SHOW).map(e => ({ name: e.name, score: e.score, time: e.time, deaths: e.deaths, date: e.date, me: !!me && e.id === me })),
    mine: rank > SHOW ? { rank, ...pick(list[rank - 1]) } : null,
  });
}
const pick = e => ({ name: e.name, score: e.score, time: e.time, deaths: e.deaths, date: e.date });

export async function onRequestPost({ request, env }) {
  if (!env.KV) return json({ error: 'KV binding "KV" fehlt' }, 503);
  let b;
  try { b = await request.json(); } catch (e) { return json({ error: 'json' }, 400); }
  const level = levelOf(b.level);
  const id = String(b.id || '');
  const score = Number(b.score), total = Number(b.total), time = Number(b.time), deaths = Number(b.deaths);
  // basic plausibility checks (a determined cheater can still fake results)
  if (level === null || !/^[a-z0-9-]{8,40}$/i.test(id)) return json({ error: 'bad request' }, 400);
  if (!Number.isInteger(score) || !Number.isInteger(total) || score < 0 || total < 1 || total > 3000 || score > total) return json({ error: 'score' }, 400);
  if (!(time >= 3 && time < 3600) || !Number.isInteger(deaths) || deaths < 0 || deaths > 9999) return json({ error: 'time' }, 400);

  const entry = { id, name: cleanName(b.name), score, time: Math.round(time * 100) / 100, deaths, date: new Date().toISOString().slice(0, 10) };
  const list = await load(env, level);
  const i = list.findIndex(e => e.id === id);
  if (i < 0) list.push(entry);
  else if (better(entry, list[i])) list[i] = entry;
  else list[i].name = entry.name; // not better, but keep the newest name
  list.sort((a, c) => c.score - a.score || a.time - c.time);
  const kept = list.slice(0, KEEP);
  await env.KV.put('lb:' + level, JSON.stringify(kept));
  const rank = kept.findIndex(e => e.id === id) + 1;
  return json({ ok: true, rank: rank || null, players: kept.length });
}
