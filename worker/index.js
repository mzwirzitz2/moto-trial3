// Worker entry: /api/scores is the leaderboard, everything else is the game (static assets).
import { onRequestGet, onRequestPost } from './scores.js';

export default {
  async fetch(request, env, ctx) {
    const url = new URL(request.url);
    if (url.pathname === '/api/scores') {
      if (request.method === 'GET') return onRequestGet({ request, env, ctx });
      if (request.method === 'POST') return onRequestPost({ request, env, ctx });
      return new Response('Method not allowed', { status: 405 });
    }
    return env.ASSETS.fetch(request);
  },
};
