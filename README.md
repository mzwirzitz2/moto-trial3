# Moto Trial

Motorrad-Geschicklichkeitsspiel im Browser (50 Level), gehostet auf Cloudflare Pages, installierbar als PWA.

- `index.html` – Seite, Grafik, Menü, Steuerung
- `game.js` – Physik, Level und Level-Generator
- `version.js` – App-Version (bei jedem Deploy `APP_VERSION` hochzählen)
- `sw.js`, `manifest.webmanifest`, `icons/` – PWA / Offline
- `worker/` – Worker-Skript: `/api/scores` (geteilte Online-Bestenliste in KV), alles andere sind die statischen Spieldateien
- `wrangler.jsonc` – Worker-Konfiguration (Name `moto-trial3`, Assets, KV-Binding)
- `.assetsignore` – Dateien, die nicht ausgeliefert werden

## Online-Bestenliste

Die Bestenliste läuft im Worker unter `/api/scores` und speichert in Cloudflare KV.
Das KV-Binding (`KV` → Namespace `f8088560d28a4da79596f62c43458984`) steht in `wrangler.jsonc`
und wird beim Deploy automatisch gesetzt – im Dashboard muss dafür nichts eingestellt werden.

Test nach dem Deploy: `https://<deine-seite>/api/scores?level=0` liefert `{"level":0,"players":0,"scores":[],...}`.

Gespeichert wird pro Level ein Schlüssel `lb:<level>` mit dem besten Ergebnis je Spieler (anonyme Geräte-ID + Name).

## Lokal entwickeln

Mit Bestenliste (lokales, simuliertes KV):

```bash
npx wrangler dev
```

Nur das Spiel:

```bash
python3 -m http.server 8765
```
