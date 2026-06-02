# Kontrol Robot Kopi via Website (Node.js)

## Setup aman

```bash
cp .env.example .env
npm install
```

Isi `.env` — **jangan commit file `.env`**.

| Variabel | Wajib | Keterangan |
|----------|-------|------------|
| `ROBOT_API_KEY` | Disarankan | Sama dengan `api_token` di `robot.config.ini` |
| `WEB_API_KEY` | Opsional | Lindungi API HTTP dari akses luar |
| `ALLOW_LOCAL_WITHOUT_KEY` | Default `true` | Akses dari `localhost` tanpa header |

## Menjalankan

1. Jalankan program C++ robot (dengan `robot.config.ini`).
2. `npm start`
3. Buka http://localhost:3000

Lihat README utama di folder induk untuk detail API dan keamanan.
