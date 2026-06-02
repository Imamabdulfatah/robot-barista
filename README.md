# Robot Kopi — Aubo i5 + Arduino + Web Control

Aplikasi Windows untuk mengendalikan robot lengan **Aubo i5** pada sistem barista kopi otomatis. Robot mengambil gelas, menyeduh kopi, dan menempatkan minuman ke **stasiun A, B, atau C**. Perintah dapat dikirim dari **Arduino (serial)**, **website Node.js**, atau keduanya secara bersamaan.

---

## Fitur

- Kontrol robot Aubo i5 via SDK resmi (login, startup, pergerakan joint)
- Gripper/tangan robot via serial (`Com_Hands_AngleSet`)
- Trigger mesin kopi & sinyal suara lewat I/O digital robot
- **3 stasiun penyajian** (A, B, C) dengan pose joint tersimpan
- **Website lokal** (`web-control/`) — tombol *Sajikan* (alur lengkap) dan *Penempatan* (hanya ke meja)
- **Arduino** — perintah teks `EKSEKUSI_1` … `EKSEKUSI_3` lewat COM serial
- **TCP bridge** — Node.js ↔ program C++ di `localhost:8765`

---

## Arsitektur

```
┌─────────────┐     HTTP :3000      ┌──────────────────┐
│   Browser   │ ──────────────────► │  web-control/    │
└─────────────┘                     │  (Node.js)       │
                                      └────────┬─────────┘
                                               │ TCP :8765
┌─────────────┐     Serial COM6                ▼
│   Arduino   │ ──────────────────► ┌──────────────────┐
└─────────────┘                     │ auboi5-sdk.exe   │
                                      │ (C++ / VS)       │
                                      └────────┬─────────┘
                                               │
                    ┌──────────────────────────┼──────────────────────────┐
                    ▼                          ▼                          ▼
              Aubo i5 :8899              Gripper (COM)                I/O mesin
              (IP di config)               115200 baud                  & suara
```

---

## Persyaratan

| Komponen | Versi / catatan |
|----------|------------------|
| OS | Windows 10/11 (x64) |
| IDE | Visual Studio 2017+ (toolset v141, platform **x64**) |
| Node.js | 18+ (untuk website) |
| Hardware | Aubo i5, gripper serial, Arduino (opsional), mesin kopi |
| Library eksternal | [serial](https://github.com/wjwwood/serial) — path di `.vcxproj` perlu disesuaikan |

> **Catatan:** File `libserviceinterface.lib` dan header SDK ada di folder `dependens/auboi5/`. Lisensi SDK mengikuti kebijakan **Aubo Robotics**.

---

## Struktur proyek

```
auboi5-sdk-for-windows-x64/
├── auboi5-sdk-for-windows-x64.cpp   # Logika utama robot & bridge TCP
├── example.cpp / example.h          # Wrapper SDK Aubo & komunikasi gripper
├── dependens/auboi5/                # Header & library SDK Aubo
├── web-control/                     # Website kontrol (Express)
│   ├── server.js
│   ├── package.json
│   └── public/                      # UI tombol A / B / C
├── auboi5-sdk-for-windows-x64.vcxproj
└── README.md
```

---

## Konfigurasi & keamanan (wajib sebelum push GitHub)

**Jangan commit IP robot, token, atau file `.env`.** Repositori sudah menyertakan `.gitignore` dan file contoh tanpa rahasia.

### 1. Program robot (C++)

```bash
copy robot.config.example.ini robot.config.ini
```

Edit `robot.config.ini` (file ini **hanya di komputer Anda**):

| Kunci | Keterangan |
|-------|------------|
| `robot_addr` | IP controller Aubo |
| `robot_port` | Port SDK (biasanya `8899`) |
| `com_hand` | Nomor COM gripper |
| `arduino_port` | Port Arduino |
| `api_token` | Token acak untuk bridge TCP |

Generate token:

```bash
node -e "console.log(require('crypto').randomBytes(32).toString('hex'))"
```

Letakkan `robot.config.ini` di folder yang sama dengan `.exe` setelah build.

Override opsional lewat **Environment Variables** Windows: `ROBOT_ADDR`, `ROBOT_API_TOKEN`, `COM_HAND`, dll.

### 2. Website (Node.js)

```bash
cd web-control
copy .env.example .env
```

Edit `web-control/.env`:

| Variabel | Keterangan |
|----------|------------|
| `ROBOT_API_KEY` | **Sama** dengan `api_token` di `robot.config.ini` |
| `WEB_API_KEY` | Kunci terpisah untuk API HTTP (opsional di localhost) |
| `ALLOW_LOCAL_WITHOUT_KEY` | `true` = browser lokal tanpa header (dev) |

### Alur autentikasi

```
Browser → Node.js (WEB_API_KEY opsional) → TCP "TOKEN|SAJIKAN_A" → C++ (validasi api_token)
Arduino serial → C++ (tanpa token, akses fisik lokal)
```

---

## Build (Visual Studio)

1. Clone repositori ini.
2. Buat `robot.config.ini` dari contoh (lihat atas).
2. Buka `auboi5-sdk-for-windows-x64.vcxproj`.
3. Sesuaikan path **serial library** di Project Properties → C/C++ → Additional Include Directories (lihat `.vcxproj`).
4. Pilih konfigurasi **Debug | x64** (atau Release | x64).
5. Build Solution (`Ctrl+Shift+B`).
6. Jalankan `x64\Debug\auboi5-sdk-for-windows-x64.exe` (atau folder Release).

Di konsol harus muncul antara lain:

```
Tangan konek...
i5 Connecting...
TCP bridge aktif di port 8765 (website Node.js)
```

---

## Menjalankan website

Program robot **harus sudah berjalan** sebelum website di-start.

```bash
cd web-control
npm install
npm start
```

Buka: **http://localhost:3000**

| Tombol | Perintah TCP | Fungsi |
|--------|--------------|--------|
| Sajikan kopi A / B / C | `SAJIKAN_A` … `C` | Ambil gelas → seduh → taruh ke stasiun |
| Penempatan A / B / C | `PESAN_A` … `C` | Hanya bawa ke meja (gelas sudah siap) |

### API REST

```http
POST /api/sajikan/a
POST /api/sajikan/b
POST /api/sajikan/c

POST /api/penempatan/a
POST /api/penempatan/b
POST /api/penempatan/c

GET  /api/health
```

Contoh dengan API key (jika `WEB_API_KEY` di-set dan bukan localhost):

```powershell
$headers = @{ "X-API-Key" = "kunci-dari-env" }
Invoke-RestMethod -Method Post -Uri http://localhost:3000/api/sajikan/a -Headers $headers
```

---

## Perintah Arduino (serial)

Kirim string berikut ke COM Arduino (default `COM6`, 9600 baud):

| Perintah | Stasiun | Mode |
|----------|---------|------|
| `EKSEKUSI_1` | A | Sajikan lengkap |
| `EKSEKUSI_2` | B | Sajikan lengkap |
| `EKSEKUSI_3` | C | Sajikan lengkap |

Program C++ memetakan perintah yang sama dari TCP (`SAJIKAN_A`, dll.).

---

## Alur kerja singkat

1. **Inisialisasi** — gripper, login Aubo, startup robot, TCP bridge.
2. **Antrian perintah** — perintah dari web/Arduino masuk antrian; dieksekusi satu per satu.
3. **Sajikan lengkap** — `siapAmbilGelas` → ambil gelas → seduh → tunggu → taruh ke pose stasiun → `home`.
4. **Penempatan** — `persiapan()` → pose stasiun → lepas/genggam → `home`.

Pose joint didefinisikan di array `kRobotPoses[]` dalam `auboi5-sdk-for-windows-x64.cpp`.

---

## Troubleshooting

| Masalah | Solusi |
|---------|--------|
| `i5 not connect` | Cek kabel LAN, IP `ROBOT_ADDR`, firewall, controller Aubo menyala |
| `Hands_Com open error` | Cek nomor COM gripper (`COM_HAND`) di Device Manager |
| Website timeout | Pastikan `.exe` robot sudah jalan dan port 8765 tidak dipakai aplikasi lain |
| Arduino tidak terbaca | Cek `ARDUINO_PORT`; program tetap jalan tanpa Arduino (hanya TCP) |
| Gerakan tidak jalan setelah `pos("ambilGelas")` | Nama pose **case-sensitive** — gunakan `"AmbilGelas"` / `"SeduhKopi"` di kode |

---

## Push ke GitHub

File `.gitignore` sudah mengabaikan:

- `robot.config.ini`, `.env`, `web-control/.env`
- folder build (`x64/`, `Debug/`, `.vs/`, `node_modules/`)

**Sebelum push, cek tidak ada rahasia:**

```bash
git status
git grep -i "192.168" -- . ":!*.example.*"
git grep -i "api_token\|ROBOT_API" -- . ":!*.example.*" ":!.env.example"
```

```bash
git add .
git commit -m "Initial commit: robot kopi Aubo + web control"
git remote add origin https://github.com/<username>/<repo>.git
git push -u origin main
```

Jika pernah commit IP/token secara tidak sengaja, ganti token/IP baru dan pertimbangkan [GitHub secret scanning](https://docs.github.com/en/code-security/secret-scanning) / rewrite history.

---

## Lisensi & atribusi

- Kode aplikasi: sesuaikan lisensi repositori Anda (mis. MIT) jika diperlukan.
- **Aubo SDK** (`dependens/auboi5/`) — hak cipta Aubo Robotics; distribusikan sesuai perjanjian lisensi SDK.
- Library **serial** — lisensi MIT (William Woodall).

---

## Kontributor

Proyek integrasi robot kopi — SMK / barista otomatis (Cikalong).

Jika ada pertanyaan atau issue, buka **Issues** di repositori GitHub.
