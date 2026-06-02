require('dotenv').config();

const express = require('express');
const net = require('net');
const path = require('path');
const WEB_PORT = Number(process.env.WEB_PORT) || 3000;
const ROBOT_HOST = process.env.ROBOT_HOST || '127.0.0.1';
const ROBOT_BRIDGE_PORT = Number(process.env.ROBOT_BRIDGE_PORT || process.env.ROBOT_PORT) || 8765;
const ROBOT_API_KEY = process.env.ROBOT_API_KEY || '';
const WEB_API_KEY = process.env.WEB_API_KEY || '';
const ALLOW_LOCAL_WITHOUT_KEY = process.env.ALLOW_LOCAL_WITHOUT_KEY !== 'false';

const app = express();
app.use(express.json());
app.use(express.static(path.join(__dirname, 'public')));

function isLocalRequest(req) {
  const ip = req.ip || req.socket.remoteAddress || '';
  return ip === '127.0.0.1'
    || ip === '::1'
    || ip === '::ffff:127.0.0.1';
}

function requireWebApiKey(req, res, next) {
  if (!WEB_API_KEY) {
    return next();
  }
  if (ALLOW_LOCAL_WITHOUT_KEY && isLocalRequest(req)) {
    return next();
  }
  const provided = req.get('x-api-key');
  if (provided && provided === WEB_API_KEY) {
    return next();
  }
  return res.status(401).json({ ok: false, error: 'Unauthorized: X-API-Key tidak valid' });
}

function sendRobotCommand(command) {
  return new Promise((resolve, reject) => {
    const payload = ROBOT_API_KEY ? `${ROBOT_API_KEY}|${command}` : command;

    const client = net.createConnection({ host: ROBOT_HOST, port: ROBOT_BRIDGE_PORT }, () => {
      client.write(`${payload}\n`);
    });

    let response = '';
    client.setEncoding('utf8');

    client.on('data', (chunk) => {
      response += chunk;
    });

    client.on('end', () => {
      const trimmed = response.trim();
      if (trimmed === 'UNAUTHORIZED') {
        reject(new Error('Robot menolak token. Samakan ROBOT_API_KEY dengan api_token di robot.config.ini'));
        return;
      }
      resolve(trimmed || 'OK');
    });

    client.on('error', reject);

    setTimeout(() => {
      client.destroy();
      reject(new Error('Timeout: program robot tidak merespons. Pastikan auboi5-sdk sedang berjalan.'));
    }, 5000);
  });
}

const SAJIKAN_CMD = { a: 'SAJIKAN_A', b: 'SAJIKAN_B', c: 'SAJIKAN_C' };
const PESAN_CMD = { a: 'PESAN_A', b: 'PESAN_B', c: 'PESAN_C' };

function normalizeStation(station) {
  const key = String(station || '').toLowerCase();
  if (!SAJIKAN_CMD[key]) {
    throw new Error('Stasiun harus A, B, atau C');
  }
  return key;
}

app.post('/api/sajikan/:station', requireWebApiKey, async (req, res) => {
  try {
    const key = normalizeStation(req.params.station);
    const reply = await sendRobotCommand(SAJIKAN_CMD[key]);
    res.json({ ok: true, station: key.toUpperCase(), mode: 'sajikan', robot: reply });
  } catch (err) {
    res.status(500).json({ ok: false, error: err.message });
  }
});

app.post('/api/penempatan/:station', requireWebApiKey, async (req, res) => {
  try {
    const key = normalizeStation(req.params.station);
    const reply = await sendRobotCommand(PESAN_CMD[key]);
    res.json({ ok: true, station: key.toUpperCase(), mode: 'penempatan', robot: reply });
  } catch (err) {
    res.status(500).json({ ok: false, error: err.message });
  }
});

app.get('/api/health', (_req, res) => {
  res.json({
    ok: true,
    webApiKeyRequired: Boolean(WEB_API_KEY),
    robotApiKeyConfigured: Boolean(ROBOT_API_KEY),
    allowLocalWithoutKey: ALLOW_LOCAL_WITHOUT_KEY,
  });
});

app.listen(WEB_PORT, () => {
  console.log(`Website kontrol robot: http://localhost:${WEB_PORT}`);
  console.log(`Bridge ke program C++: ${ROBOT_HOST}:${ROBOT_BRIDGE_PORT}`);
  if (!ROBOT_API_KEY) {
    console.warn('PERINGATAN: ROBOT_API_KEY kosong — set di .env untuk produksi.');
  }
  if (WEB_API_KEY) {
    console.log('WEB_API_KEY aktif (localhost boleh tanpa header jika ALLOW_LOCAL_WITHOUT_KEY=true)');
  }
});
