const statusEl = document.getElementById('status');
let busy = false;

function setStatus(text, type = '') {
  statusEl.textContent = text;
  statusEl.className = `status ${type}`.trim();
}

async function runCommand(station, action) {
  if (busy) return;
  busy = true;
  document.querySelectorAll('.btn').forEach((b) => { b.disabled = true; });

  const label = action === 'sajikan' ? 'Sajikan' : 'Penempatan';
  setStatus(`${label} stasiun ${station.toUpperCase()}…`, 'busy');

  try {
    const res = await fetch(`/api/${action}/${station}`, { method: 'POST' });
    const data = await res.json();
    if (!res.ok || !data.ok) {
      throw new Error(data.error || 'Perintah gagal');
    }
    setStatus(`Perintah ${label} ${data.station} dikirim ke robot.`, 'ok');
  } catch (err) {
    setStatus(err.message, 'err');
  } finally {
    busy = false;
    document.querySelectorAll('.btn').forEach((b) => { b.disabled = false; });
  }
}

document.querySelectorAll('.station').forEach((section) => {
  const station = section.dataset.station;
  section.querySelectorAll('[data-action]').forEach((btn) => {
    btn.addEventListener('click', () => runCommand(station, btn.dataset.action));
  });
});
