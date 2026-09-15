// Runs the function node straight out of cmt_flow.json - no copy to drift.
//   node node-red/test_cmt_map.js
const assert = require('assert');
const flow = require('./cmt_flow.json');
const src = flow.find(n => n.id === 'cmt_map').func;
const run = new Function('msg', 'flow', src);

const ctx = new Map();
const store = { get: k => ctx.get(k), set: (k, v) => ctx.set(k, v) };
const send = (topic, payload) => run({ topic, payload }, store) || [[]];
const by = (r, t) => (r[0].find(m => m.topic.endsWith(t)) || {}).payload;

// status: birth/will -> boolean
assert.deepStrictEqual(send('vehicle/canlog-3C71BF/status', { state: 'offline' })[0][0],
  { topic: 'cmt/CMT-01/status', retain: true, payload: { online: false } });
assert.strictEqual(send('vehicle/canlog-3C71BF/status', { state: 'online' })[0][0].payload.online, true);

// first data batch
let r = send('vehicle/canlog-3C71BF/data', {
  sig: { speed: 42, distance_total: 48213.203, mcu_temp: 38, batt_voltage: 46.8, soc: 57, power: 3.76 },
  energy: { km: 12.4, kW: 3.76, recent: 220 }
});
const t = by(r, '/telemetry'), b = by(r, '/battery');
assert.strictEqual(t.speedKph, 42);
assert.strictEqual(t.odometerKm, 48213.203);
assert.strictEqual(t.tripKm, 12.4);
assert.strictEqual(t.motorTempC, 38);
assert.ok(!('lat' in t), 'no GPS on this board, so no lat');
assert.ok(/^\d{4}-\d{2}-\d{2}T.*Z$/.test(t.ts));
assert.strictEqual(b.packVoltage, 46.8);
assert.strictEqual(b.totalPct, 57);
assert.strictEqual(b.packCurrentA, 80.3);            // 3.76 kW / 46.8 V
assert.strictEqual(b.rangeKm, 13);                   // 5 kWh * 57% / 220 Wh/km
assert.strictEqual(b.cells.length, 1);
assert.ok(!by(r, '/alert'), 'nothing wrong yet');

// sparse batch: firmware sends only what changed, cache must fill the rest
r = send('vehicle/canlog-3C71BF/data', { sig: { speed: 0 }, energy: { km: 12.4 } });
assert.strictEqual(by(r, '/telemetry').speedKph, 0);
assert.strictEqual(by(r, '/telemetry').odometerKm, 48213.203, 'held from the previous batch');
assert.strictEqual(by(r, '/battery').packVoltage, 46.8);

// alert fires once on the edge, not every batch
r = send('vehicle/canlog-3C71BF/data', { sig: { batt_temp_1: 52.3 } });
assert.deepStrictEqual(
  (({ type, severity, message }) => ({ type, severity, message }))(by(r, '/alert')),
  { type: 'over_temp', severity: 'warning', message: 'Pack at 52.3C' });
assert.ok(!by(send('vehicle/canlog-3C71BF/data', { sig: {} }), '/alert'), 'no repeat while unchanged');

// escalation is a new edge, so it re-fires; fault outranks temperature
assert.strictEqual(by(send('vehicle/canlog-3C71BF/data', { sig: { batt_temp_1: 61 } }), '/alert').severity, 'critical');
assert.strictEqual(by(send('vehicle/canlog-3C71BF/data', { sig: { fault: 1 } }), '/alert').type, 'fault');

// bad input must not produce NaN fields
r = send('vehicle/canlog-3C71BF/data', { sig: { batt_voltage: 0, speed: null } });
assert.ok(!('speedKph' in by(r, '/telemetry')));
assert.ok(!('packCurrentA' in by(r, '/battery')), 'no divide by zero volts');

// cmd topic is not ours
assert.strictEqual(run({ topic: 'vehicle/canlog-3C71BF/cmd', payload: {} }, store), null);

console.log('ok');
