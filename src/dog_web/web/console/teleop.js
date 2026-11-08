const axes = {w:[1,0,0], s:[-1,0,0], a:[0,1,0], d:[0,-1,0], q:[0,0,1], e:[0,0,-1]};
export function combinedDirection(inputs) {
  const keys = new Set(inputs.values());
  return Object.fromEntries(['x','y','yaw'].map((axis,i) =>
    [axis, Math.sign([...keys].reduce((sum,key) => sum + axes[key][i], 0))]));
}

// One HTTP request at a time. Input changes replace pending commands rather
// than queueing movement packets ahead of a release. Heartbeat is 20 Hz.
export class TeleopController {
  constructor({api, limits, onError, onChange = () => {}, onLease = () => {}, interval = 50}) {
    Object.assign(this, {api, limits, onError, onChange, onLease});
    this.inputs = new Map(); this.token = null; this.running = null; this.dirty = false;
    this.timer = setInterval(() => {if (this.inputs.size || this.token) this.pump();}, interval);
  }
  direction() {
    // Multiple sources for one key count once; opposite directions cancel.
    return combinedDirection(this.inputs);
  }
  setInputs(entries) {
    const next = new Map(entries.filter(([,key]) => axes[key]));
    if (JSON.stringify([...next]) === JSON.stringify([...this.inputs])) return;
    this.inputs = next; this.onChange(this.direction()); this.pump();
  }
  press(source, key) {
    if (!axes[key] || this.inputs.get(source) === key) return;
    this.inputs.set(source,key); this.onChange(this.direction()); this.pump();
  }
  lift(source) {
    if (!this.inputs.delete(source)) return;
    this.onChange(this.direction()); this.pump();
  }
  async stop() {
    this.inputs.clear(); this.onChange(this.direction());
    await this.pump();
  }
  pump() {
    this.dirty = true;
    if (!this.running) {
      this.running = Promise.resolve().then(() => this.drain()).finally(() => {
        this.running = null;
        if (this.dirty) this.pump();
      });
    }
    return this.running;
  }
  async dropLease() {
    const token = this.token; this.token = null;
    this.onLease(null);
    if (token) await this.api('/api/control/release', {token});
  }
  async drain() {
    try {
      while (this.dirty) {
        this.dirty = false;
        const direction = this.direction();
        if (!Object.values(direction).some(Boolean)) {await this.dropLease(); continue;}
        if (!this.token) {
          this.token = (await this.api('/api/control/acquire', {})).token;
          this.onLease(this.token);
          // A release or combination change while acquiring must win.
          if (this.dirty) continue;
        }
        const limits = this.limits();
        if (!limits) throw new Error('尚未读取速度配置');
        await this.api('/api/control/velocity', {token:this.token,
          ...Object.fromEntries(['x','y','yaw'].map(axis => [axis,direction[axis]*limits[axis]]))});
      }
    } catch (error) {
      this.inputs.clear(); this.dirty = false; this.onChange(this.direction());
      try {await this.dropLease();} catch { /* Independent backend deadman expires. */ }
      // Ignore input that arrived while cleaning up a failed request. A fresh
      // press is required after reporting the error, never automatic resumption.
      this.inputs.clear(); this.dirty = false; this.onChange(this.direction());
      this.onError(error);
    }
  }
  async dispose() {clearInterval(this.timer); await this.stop();}
}
