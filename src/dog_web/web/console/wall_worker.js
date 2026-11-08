import {classifyWallPoints} from './wall_geometry.js';
self.onmessage=event=>{
  const flags=classifyWallPoints(event.data);
  self.postMessage(flags,[flags.buffer]);
};
