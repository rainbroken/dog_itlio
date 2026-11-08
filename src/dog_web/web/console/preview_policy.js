// Map limits are slice sizes, never limits on the cached/displayed map total.
// Slow links send smaller slices and eventually complete every block.
export class PreviewPolicy {
  constructor(kind){this.kind=kind;this.scale=1;}
  options(driving){
    const map=this.kind==='map';
    return {limit:Math.max(map?32:256,Math.floor((map?(driving?256:512):(driving?1500:3000))*this.scale)),
      period:map?(driving?40:0):(driving?350:200),bytesPerSecond:map?(driving?24000:120000):(driving?16000:48000)};
  }
  completed(milliseconds){
    if(milliseconds>800)this.scale=Math.max(1/32,this.scale/2);
    else if(milliseconds<250)this.scale=Math.min(1,this.scale*1.1);
  }
  failed(){this.scale=Math.max(1/32,this.scale/2);}
  delay(options,bytes,elapsed){return Math.max(options.period,bytes*1000/options.bytesPerSecond-elapsed);}
}
