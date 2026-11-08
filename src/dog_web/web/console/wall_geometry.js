// Display-only local plane classification. Vertical furniture can also match;
// these flags must never be used as obstacle or traversability evidence.
function verticalPlane(s) {
  const n=s[0];
  if(n<8 || s[11]-s[10]<.3)return false;
  const mean=s.slice(1,4).map(v=>v/n);
  const a=[
    [s[4]/n-mean[0]**2,s[5]/n-mean[0]*mean[1],s[6]/n-mean[0]*mean[2]],
    [s[5]/n-mean[0]*mean[1],s[7]/n-mean[1]**2,s[8]/n-mean[1]*mean[2]],
    [s[6]/n-mean[0]*mean[2],s[8]/n-mean[1]*mean[2],s[9]/n-mean[2]**2]];
  const v=[[1,0,0],[0,1,0],[0,0,1]];
  // Jacobi diagonalization of a symmetric covariance; columns of v are normals.
  for(let step=0;step<16;step++) {
    let p=0,q=1;
    for(const [i,j] of [[0,2],[1,2]])if(Math.abs(a[i][j])>Math.abs(a[p][q])){p=i;q=j;}
    if(Math.abs(a[p][q])<1e-10)break;
    const angle=.5*Math.atan2(2*a[p][q],a[q][q]-a[p][p]);
    const c=Math.cos(angle),t=Math.sin(angle),pp=a[p][p],qq=a[q][q],pq=a[p][q];
    a[p][p]=c*c*pp-2*c*t*pq+t*t*qq;
    a[q][q]=t*t*pp+2*c*t*pq+c*c*qq;a[p][q]=a[q][p]=0;
    for(let k=0;k<3;k++) {
      if(k!==p&&k!==q){const kp=a[k][p],kq=a[k][q];a[k][p]=a[p][k]=c*kp-t*kq;a[k][q]=a[q][k]=t*kp+c*kq;}
      const vp=v[k][p],vq=v[k][q];v[k][p]=c*vp-t*vq;v[k][q]=t*vp+c*vq;
    }
  }
  const order=[0,1,2].sort((i,j)=>a[i][i]-a[j][j]);
  const [small,middle,large]=order.map(i=>Math.max(0,a[i][i]));
  return middle>.0009 && middle>.08*large && small<.1*middle && Math.abs(v[2][order[0]])<.4;
}

export function classifyWallPoints(positions) {
  const count=positions.length/3,flags=new Uint8Array(count),cells=new Map(),keys=new Array(count);
  if(!count)return flags;
  const origin=positions.slice(0,3),key=(x,y,z)=>`${x},${y},${z}`,size=.25;
  for(let i=0;i<count;i++) {
    const x=positions[i*3]-origin[0],y=positions[i*3+1]-origin[1],z=positions[i*3+2]-origin[2];
    if(![x,y,z].every(Number.isFinite))continue;
    const cell=[Math.floor(x/size),Math.floor(y/size),Math.floor(z/size)],id=key(...cell);keys[i]=id;
    let bucket=cells.get(id);
    if(!bucket){bucket={cell,s:new Float64Array(12)};bucket.s[10]=Infinity;bucket.s[11]=-Infinity;cells.set(id,bucket);}
    const s=bucket.s;s[0]++;s[1]+=x;s[2]+=y;s[3]+=z;s[4]+=x*x;s[5]+=x*y;s[6]+=x*z;
    s[7]+=y*y;s[8]+=y*z;s[9]+=z*z;s[10]=Math.min(s[10],z);s[11]=Math.max(s[11],z);
  }
  for(const bucket of cells.values()) {
    const sum=new Float64Array(12);sum[10]=Infinity;sum[11]=-Infinity;
    const [x,y,z]=bucket.cell;
    for(let dx=-1;dx<=1;dx++)for(let dy=-1;dy<=1;dy++)for(let dz=-1;dz<=1;dz++) {
      const other=cells.get(key(x+dx,y+dy,z+dz));if(!other)continue;
      for(let j=0;j<10;j++)sum[j]+=other.s[j];
      sum[10]=Math.min(sum[10],other.s[10]);sum[11]=Math.max(sum[11],other.s[11]);
    }
    bucket.wall=verticalPlane(sum);
  }
  for(let i=0;i<count;i++)flags[i]=cells.get(keys[i])?.wall?1:0;
  return flags;
}
