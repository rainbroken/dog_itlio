// Bundle local runtime dependencies without a CDN or a browser build step.
import {mkdir, copyFile} from 'node:fs/promises';
import {fileURLToPath} from 'node:url';
const root = new URL('../', import.meta.url);
await mkdir(new URL('vendor/', root), {recursive: true});
for (const [source, destination] of [
  ['build/three.module.js', 'three.module.js'],
  ['examples/jsm/controls/OrbitControls.js', 'OrbitControls.js'],
  ['LICENSE', 'THREE-LICENSE.txt'],
  ...['OBJLoader', 'MTLLoader', 'STLLoader', 'ColladaLoader', 'TGALoader'].map(name =>
    [`examples/jsm/loaders/${name}.js`, `loaders/${name}.js`]),
]) {
  await mkdir(new URL(`vendor/${destination.split('/').slice(0,-1).join('/')}/`, root), {recursive: true});
  await copyFile(fileURLToPath(new URL(`node_modules/three/${source}`, root)),
                 fileURLToPath(new URL(`vendor/${destination}`, root)));
}
for (const file of ['src/URDFLoader.js', 'src/URDFClasses.js', 'README.md']) {
  await copyFile(new URL(`node_modules/urdf-loader/${file}`, root),
    new URL(`vendor/${file.split('/').at(-1) === 'README.md' ? 'URDF-NOTICE.md' : file.split('/').at(-1)}`, root));
}
console.log('Three.js assets prepared locally.');
