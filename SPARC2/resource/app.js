import * as T from 'three';
import {OrbitControls} from './vendor/OrbitControls.js';
import {PCDLoader} from './vendor/PCDLoader.js';
import {PLYLoader} from './vendor/PLYLoader.js';
const $=id=>document.getElementById(id), status=$('status');
const scene=new T.Scene();scene.background=new T.Color('#161b22');
const camera=new T.PerspectiveCamera(55,1,.01,1e8);camera.position.set(4,3,5);
const renderer=new T.WebGLRenderer({antialias:true});renderer.setPixelRatio(Math.min(devicePixelRatio,2));$('view').append(renderer.domElement);
const controls=new OrbitControls(camera,renderer.domElement);
scene.add(new T.HemisphereLight(0xffffff,0x555566,3));
const grid=new T.GridHelper(10,20,0x607080,0x303c48);scene.add(grid);
let current;
function fit(){if(!current)return;const box=new T.Box3().setFromObject(current);if(box.isEmpty())return;const center=box.getCenter(new T.Vector3()),size=box.getSize(new T.Vector3()).length()||1;controls.target.copy(center);camera.position.copy(center).add(new T.Vector3(1,.7,1).normalize().multiplyScalar(size*1.4));camera.near=size/10000;camera.far=size*100;camera.updateProjectionMatrix();controls.update();}
function clear(){if(current){scene.remove(current);current.traverse(o=>{o.geometry?.dispose();o.material?.dispose();});current=null;}}
function urdf(text){
 const doc=new DOMParser().parseFromString(text,'application/xml');if(doc.querySelector('parsererror')||!doc.querySelector('robot'))throw Error('Invalid URDF');
 const root=new T.Group(),links=new Map();
 for(const link of doc.querySelectorAll('robot > link')){
  const group=new T.Group();links.set(link.getAttribute('name'),group);
  for(const visual of link.querySelectorAll('visual')){
   const g=visual.querySelector('geometry');let geo;
   if(g?.querySelector('mesh'))throw Error('Prototype URDF supports box, sphere and cylinder; mesh assets are not supported yet.');
   const b=g?.querySelector('box'),s=g?.querySelector('sphere'),c=g?.querySelector('cylinder');
   if(b)geo=new T.BoxGeometry(...b.getAttribute('size').split(/\s+/).map(Number));
   if(s)geo=new T.SphereGeometry(Number(s.getAttribute('radius')));
   if(c){geo=new T.CylinderGeometry(+c.getAttribute('radius'),+c.getAttribute('radius'),+c.getAttribute('length'));geo.rotateX(Math.PI/2);}
   if(geo){const mesh=new T.Mesh(geo,new T.MeshStandardMaterial({color:0x65a9d8}));origin(mesh,visual.querySelector('origin'));group.add(mesh);}
  }root.add(group);
 }
 for(const joint of doc.querySelectorAll('joint')){const parent=links.get(joint.querySelector('parent')?.getAttribute('link')),child=links.get(joint.querySelector('child')?.getAttribute('link'));if(parent&&child){origin(child,joint.querySelector('origin'));parent.add(child);}}
 root.rotation.x=-Math.PI/2;return root;
}
function origin(obj,el){if(!el)return;obj.position.fromArray((el.getAttribute('xyz')||'0 0 0').trim().split(/\s+/).map(Number));const r=(el.getAttribute('rpy')||'0 0 0').trim().split(/\s+/).map(Number);obj.rotation.set(...r,'ZYX');}
async function load(url,name=url){status.textContent='Loading '+name;try{
 const response=await fetch(url);if(!response.ok)throw Error('HTTP '+response.status);const data=await response.arrayBuffer();let obj;
 if(name.toLowerCase().endsWith('.pcd'))obj=new PCDLoader().parse(data);
 else if(name.toLowerCase().endsWith('.ply')){const geo=new PLYLoader().parse(data);obj=new T.Points(geo,new T.PointsMaterial({size:.01,vertexColors:geo.hasAttribute('color'),color:0xffffff}));}
 else obj=urdf(new TextDecoder().decode(data));
 clear();current=obj;scene.add(obj);fit();if(obj.isPoints){obj.material.size=new T.Box3().setFromObject(obj).getSize(new T.Vector3()).length()/1500;}
 status.textContent='Loaded\n'+name+'\n'+(obj.geometry?.attributes.position.count?.toLocaleString()||'Robot')+' points / elements';
 }catch(e){status.textContent='Error: '+e.message;}}
$('fit').onclick=fit;$('clear').onclick=()=>{clear();status.textContent='Scene cleared';};
$('load').onclick=()=>load($('files').value);$('open').onclick=()=>$('local').click();
$('local').onchange=async()=>{const file=$('local').files[0];if(file){const url=URL.createObjectURL(file);await load(url,file.name);URL.revokeObjectURL(url);}};
new ResizeObserver(()=>{const {width,height}=$('view').getBoundingClientRect();renderer.setSize(width,height);camera.aspect=width/height;camera.updateProjectionMatrix();}).observe($('view'));
renderer.setAnimationLoop(()=>{controls.update();renderer.render(scene,camera);});
try{const files=await(await fetch('/api/files')).json();for(const f of files){const option=document.createElement('option');option.value=f;option.textContent=f.split('/').pop();$('files').append(option);}if(files.length)await load(files[0]);else status.textContent='Ready — open a local file';}catch(e){status.textContent=e.message;}
