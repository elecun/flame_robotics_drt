import assert from 'node:assert/strict';
import {Ray,Vector3} from 'three';
import {beginJointDrag,updateJointDrag} from '../core/joint-drag.mjs';
const v=(x,y,z)=>new Vector3(x,y,z);
const ray=(point,eye=v(0,0,10))=>new Ray(eye,point.clone().sub(eye).normalize());
const close=(a,b)=>assert.ok(Math.abs(a-b)<1e-8, a+' != '+b);
for(const eye of [v(0,0,10),v(4,3,20)]){
 const d=beginJointDrag(ray(v(1,0,0),eye),v(0,0,0),v(0,0,1),'revolute',.3);
 close(updateJointDrag(d,ray(v(0,1,0),eye)),.3+Math.PI/2);
 close(updateJointDrag(d,ray(v(-1,0,0),eye)),.3+Math.PI);
 close(updateJointDrag(d,ray(v(0,-1,0),eye)),.3+Math.PI*1.5);
 const p=beginJointDrag(ray(v(0,0,0),eye),v(0,0,0),v(1,0,0),'prismatic',2);
 close(updateJointDrag(p,ray(v(3,0,0),eye)),5);
}
assert.equal(beginJointDrag(new Ray(v(0,0,10),v(0,0,-1)),v(0,0,0),v(0,0,1),'prismatic',0),null);
console.log('PASS: world-space rotation, translation, zoom/view independence, angle unwrapping and degenerate view');
