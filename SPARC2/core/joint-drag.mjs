import {Plane, Vector3} from 'three';

// A drag is constrained to the joint's world-space axis or rotation plane.
// Keep the initial grab offset so pointer-down never snaps the model.
export function beginJointDrag(ray, origin, axis, type, value) {
 const normal=axis.clone().normalize();
 if(type==='prismatic'){
  normal.copy(ray.direction).addScaledVector(axis,-ray.direction.dot(axis));
  if(normal.lengthSq()<1e-6)return null;
  normal.normalize();
 }
 const plane=new Plane().setFromNormalAndCoplanarPoint(normal,origin);
 const point=intersect(ray,plane);
 if(!point)return null;
 const vector=point.sub(origin);
 if(type!=='prismatic' && vector.lengthSq()<1e-10)return null;
 return {plane,origin:origin.clone(),axis:axis.clone(),type,value,
  initial:vector.dot(axis),previous:vector.normalize(),angle:0};
}
function intersect(ray,plane){
 if(Math.abs(ray.direction.dot(plane.normal))<1e-4)return null;
 return ray.intersectPlane(plane,new Vector3());
}
export function updateJointDrag(drag,ray){
 const point=intersect(ray,drag.plane);
 if(!point)return null;
 const vector=point.sub(drag.origin);
 if(drag.type==='prismatic')return drag.value+vector.dot(drag.axis)-drag.initial;
 if(vector.lengthSq()<1e-10)return null;
 vector.normalize();
 drag.angle+=Math.atan2(drag.axis.dot(new Vector3().crossVectors(drag.previous,vector)),drag.previous.dot(vector));
 drag.previous.copy(vector);
 return drag.value+drag.angle;
}
