import {AxesHelper} from 'three';

// Parenting each frame to its URDF joint preserves its full kinematic transform.
export function addRobotCoordinates(robot, visible=false) {
 return Object.values(robot.joints).map(joint=>{
  const axes=new AxesHelper(.2);
  axes.name='coordinates:'+joint.name;
  axes.visible=visible;
  axes.raycast=()=>{}; // Helpers must never intercept link selection.
  axes.material.depthTest=false;
  axes.renderOrder=5;
  joint.add(axes);
  return axes;
 });
}
