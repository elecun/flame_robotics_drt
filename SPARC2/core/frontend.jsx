import React,{useEffect,useRef,useState} from 'react';
import {createRoot} from 'react-dom/client';
import {Layout,Button,Select,ConfigProvider,theme,Alert,InputNumber,Dropdown,Space,Modal,Switch} from 'antd';
import * as T from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {PCDLoader} from 'three/addons/loaders/PCDLoader.js';
import URDFLoader from 'urdf-loader';
import './style.css';
import {addRobotCoordinates} from './robot-coordinates.mjs';
import {beginJointDrag,updateJointDrag} from './joint-drag.mjs';
const modes=['offline','simulation','online','custom'];
function View({progress,joints,connected,onJoint,showOrigin,pcdFile,showRobotCoordinates}){
 const sceneRef=useRef(),pcdRef=useRef(),fitCloud=useRef();
 const [pcdStatus,setPcdStatus]=useState('');
 useEffect(()=>{
  if(!pcdFile)return;
  let cancelled=false;
  setPcdStatus('Loading '+pcdFile.name);
  pcdFile.arrayBuffer().then(data=>{
   if(cancelled)return;
   const cloud=new PCDLoader().parse(data);
   const positions=cloud.geometry.getAttribute('position');
   if(!positions?.count)throw Error('PCD has no points');
   for(const value of positions.array)if(!Number.isFinite(value)){cloud.geometry.dispose();cloud.material.dispose();throw Error('PCD contains invalid coordinates');}
   if(pcdRef.current){sceneRef.current.remove(pcdRef.current);pcdRef.current.geometry.dispose();pcdRef.current.material.dispose();}
   pcdRef.current=cloud;sceneRef.current.add(cloud);fitCloud.current(cloud);
   setPcdStatus(pcdFile.name+' · '+positions.count.toLocaleString()+' points');
  }).catch(e=>{if(!cancelled)setPcdStatus('PCD error: '+e.message);});
  return()=>{cancelled=true;};
 },[pcdFile]);
 const robotAxes=useRef([]),robotAxesVisible=useRef(showRobotCoordinates);
 robotAxesVisible.current=showRobotCoordinates;
 useEffect(()=>{robotAxes.current.forEach(axis=>{axis.visible=showRobotCoordinates;});},[showRobotCoordinates]);
 const originRef=useRef(),originVisible=useRef(showOrigin);originVisible.current=showOrigin;
 useEffect(()=>{if(originRef.current)originRef.current.visible=showOrigin;},[showOrigin]);
 const robotRef=useRef(), latestJoints=useRef(joints),[selected,setSelected]=useState(null);
 const interaction=useRef();interaction.current={connected,onJoint};
 latestJoints.current=joints;
 useEffect(()=>{if(robotRef.current)robotRef.current.setJointValues(joints||{});},[joints]);
 const host=useRef(),[error,setError]=useState(''),[loaded,setLoaded]=useState(false);
 useEffect(()=>{
  const scene=new T.Scene();sceneRef.current=scene;scene.background=new T.Color('#111923');
  const camera=new T.PerspectiveCamera(45,1,.01,1000);camera.up.set(0,0,1);camera.position.set(10,10,10);
  const renderer=new T.WebGLRenderer({antialias:true});renderer.setPixelRatio(Math.min(devicePixelRatio,2));host.current.appendChild(renderer.domElement);
  const controls=new OrbitControls(camera,renderer.domElement);
  let drag=null, pending=null, sending=false, selectedJoint=null, highlighted=[];
  const flush=async()=>{
   if(sending)return;
   sending=true;
   try{while(pending){const next=pending;pending=null;await interaction.current.onJoint(next.name,next.value);}}
   finally{sending=false;}
  };
  const finish=()=>{
   if(!drag)return;
   const id=drag.id;drag=null;controls.enabled=true;
   if(renderer.domElement.hasPointerCapture(id))renderer.domElement.releasePointerCapture(id);
  };
  const pointerRay=e=>{
   const rect=renderer.domElement.getBoundingClientRect(),caster=new T.Raycaster();
   caster.setFromCamera(new T.Vector2((e.clientX-rect.left)/rect.width*2-1,-(e.clientY-rect.top)/rect.height*2+1),camera);
   return caster.ray;
  };
  const move=e=>{
   if(!drag || e.pointerId!==drag.id)return;
   e.stopImmediatePropagation();e.preventDefault();
   if(!(e.buttons&1) || !interaction.current.connected){finish();return;}
   const joint=drag.joint;
   let value=updateJointDrag(drag.geometry,pointerRay(e));
   if(value===null)return;
   if(joint.jointType!=='continuous')value=T.MathUtils.clamp(value,joint.limit.lower,joint.limit.upper);
   pending={name:joint.name,value};void flush();
  };
  const release=e=>{if(drag){e.stopImmediatePropagation();finish();}};
  const clearSelection=()=>{
   finish();selectedJoint=null;
   for(const [mesh,original] of highlighted){
    (Array.isArray(mesh.material)?mesh.material:[mesh.material]).forEach(m=>m.dispose());
    mesh.material=original;
   }
   highlighted=[];setSelected(null);setError('');
  };
  host.current.clearSelection=clearSelection;
  const escape=e=>{if(e.key==='Escape'){clearSelection();}};
  window.addEventListener('keydown',escape);
  const select=e=>{
   e.preventDefault();clearSelection();
   const rect=renderer.domElement.getBoundingClientRect(),ray=new T.Raycaster();
   ray.setFromCamera(new T.Vector2((e.clientX-rect.left)/rect.width*2-1,-(e.clientY-rect.top)/rect.height*2+1),camera);
   let link=robotRef.current && ray.intersectObject(robotRef.current,true)[0]?.object;
   while(link && !link.isURDFLink)link=link.parent;
   if(!link)return;
   const highlight=node=>{
    if(node!==link && node.isURDFLink)return;
    if(node.isMesh){
     const original=node.material;
     const red=m=>{const copy=m.clone();copy.color?.set(0xff0000);return copy;};
     node.material=Array.isArray(original)?original.map(red):red(original);
     highlighted.push([node,original]);
    }
    node.children.forEach(highlight);
   };
   highlight(link);
   let joint=link.parent;
   while(joint && !(joint.isURDFJoint && ['revolute','continuous','prismatic'].includes(joint.jointType)))joint=joint.parent;
   if(joint && !joint.isURDFMimicJoint){
    selectedJoint=joint;
    setSelected({name:joint.name,type:joint.jointType,lower:joint.limit.lower,upper:joint.limit.upper});
   }else setError('선택한 링크에 조정 가능한 joint가 없습니다. 빈 공간을 더블클릭하면 선택이 해제됩니다.');
  };
  const pick=e=>{
   if(e.button!==0||!selectedJoint||!interaction.current.connected)return;
   e.stopImmediatePropagation();e.preventDefault();
   const object=selectedJoint;
   object.updateWorldMatrix(true,false);
   const origin=object.getWorldPosition(new T.Vector3());
   const axis=object.axis.clone().transformDirection(object.matrixWorld);
   const geometry=beginJointDrag(pointerRay(e),origin,axis,object.jointType,latestJoints.current?.[object.name]??0);
   if(!geometry){setError('관절 축과 시선이 겹칩니다. 선택을 해제하고 시점을 바꾼 뒤 다시 선택해 주세요.');return;}
   setError('');drag={joint:object,id:e.pointerId,geometry};
   controls.enabled=false;renderer.domElement.setPointerCapture(e.pointerId);
  };
  renderer.domElement.addEventListener('dblclick',select);
  renderer.domElement.addEventListener('pointerdown',pick,true);
  renderer.domElement.addEventListener('pointermove',move,true);
  renderer.domElement.addEventListener('pointerup',release,true);
  renderer.domElement.addEventListener('pointercancel',release,true);
  renderer.domElement.addEventListener('lostpointercapture',finish);
  window.addEventListener('blur',finish);
  scene.add(new T.HemisphereLight(0xffffff,0x627080,3));
  const light=new T.DirectionalLight(0xffffff,3);light.position.set(5,5,10);scene.add(light);
  const grid=new T.GridHelper(30,30,0x557080,0x263440);grid.rotation.x=Math.PI/2;scene.add(grid);
  const originAxes=new T.AxesHelper(1);originAxes.visible=originVisible.current;scene.add(originAxes);originRef.current=originAxes;
  const axesScene=new T.Scene(),axesCamera=new T.PerspectiveCamera(45,1,.1,10);axesCamera.up.set(0,0,1);
  axesScene.add(new T.AxesHelper(1.3));
  for(const [label,pos,color] of [['X',[1.6,0,0],'#ff5555'],['Y',[0,1.6,0],'#55ee77'],['Z',[0,0,1.6],'#5599ff']]){
   const canvas=document.createElement('canvas');canvas.width=canvas.height=64;const ctx=canvas.getContext('2d');ctx.font='bold 44px sans-serif';ctx.fillStyle=color;ctx.fillText(label,12,48);
   const sprite=new T.Sprite(new T.SpriteMaterial({map:new T.CanvasTexture(canvas),depthTest:false,depthWrite:false}));sprite.position.set(...pos);sprite.scale.setScalar(.7);sprite.renderOrder=10;axesScene.add(sprite);
  }
  let robot,disposed=false;
  const fit=()=>{if(!robot)return;const box=new T.Box3().setFromObject(robot),c=box.getCenter(new T.Vector3()),d=box.getSize(new T.Vector3()).length();controls.target.copy(c);camera.position.copy(c).add(new T.Vector3(1,-1,.8).normalize().multiplyScalar(d*1.2));controls.update();};
  const manager=new T.LoadingManager();manager.onError=url=>setError('리소스 로드 실패: '+url);manager.onLoad=()=>{if(!disposed){fit();setLoaded(true);}};
  manager.setURLModifier(url=>{const resolved=new URL(url,location.href);if(resolved.origin!==location.origin||!resolved.pathname.startsWith('/resources/'))throw Error('Only local model resources allowed');return resolved.href;});
  new URDFLoader(manager).load('/resources/urdf/model.urdf',value=>{robot=value;robotRef.current=robot;robotAxes.current=addRobotCoordinates(robot,robotAxesVisible.current);robot.setJointValues(latestJoints.current||{});if(!disposed)scene.add(robot);},undefined,e=>setError(String(e)));
  const resize=new ResizeObserver(()=>{const {width,height}=host.current.getBoundingClientRect();renderer.setSize(width,height);camera.aspect=width/height;camera.updateProjectionMatrix();});resize.observe(host.current);
  fitCloud.current=cloud=>{
   const box=new T.Box3().setFromObject(cloud),center=box.getCenter(new T.Vector3()),size=box.getSize(new T.Vector3()).length()||1;
   cloud.material.size=size/1500;camera.near=size/10000;camera.far=Math.max(size*100,1000);camera.updateProjectionMatrix();
   controls.target.copy(center);camera.position.copy(center).add(new T.Vector3(1,-1,.8).normalize().multiplyScalar(size*1.2));controls.update();
  };
  host.current.fit=()=>pcdRef.current?fitCloud.current(pcdRef.current):fit();
  renderer.setAnimationLoop(()=>{controls.update();const w=renderer.domElement.clientWidth,h=renderer.domElement.clientHeight;renderer.setViewport(0,0,w,h);renderer.render(scene,camera);renderer.autoClear=false;renderer.clearDepth();renderer.setViewport(12,12,110,110);axesCamera.position.copy(camera.position).sub(controls.target).normalize().multiplyScalar(6);axesCamera.lookAt(0,0,0);renderer.render(axesScene,axesCamera);renderer.autoClear=true;});
  return()=>{window.removeEventListener("keydown",escape);originRef.current=null;clearSelection();pending=null;renderer.domElement.removeEventListener('dblclick',select);
   renderer.domElement.removeEventListener('pointermove',move,true);
   renderer.domElement.removeEventListener('pointerup',release,true);
   renderer.domElement.removeEventListener('pointercancel',release,true);
   renderer.domElement.removeEventListener('lostpointercapture',finish);
   window.removeEventListener('blur',finish);
   renderer.domElement.removeEventListener("pointerdown",pick,true);robotRef.current=null;disposed=true;resize.disconnect();renderer.setAnimationLoop(null);controls.dispose();scene.traverse(o=>{o.geometry?.dispose();if(o.material)(Array.isArray(o.material)?o.material:[o.material]).forEach(m=>m.dispose());});renderer.dispose();renderer.domElement.remove();};
 },[]);
 return <div className="viewport" ref={host} data-model-ready={loaded}><div className="scene-label">{pcdStatus&&<p role="status">{pcdStatus}</p>}</div><Button className="fit" onClick={()=>host.current.fit?.()}>Fit view</Button>{selected&&<div className="joint-editor"><strong>{selected.name}</strong><p>{selected.type} · {selected.type==='prismatic'?'m':'rad'}</p><InputNumber aria-label="Joint value" disabled={!connected} min={selected.type==='continuous'?undefined:selected.lower} max={selected.type==='continuous'?undefined:selected.upper} step={.01} value={joints?.[selected.name]??0} onChange={v=>{if(v!==null)onJoint(selected.name,v);}}/><Button onClick={()=>host.current.clearSelection?.()}>Close</Button><p>{selected.type==='continuous'?'Continuous rotation':selected.lower+' … '+selected.upper}</p></div>}{error&&<Alert className="error" type="error" title={error}/>}<div className="view-footer">Double-click: select / clear · Selected: left-drag joint · Unselected: left-drag orbit · Pan: right drag · Zoom: scroll <span>Processing {progress}%</span></div></div>;
}
function App(){
 const fileInput=useRef();
 const [pcdFile,setPcdFile]=useState(null),[settingsOpen,setSettingsOpen]=useState(false),[draft,setDraft]=useState({speed:1,show_origin:true}),[saving,setSaving]=useState(false);
 async function applySettings(save){
  setSaving(true);
  const ok=await request(save?'/api/settings':'/api/state',save?'PUT':'PATCH',draft);
  setSaving(false);if(ok&&save)setSettingsOpen(false);
 }

 const [collapsed,setCollapsed]=useState(false),[state,setState]=useState(null),[connected,setConnected]=useState(false),[error,setError]=useState('');
 useEffect(()=>{let socket,timer,stopped=false;function connect(){socket=new WebSocket(location.origin.replace(/^http/,'ws')+'/ws');socket.onopen=()=>setConnected(true);socket.onmessage=e=>setState(JSON.parse(e.data));socket.onclose=()=>{setConnected(false);if(!stopped)timer=setTimeout(connect,1000);};}connect();return()=>{stopped=true;clearTimeout(timer);socket?.close();};},[]);
 async function request(path,method,body){try{const r=await fetch(path,{method,headers:{'Content-Type':'application/json'},body:body?JSON.stringify(body):undefined});if(!r.ok)throw Error(await r.text());setError('');return true;}catch(e){setError(e.message);return false;}}
 return <ConfigProvider theme={{algorithm:theme.darkAlgorithm,token:{colorPrimary:'#35b7bc',borderRadius:5}}}><Layout className="app"><Layout.Sider trigger={null} collapsible collapsed={collapsed} width={250} breakpoint="lg" onBreakpoint={setCollapsed}><div className="brand">{collapsed?'S':'SPARC'}</div></Layout.Sider><Layout><Layout.Header className="header"><Button aria-label="Toggle sidebar" onClick={()=>setCollapsed(!collapsed)}>☰</Button><nav className="header-menus" aria-label="Application menu"><Dropdown trigger={['click']} placement="bottomLeft" menu={{items:[{key:'pcd',label:'Load PCD',onClick:()=>fileInput.current.click()},{type:'divider'},{key:'settings',label:'Settings',onClick:()=>{setDraft({speed:state?.speed??1,show_origin:state?.show_origin??true});setSettingsOpen(true);}},{key:'reload',label:'Reload',onClick:()=>location.reload()}]}}><Button type="text" className="menu-trigger" aria-label="File"><Space>File<span aria-hidden="true">⌄</span></Space></Button></Dropdown><Dropdown trigger={['click']} menu={{items:['Pose Finder','Path Planner','Optimizer'].map(label=>({key:label,label,children:[{key:label+'-empty',label:'No algorithms registered',disabled:true}]}))}}><Button type="text" className="menu-trigger" aria-label="Process"><Space>Process<span aria-hidden="true">⌄</span></Space></Button></Dropdown><Dropdown trigger={['click']} placement="bottomLeft" menu={{items:[{key:'origin',label:<span role="menuitemcheckbox" aria-checked={state?.show_origin??true}>{state?.show_origin?'✓ ':''}Show Origin Coordinates</span>,disabled:!connected,onClick:()=>request('/api/state','PATCH',{show_origin:!state?.show_origin})},{key:'robot-coordinates',label:<span role="menuitemcheckbox" aria-checked={state?.show_robot_coordinates??false}>{state?.show_robot_coordinates?'✓ ':''}Show Robot Coord.</span>,disabled:!connected,onClick:()=>request('/api/state','PATCH',{show_robot_coordinates:!state?.show_robot_coordinates})}]}}><Button type="text" className="menu-trigger" aria-label="View"><Space>View<span aria-hidden="true">⌄</span></Space></Button></Dropdown></nav><Select aria-label="Operating mode" value={state?.mode??'offline'} disabled={!connected} style={{width:150}} options={modes.map(value=>({value,label:value.toUpperCase()}))} onChange={mode=>request('/api/state','PATCH',{mode})}/></Layout.Header><Layout.Content className="content">{error&&<Alert type="error" title={error}/>}<View showRobotCoordinates={state?.show_robot_coordinates??false} pcdFile={pcdFile} showOrigin={state?.show_origin??true} progress={state?.progress??0} joints={state?.joints} connected={connected} onJoint={(name,value)=>request("/api/joints/"+encodeURIComponent(name),"PATCH",{value})}/></Layout.Content></Layout></Layout><input ref={fileInput} type="file" accept=".pcd" hidden onChange={e=>{const file=e.target.files[0];if(file)setPcdFile(file);e.target.value='';}}/><Modal title="Settings" open={settingsOpen} onCancel={()=>setSettingsOpen(false)} footer={<Space><Button disabled={!connected||saving} onClick={()=>applySettings(false)}>Apply</Button><Button type="primary" disabled={!connected||saving} onClick={()=>applySettings(true)}>Save &amp; Close</Button></Space>}>{error&&<Alert type="error" title={error}/>}<p>Processing speed</p><InputNumber aria-label="Settings processing speed" min={0} max={2} step={.1} value={draft.speed} onChange={v=>{if(v!==null)setDraft({...draft,speed:v});}}/><p>Show origin coordinates</p><Switch checked={draft.show_origin} onChange={v=>setDraft({...draft,show_origin:v})}/></Modal></ConfigProvider>;
}
createRoot(document.getElementById('root')).render(<App/>);
