import * as THREE from '../vendor/three/three.module.js?v=0.186.1';
import {buildDeposition,beadVertices,depositedRange,isDepositing} from './deposition.js?v=2.2';
import {primitivePose} from './simulation.js?v=2.2';

export function createConcreteViewer(container) {
  const renderer=new THREE.WebGLRenderer({antialias:true,powerPreference:'high-performance'});
  renderer.setPixelRatio(Math.min(window.devicePixelRatio||1,2));
  renderer.setClearColor(0x151c21);renderer.outputColorSpace=THREE.SRGBColorSpace;
  renderer.toneMapping=THREE.ACESFilmicToneMapping;renderer.toneMappingExposure=.85;
  renderer.domElement.className='concrete-webgl';renderer.domElement.setAttribute('aria-label','Deposición tridimensional de concreto');
  container.appendChild(renderer.domElement);
  const scene=new THREE.Scene(),camera=new THREE.OrthographicCamera(-1,1,1,-1,.1,10000);
  camera.up.set(0,0,1);
  scene.add(new THREE.HemisphereLight(0xe3efff,0x404244,2.2));
  const light=new THREE.DirectionalLight(0xffffff,3);light.position.set(-150,-250,500);scene.add(light);
  const fill=new THREE.DirectionalLight(0xffd2ab,1.1);fill.position.set(300,200,150);scene.add(fill);
  const content=new THREE.Group();scene.add(content);
  const material=new THREE.MeshStandardMaterial({vertexColors:true,roughness:.95,metalness:0,side:THREE.DoubleSide});
  const nozzleMaterial=new THREE.MeshStandardMaterial({color:0xf3a35c,metalness:.2,roughness:.5,transparent:true,opacity:.9});
  let lastSim=null,lastTrack=null,lastKey=null,data=null,mesh=null,partial=null,nozzle=null,axes=null,grid=null;
  let original=null,paths=null,travels=null,bounds=new THREE.Box3(),lastSize=null;
  const raycaster=new THREE.Raycaster(),pointer=new THREE.Vector2();

  function clean() {
    content.traverse(object=>{object.geometry?.dispose();if(object.material&&object.material!==material&&object.material!==nozzleMaterial){object.material.map?.dispose();object.material.dispose();}});
    content.clear();
  }
  function lineSegments(values,color,dashed=false) {
    const geometry=new THREE.BufferGeometry();geometry.setAttribute('position',new THREE.Float32BufferAttribute(values,3));
    const line=new THREE.LineSegments(geometry,dashed?new THREE.LineDashedMaterial({color,dashSize:4,gapSize:3,transparent:true,opacity:.65}):new THREE.LineBasicMaterial({color,transparent:true,opacity:.5}));
    if(dashed)line.computeLineDistances();content.add(line);return line;
  }
  function label(text,position,size) {
    const canvas=document.createElement('canvas');canvas.width=128;canvas.height=64;
    const ctx=canvas.getContext('2d');ctx.fillStyle='#e3edf2';ctx.font='bold 42px sans-serif';ctx.textAlign='center';ctx.fillText(text,64,48);
    const texture=new THREE.CanvasTexture(canvas),sprite=new THREE.Sprite(new THREE.SpriteMaterial({map:texture,depthTest:false}));
    sprite.position.copy(position);sprite.scale.set(size*2,size,1);content.add(sprite);
  }
  function rebuild(sim,track,include,key) {
    clean();data=buildDeposition(sim,track,include);
    const geometry=new THREE.BufferGeometry();geometry.setAttribute('position',new THREE.BufferAttribute(data.positions,3));
    geometry.setAttribute('color',new THREE.BufferAttribute(data.colors,3));geometry.computeVertexNormals();geometry.setDrawRange(0,0);
    mesh=new THREE.Mesh(geometry,material);content.add(mesh);
    const partGeometry=new THREE.BufferGeometry();partGeometry.setAttribute('position',new THREE.Float32BufferAttribute(new Float32Array(108),3));
    partGeometry.setAttribute('color',new THREE.Float32BufferAttribute(new Float32Array(108),3));
    partial=new THREE.Mesh(partGeometry,material);partial.visible=false;partial.frustumCulled=false;content.add(partial);
    const route=[],travel=[];
    for(const p of (sim.machine_primitives||[]).filter(include)) {
      const destination=isDepositing(p)?route:travel;
      const count=p.type==='ARC'?Math.min(256,Math.max(1,Math.ceil(Math.abs(p.sweep_rad)/.08))):1;
      for(let i=0;i<count;i++)for(const t of [i/count,(i+1)/count]) {
        const q=primitivePose(p,t);destination.push(q.X,q.Y,p.generated_reason==='INITIAL_XY_APPROACH'?p.end_xyz[2]:q.Z);
      }
    }
    paths=lineSegments(route,0xeab17a);travels=lineSegments(travel,0x69b9dd,true);
    const raw=[];
    for(const stroke of (sim.original_strokes||[]).filter(include))for(let i=1;i<stroke.points.length;i++)raw.push(...stroke.points[i-1],...stroke.points[i]);
    original=lineSegments(raw,0x7c8993);
    bounds.makeEmpty();
    for(const values of [data.positions,route,travel])for(let i=0;i<values.length;i+=3)bounds.expandByPoint(new THREE.Vector3(values[i],values[i+1],values[i+2]));
    bounds.expandByPoint(new THREE.Vector3(0,0,0));if(bounds.isEmpty())bounds.set(new THREE.Vector3(-10,-10,0),new THREE.Vector3(10,10,20));
    const span=Math.max(...bounds.getSize(new THREE.Vector3()).toArray(),30),axisSize=span*.22;
    axes=new THREE.AxesHelper(axisSize);axes.material.depthTest=false;axes.renderOrder=3;content.add(axes);
    label('X',new THREE.Vector3(axisSize*1.12,0,0),span*.035);
    label('Y',new THREE.Vector3(0,axisSize*1.12,0),span*.035);
    label('Z',new THREE.Vector3(0,0,axisSize*1.12),span*.035);
    grid=new THREE.GridHelper(span*1.8,24,0x4a5c66,0x28363e);grid.rotation.x=Math.PI/2;
    const center=bounds.getCenter(new THREE.Vector3());grid.position.set(center.x,center.y,Math.min(0,bounds.min.z)-.15);content.add(grid);
    const profile=sim.audit?.config?.nozzle||{};
    nozzle=new THREE.Mesh(new THREE.BoxGeometry(profile.length_mm??35,profile.width_mm??15,12),nozzleMaterial);content.add(nozzle);
    lastSim=sim;lastTrack=track;lastKey=key;
  }
  function frame(view,w,h) {
    const size=bounds.getSize(new THREE.Vector3()),span=Math.max(size.x,size.y,size.z,30);
    const center=view.focus?new THREE.Vector3(...view.focus):bounds.getCenter(new THREE.Vector3());
    const elevation=Math.max(-Math.PI/2+.001,Math.min(Math.PI/2-.001,view.elevation));
    const direction=new THREE.Vector3(Math.sin(view.yaw)*Math.cos(elevation),-Math.cos(view.yaw)*Math.cos(elevation),Math.sin(elevation));
    camera.position.copy(center).addScaledVector(direction,span*4+100);camera.up.set(0,0,1);camera.lookAt(center);camera.updateMatrixWorld();
    let extentX=0,extentY=0;
    for(const x of [bounds.min.x,bounds.max.x])for(const y of [bounds.min.y,bounds.max.y])for(const z of [bounds.min.z,bounds.max.z]) {
      const p=new THREE.Vector3(x,y,z).applyMatrix4(camera.matrixWorldInverse);extentX=Math.max(extentX,Math.abs(p.x));extentY=Math.max(extentY,Math.abs(p.y));
    }
    const halfHeight=Math.max(extentY,extentX*h/w,15)*1.16,halfWidth=halfHeight*w/h;
    camera.left=-halfWidth;camera.right=halfWidth;camera.top=halfHeight;camera.bottom=-halfHeight;
    camera.near=.01;camera.far=span*10+1000;camera.zoom=view.zoom;camera.updateProjectionMatrix();
    camera.translateX(-view.panX*2*halfWidth/(w*view.zoom));camera.translateY(view.panY*2*halfHeight/(h*view.zoom));camera.updateMatrixWorld();
  }
  function draw({sim,track,head,include,key,view,options}) {
    renderer.domElement.hidden=false;
    if(lastSim!==sim||lastTrack!==track||lastKey!==key)rebuild(sim,track,include,key);
    const box=container.getBoundingClientRect(),w=Math.max(1,box.width),h=Math.max(1,box.height);
    if(!lastSize||lastSize[0]!==w||lastSize[1]!==h){renderer.setSize(w,h,false);lastSize=[w,h];}
    frame(view,w,h);
    const range=depositedRange(data,head);mesh.geometry.setDrawRange(0,range.complete*36);mesh.visible=options.material;
    partial.visible=options.material&&!!range.partial;
    if(range.partial) {
      const p=partial.geometry.attributes.position,c=partial.geometry.attributes.color;
      p.array.set(beadVertices(range.partial.piece,range.partial.end));p.needsUpdate=true;
      for(let i=0;i<36;i++)c.setXYZ(i,...range.partial.piece.color);c.needsUpdate=true;
      partial.geometry.computeVertexNormals();partial.userData.sampleId=range.partial.piece.primitive.sample_id;
    }
    paths.visible=options.paths&&options.compare!=='original';travels.visible=options.travels;
    original.visible=options.paths&&options.compare!=='optimized';
    axes.visible=options.axes;for(const obj of content.children)if(obj.isSprite)obj.visible=options.axes;
    nozzle.visible=options.nozzle&&!!head;
    if(head){nozzle.position.set(head.pose.X,head.pose.Y,head.pose.Z+6);nozzle.rotation.z=(head.pose.C_deg-(head.pose.c_reference_shift_deg||0))*Math.PI/180;}
    renderer.render(scene,camera);
    return {segments:data.pieces.length,heights:data.heights,firstHeight:data.firstHeight,nextHeight:data.nextHeight,heightSource:data.heightSource};
  }
  function pick(x,y) {
    if(!mesh)return null;
    const box=container.getBoundingClientRect();pointer.set(x/box.width*2-1,-y/box.height*2+1);raycaster.setFromCamera(pointer,camera);
    const hit=raycaster.intersectObjects([mesh,partial].filter(o=>o.visible),false)[0];
    if(!hit)return null;
    return hit.object===partial?partial.userData.sampleId:data.pieces[Math.floor(hit.faceIndex/12)]?.primitive.sample_id;
  }
  return {draw,pick,hide:()=>{renderer.domElement.hidden=true;},dispose:()=>{clean();material.dispose();nozzleMaterial.dispose();renderer.dispose();renderer.domElement.remove();}};
}
