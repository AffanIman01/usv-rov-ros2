import fs from 'node:fs';
import path from 'node:path';
import * as T from 'three';
import {GLTFLoader} from 'three/addons/loaders/GLTFLoader.js';
import {createSensorRack} from './sensor-rack.mjs';
import {enrichRovModel} from './rov-anatomy.mjs';
import {makeMechanism} from './mechanism.mjs';

// Usage: node export_models.mjs SOURCE_DIST OUTPUT_PACKAGE
const source=path.resolve(process.argv[2]);
const out=path.resolve(process.argv[3]);
for(const d of ['meshes','urdf','models','worlds','config'])fs.mkdirSync(path.join(out,d),{recursive:true});
const loader=new GLTFLoader();
async function load(file){const b=fs.readFileSync(path.join(source,'models',file));return (await loader.parseAsync(b.buffer.slice(b.byteOffset,b.byteOffset+b.byteLength),'')).scene;}
const xml=s=>String(s).replaceAll('&','&amp;').replaceAll('<','&lt;').replaceAll('"','&quot;');
const num=n=>Math.abs(n)<0.00000005?'0':Number(n.toFixed(7)).toString();
const nums=a=>Array.from(a,num).join(' ');
const safe=s=>s.replace(/[^A-Za-z0-9_]/g,'_');
// Proper rotations only; never reflect a mesh to reconcile inherited side labels.
const glbToROS=p=>[p[2],p[0],p[1]];
const sceneToROS=p=>[p[0],-p[2],p[1]];
const records=[];

function exportPart(name,root,rotate,exclude=new Set()){
 root.updateWorldMatrix(true,true);
 const origin=rotate(root.getWorldPosition(new T.Vector3()).toArray());
 const primitives=[];
 function visit(o){
  if(exclude.has(o))return;
  if(o.isMesh){
   const g=o.geometry,p=g.attributes.position,n=g.attributes.normal;
   const index=g.index;const count=index?index.count:p.count;
   const groups=Array.isArray(o.material)?g.groups:[{start:0,count,materialIndex:0}];
   const nm=new T.Matrix3().getNormalMatrix(o.matrixWorld);
   for(const group of groups){
    const material=Array.isArray(o.material)?o.material[group.materialIndex]:o.material;
    if(!material||group.count===0)continue;
    const positions=[],normals=[],indices=[];const remap=new Map();
    for(let i=group.start;i<Math.min(group.start+group.count,count);i++){
     const old=index?index.getX(i):i;
     if(!remap.has(old)){
      const pos=new T.Vector3().fromBufferAttribute(p,old).applyMatrix4(o.matrixWorld);
      const pp=rotate(pos.toArray()).map((v,j)=>v-origin[j]);positions.push(...pp);
      const nn=n?new T.Vector3().fromBufferAttribute(n,old).applyMatrix3(nm).normalize():new T.Vector3(0,1,0);
      normals.push(...rotate(nn.toArray()));remap.set(old,remap.size);
     }
     indices.push(remap.get(old));
    }
    const color=material.color||new T.Color(0xcccccc);
    primitives.push({positions,normals,indices,color:[color.r,color.g,color.b,material.opacity??1],name:o.name||name});
   }
  }
  for(const child of o.children)visit(child);
 }
 visit(root);
 if(!primitives.length)throw new Error('Empty export: '+name);
 return {name:safe(name),origin,primitives};
}

function dae(part){
 const effects=[],materials=[],geometries=[],nodes=[];
 part.primitives.forEach((p,i)=>{
  const id=`g${i}`,mat=`m${i}`;
  effects.push(`<effect id="${mat}_effect"><profile_COMMON><technique sid="common"><phong><ambient><color>${nums(p.color)}</color></ambient><diffuse><color>${nums(p.color)}</color></diffuse><specular><color>0.12 0.12 0.12 1</color></specular><shininess><float>20</float></shininess></phong></technique></profile_COMMON></effect>`);
  materials.push(`<material id="${mat}" name="${mat}"><instance_effect url="#${mat}_effect"/></material>`);
  const src=(suffix,values)=>`<source id="${id}_${suffix}"><float_array id="${id}_${suffix}_array" count="${values.length}">${nums(values)}</float_array><technique_common><accessor source="#${id}_${suffix}_array" count="${values.length/3}" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>`;
  geometries.push(`<geometry id="${id}" name="${xml(p.name)}"><mesh>${src('positions',p.positions)}${src('normals',p.normals)}<vertices id="${id}_vertices"><input semantic="POSITION" source="#${id}_positions"/></vertices><triangles count="${p.indices.length/3}" material="${mat}"><input semantic="VERTEX" source="#${id}_vertices" offset="0"/><input semantic="NORMAL" source="#${id}_normals" offset="1"/><p>${p.indices.map(n=>`${n} ${n}`).join(' ')}</p></triangles></mesh></geometry>`);
  nodes.push(`<node id="node${i}"><instance_geometry url="#${id}"><bind_material><technique_common><instance_material symbol="${mat}" target="#${mat}"/></technique_common></bind_material></instance_geometry></node>`);
 });
 return `<?xml version="1.0" encoding="utf-8"?>\n<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1"><asset><contributor><authoring_tool>USV-ROV site visual transfer</authoring_tool></contributor><created>2026-11-06T00:00:00Z</created><modified>2026-11-06T00:00:00Z</modified><unit name="meter" meter="1"/><up_axis>Z_UP</up_axis></asset><library_effects>${effects.join('')}</library_effects><library_materials>${materials.join('')}</library_materials><library_geometries>${geometries.join('')}</library_geometries><library_visual_scenes><visual_scene id="Scene">${nodes.join('')}</visual_scene></library_visual_scenes><scene><instance_visual_scene url="#Scene"/></scene></COLLADA>`;
}

function saveModel(name,parts,frames=[],notes=[]){
 const min=[Infinity,Infinity,Infinity],max=[-Infinity,-Infinity,-Infinity];
 let triangles=0;
 const dir=path.join(out,'meshes',name);fs.mkdirSync(dir,{recursive:true});
 for(const part of parts){
  fs.writeFileSync(path.join(dir,part.name+'.dae'),dae(part));
  for(const p of part.primitives){triangles+=p.indices.length/3;for(let i=0;i<p.positions.length;i+=3)for(let a=0;a<3;a++){const v=p.positions[i+a]+part.origin[a];min[a]=Math.min(min[a],v);max[a]=Math.max(max[a],v);}}
 }
 const record={name,parts:parts.map(({name,origin,primitives})=>({name,origin,triangles:primitives.reduce((n,p)=>n+p.indices.length/3,0)})),frames,notes,bounds:{min,max,size:max.map((v,i)=>v-min[i])},triangles};
 records.push(record);
 fs.writeFileSync(path.join(out,'urdf',name+'.urdf'),urdf(name,[{record,offset:[0,0,0],prefix:''}]));
 const modelDir=path.join(out,'models',name);fs.mkdirSync(modelDir,{recursive:true});
 const visuals=record.parts.map(p=>`<visual name="${p.name}"><pose>${nums(p.origin)} 0 0 0</pose><geometry><mesh><uri>model://usv_rov_description/meshes/${name}/${p.name}.dae</uri></mesh></geometry></visual>`).join('\n');
 fs.writeFileSync(path.join(modelDir,'model.sdf'),`<?xml version="1.0"?><sdf version="1.9"><model name="${name}"><static>true</static><link name="base_link">${visuals}</link></model></sdf>\n`);
 fs.writeFileSync(path.join(modelDir,'model.config'),`<?xml version="1.0"?><model><name>${name}</name><version>0.1.0</version><sdf version="1.9">model.sdf</sdf><description>Visual-only transfer from Recovery Studio; approximate geometry, no dynamics or sensor output.</description></model>\n`);
 // Intermediate for verification and a preview rendered from the transferred geometry.
 fs.writeFileSync(path.join(path.dirname(out),'.'+name+'_geometry.json'),JSON.stringify({record,parts}));
}
function urdf(name,instances){
 let body='<link name="base_link"/>\n';
 for(const {record,offset,prefix}of instances){
  for(const p of record.parts){const id=prefix+p.name;body+=`<link name="${id}"><visual><geometry><mesh filename="package://usv_rov_description/meshes/${record.name}/${p.name}.dae"/></geometry></visual></link><joint name="${id}_fixed" type="fixed"><parent link="base_link"/><child link="${id}"/><origin xyz="${nums(p.origin.map((v,i)=>v+offset[i]))}" rpy="0 0 0"/></joint>\n`;}
  for(const f of record.frames){const id=prefix+f.name;body+=`<link name="${id}"/><joint name="${id}_fixed" type="fixed"><parent link="base_link"/><child link="${id}"/><origin xyz="${nums(f.origin.map((v,i)=>v+offset[i]))}" rpy="0 0 0"/></joint>\n`;}
 }
 return `<?xml version="1.0"?>\n<!-- Visual description only. Fixed placements are source estimates, not measured extrinsics. -->\n<robot name="${name}">\n${body}</robot>\n`;
}

const vessel=await load('me120-reference.glb');
vessel.scale.x=1.4/1.4051200151443482;
vessel.position.y=-.25;
for(const name of ['payload_lifter_moving','payload_mounting_ring'])vessel.getObjectByName(name).position.y+=.418;
const antenna=vessel.getObjectByName('antenna');antenna.position.y-=.342;
const rack=createSensorRack();antenna.add(rack.root);
// Same ivory-aerial assignment as the site's matchAerialColours().
const rod=antenna.children.find(n=>n.isMesh&&n.material?.name==='gray');
if(rod){const geo=rod.geometry,pos=geo.attributes.position,idx=geo.index,count=idx?idx.count:pos.count;const ivory=rod.material.clone();ivory.color.setHex(0xe5e4d7);rod.material=[rod.material,ivory];geo.clearGroups();let start=0,last=-1;for(let i=0;i<count;i+=3){let x=0;for(let j=0;j<3;j++)x+=pos.getX(idx?idx.getX(i+j):i+j)/3;const which=Math.abs(x+.08)<.025?1:0;if(last!==which){if(last>=0)geo.addGroup(start,i-start,last);start=i;last=which;}}if(last>=0)geo.addGroup(start,count-start,last);}
vessel.updateMatrixWorld(true);
const vesselParts=vessel.children.map(n=>exportPart(n.name,n,glbToROS,new Set([rack.root])));
for(const n of rack.root.children)vesselParts.push(exportPart(n.name,n,glbToROS));
const starlinkSupport=new T.Mesh(new T.CylinderGeometry(.016,.016,.60,12),new T.MeshStandardMaterial({color:0xa4b7c6}));
starlinkSupport.position.set(-.80,.48,-.54);
vesselParts.push(exportPart('Starlink_support_concept',starlinkSupport,sceneToROS));
// Mounted accessories are siblings of the vessel in the source's originalFrame.
for(const [file,name,pos,scale]of [['chasing-e-reel-reference.glb','mounted_e_reel',[0,.585,-.615],.8],['starlink-mini-concept.glb','mounted_starlink_mini',[.54,.78,-.80],1]]){
 const model=await load(file);model.position.set(...pos);model.scale.setScalar(scale);model.updateMatrixWorld(true);vesselParts.push(exportPart(name,model,glbToROS));
}
const frames=[
 {name:'robosense_mount_frame',origin:glbToROS(rack.owners['robosense-lidar'].getWorldPosition(new T.Vector3()).toArray())},
 {name:'zed_mount_frame',origin:glbToROS(rack.owners['zed-camera'].getWorldPosition(new T.Vector3()).toArray())},
 {name:'hikvision_mount_frame',origin:glbToROS(antenna.localToWorld(new T.Vector3(0,1.735,.108)).toArray())}
];
saveModel('me120',vesselParts,frames,['Studio neutral raised pose; waterline datum Y=-0.25 applied.','Hikvision visual remains within antenna mesh; frame is the site inspection centre, not an optical calibration.','Mounted E-Reel retains source visual scale 0.8. Standalone E-Reel remains native scale.','Small canvas wordmarks are omitted; geometric sensor housings and cable loops retained.','Source hull_port/hull_starboard identifiers are inherited labels; do not infer physical left/right from them.']);
const rov=enrichRovModel(await load('chasing-m2-pro-max-reference.glb'));rov.updateMatrixWorld(true);
saveModel('m2_pro_max',rov.children.map(n=>exportPart(n.name,n,glbToROS)),[],['Includes the site ROV anatomy corrections; no lighting beams or animated effects.']);
for(const [file,name]of [['chasing-e-reel-reference.glb','e_reel'],['starlink-mini-concept.glb','starlink_mini'],['compute-enclosure-concept.glb','compute_enclosure']]){
 const model=await load(file);saveModel(name,model.children.map(n=>exportPart(n.name,n,glbToROS)));
}
const mechanism=makeMechanism();
saveModel('recovery_concept',[exportPart('capture_head',mechanism.head,sceneToROS),exportPart('rov_yoke',mechanism.yoke,sceneToROS),exportPart('tether_guide',mechanism.tetherGuide,sceneToROS)],[],['Separate conceptual parts in source construction datums; not the assembled deployed pose.','Includes inherited lock pins/latch housings and keeper lugs from current source. These are not approval of a final mechanical design.','Tether guide is boat-local; head/yoke are carrier-local. Their zero origins are not a mating constraint.']);
const xy=[[0,0],[0,-1.7],[1,-1.7],[1,-2.4],[1,-2.9],[-.9,-2.3]];
const instances=records.map((record,i)=>({record,prefix:record.name+'_',offset:[...xy[i],.03-record.bounds.min[2]]}));
fs.writeFileSync(path.join(out,'urdf','showcase.urdf'),urdf('usv_rov_showcase',instances));
const include=instances.map(({record,offset})=>`<include><uri>model://usv_rov_description/models/${record.name}</uri><name>${record.name}</name><pose>${nums(offset)} 0 0 0</pose></include>`).join('\n');
fs.writeFileSync(path.join(out,'worlds','model_preview.sdf'),`<?xml version="1.0"?><sdf version="1.9"><world name="usv_rov_visual_preview"><plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/><plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/><plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/><gravity>0 0 -9.81</gravity><scene><ambient>0.6 0.6 0.6 1</ambient><background>0.16 0.20 0.24 1</background></scene><light name="sun" type="directional"><pose>0 0 10 0 0 0</pose><diffuse>0.9 0.9 0.9 1</diffuse><direction>-0.4 0.2 -0.9</direction></light><model name="ground"><static>true</static><link name="ground"><visual name="ground"><geometry><plane><normal>0 0 1</normal><size>12 12</size></plane></geometry><material><ambient>0.32 0.36 0.39 1</ambient><diffuse>0.32 0.36 0.39 1</diffuse></material></visual></link></model>${include}</world></sdf>\n`);
fs.writeFileSync(path.join(out,'config','model_manifest.json'),JSON.stringify({source_project:'appgprj_6ac346cd35888191b75859ccf9a8d5b5',source_commit:'9edda0a7a7fcd8700558ed00c44bbd2e29d9196e',source_site_version:58,export_date:'2026-11-06',units:'metres',axis_conversion:{glb:'(z,x,y)',scene:'(x,-z,y)',determinant:1},status:'visual-only; no dynamics, collisions, sensor data or calibrated transforms',models:records,showcase:instances.map(({record,offset})=>({model:record.name,offset}))},null,2));
console.log(JSON.stringify(records.map(r=>({name:r.name,parts:r.parts.length,triangles:r.triangles,size:r.bounds.size})),null,2));
