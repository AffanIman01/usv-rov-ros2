import * as T from 'three';
import {getHardwareBounds} from './model-framing.mjs?v=research-v';

// The supplied training deck establishes component identity and general placement.
// Service-part dimensions remain visual estimates, not an OEM interface drawing.
const scale=[.9735099783508815,.8497885496304035,.9999202388779301];
const offset=[0,.011322807520627975,.0009757578372955322];
const point=p=>p.map((v,i)=>(v+offset[i])*scale[i]);
const raw=(p,i)=>(p[i]/scale[i])-offset[i];
const pdf='./training/m2-pro-max/CHASING_M2_PRO_MAX_Training.pdf';

// Separate existing triangles into named parts without changing the hull or rotor geometry.
function extract(parent,name,test){
 const group=new T.Group();group.name=name;group.userData.accuracy='Approximate reference geometry';
 for(const mesh of [...parent.children]){
  if(!mesh.isMesh)continue;
  const geo=mesh.geometry,p=geo.attributes.position,index=geo.index,keep=[],take=[];
  for(let i=0;i<(index?.count||p.count);i+=3){
   const ids=[0,1,2].map(k=>index?index.getX(i+k):i+k);
   const vertices=ids.map(j=>{const q=[p.getX(j),p.getY(j),p.getZ(j)];return q.map((v,a)=>raw(q,a));});
   (test(vertices,mesh.material?.name)?take:keep).push(...ids);
  }
  if(!take.length)continue;
  const make=ids=>{const g=geo.clone();g.setIndex(ids);const compact=g.toNonIndexed();g.dispose();compact.computeBoundingBox();compact.computeBoundingSphere();return compact;};
  const part=new T.Mesh(make(take),mesh.material.clone());part.castShadow=part.receiveShadow=true;group.add(part);
  if(keep.length)mesh.geometry=make(keep);else{parent.remove(mesh);mesh.material.dispose();}geo.dispose();
 }
 parent.add(group);return group;
}

export function enrichRovModel(model){
 if(model.getObjectByName('rov_service_sd'))return model;
 const body=model.getObjectByName('rov_body'),battery=model.getObjectByName('rov_battery');
 if(!body||!battery)return model;
 const ports=extract(body,'rov_accessory_interfaces',v=>v.every(p=>p[1]>=.15999&&p[2]>=-.01101&&p[2]<=.05901));
 // Correct the old top-mounted placeholder to the underside shown on slide 5.
 ports.rotation.z=Math.PI;ports.position.y=.196;
 const locks=extract(battery,'rov_battery_locks',v=>v.every(p=>Math.abs(p[0])>.0549&&p[2]>-.0461&&p[2]<-.0259));
 locks.userData.referencePage=5;
 extract(battery,'rov_battery_pull_ring',v=>v.every(p=>p[2]<-.2879&&p[1]>.0979));
 extract(battery,'rov_battery_fastener',(v,mat)=>mat==='gray'&&v.every(p=>p[2]<-.2859));
 const materials={metal:new T.MeshStandardMaterial({color:0xa2acb0,metalness:.7,roughness:.3}),black:new T.MeshStandardMaterial({color:0x1e252c,metalness:.25,roughness:.4}),rubber:new T.MeshStandardMaterial({color:0x11171c,roughness:.8})};
 function mesh(parent,geometry,material,position){const m=new T.Mesh(geometry,materials[material].clone());m.position.set(...position);m.castShadow=m.receiveShadow=true;parent.add(m);return m;}
 function group(parent,name){const g=new T.Group();g.name=name;g.userData.accuracy='Training-diagram placement; dimensions unmeasured';parent.add(g);return g;}
 function cap(parent,position,radius,length,axis,material='metal'){
  const m=mesh(parent,new T.CylinderGeometry(radius,radius,length,24),material,position);m.quaternion.setFromUnitVectors(new T.Vector3(0,1,0),new T.Vector3(...axis));return m;
 }
 // Five capped peripheral interfaces. No individual UART/Ethernet pin positions are inferred.
 for(const [side,zs] of [[-1,[-.003,.024,.051]],[1,[.005,.044]]])for(const z of zs){
  const p=point([side*.052,.174,z]);cap(ports,p,.0065,.003,[side,0,0],'rubber');
 }
 const sd=group(body,'rov_service_sd');
 cap(sd,[0,.155,.038],.014,.013,[0,1,0],'black');
 cap(sd,[0,.162,.038],.012,.002,[0,1,0]);
 mesh(sd,new T.BoxGeometry(.012,.001,.002),'rubber',[0,.1633,.038]);
 const sensor=group(body,'rov_depth_temperature');
 cap(sensor,[.053,.072,-.015],.006,.014,[1,0,0]);
 cap(sensor,[.061,.072,-.015],.004,.003,[1,0,0],'black');
 const buckle=group(battery,'rov_buckle_bracket');
 for(const x of[-.012,.012])mesh(buckle,new T.BoxGeometry(.005,.014,.019),'metal',[x,.155,-.240]);
 mesh(buckle,new T.BoxGeometry(.029,.005,.019),'metal',[0,.164,-.240]);
 return model;
}

const common={subject:'rov',tag:'M2 PRO MAX · COMPONENT',logo:'./branding/chasing-logo.png',logoAlt:'CHASING',logoClass:'chasing-brand',useOwnerBounds:true,autoFrame:true,pickGeometry:true,showMarker:false};
const specs=[
 {id:'rov-camera',node:'rov_camera',label:'Camera',title:'Forward camera',purpose:'The pilot’s underwater view',showMarker:true,camera:'rov',view:[1,.22,.4],trainingSection:'media',page:4,rows:[['Position','Front · main cabin'],['Recording','Photos and video to microSD'],['Preview','Fixed forward camera']],note:'Camera view is a synthetic preview.',related:['rov-sd','rov-lights']},
 {id:'rov-thrusters',node:'rov_thrusters',label:'Thrusters',title:'Eight vectored thrusters',purpose:'Move, turn and hold orientation',effect:'thrusters',view:[1,.7,1],trainingSection:'shields',page:4,rows:[['Layout','8 vectored propellers'],['Control','Translation and rotation'],['Preview','Animated rotors']],note:'Preview rotation is illustrative; it is not calibrated RPM or thrust.',related:['rov-frame','rov-lights']},
 {id:'rov-lights',node:'rov_lights',label:'Floodlights',title:'Adjustable floodlights',purpose:'Illuminate the inspection area',showMarker:true,effect:'lights',view:[1,.8,1],trainingSection:'setup',page:13,rows:[['Output','2 × 4,000 lm'],['Mount','Folding arms'],['Adjustment','Loosen knob · position · tighten']],note:'Light cones are visual effects; brightness is not a photometric simulation.',related:['rov-camera','rov-thrusters']},
 {id:'rov-main-cabin',node:'rov_body',label:'Main cabin',title:'Main pressure cabin',purpose:'Forward housing and service interfaces',view:[1,.8,1],trainingSection:'structure',page:4,rows:[['Front','Camera'],['Top service access','microSD cap'],['Underside','Accessory interfaces']],note:'Housing geometry follows the reference model.',related:['rov-camera','rov-sd','rov-ports']},
 {id:'rov-battery',node:'rov_battery',label:'Battery',title:'Removable battery cabin',purpose:'Rear removable power module',showMarker:true,view:[-1,.65,1],trainingSection:'battery',page:29,rows:[['Location','Rear cabin'],['Retention','Twist locks and fastener'],['Removal','Pull ring · battery only']],note:'See the training sequence before battery removal.',related:['rov-battery-locks','rov-pull-ring','rov-fastener']},
 {id:'rov-frame',node:'rov_frame',label:'Hand grips',title:'Frame and hand grips',purpose:'Upper and lower handling grips',view:[1,.8,1],trainingSection:'setup',page:21,rows:[['Handling','Hold the side grips'],['Deployment','Lower gently into water'],['Thrusters','Unlock after immersion']],note:'Lifting and capture loads require a separately verified interface.',related:['rov-thrusters','rov-buckle']},
 {id:'rov-sd',node:'rov_service_sd',label:'microSD',title:'microSD service cap',purpose:'Access recorded media',view:[.3,1,.7],trainingSection:'media',page:27,rows:[['Location','Main cabin · upper access'],['Before opening','Power off · dry completely'],['Alternative','Download through GO1']],note:'Cap shape and position are reference estimates.',related:['rov-camera','rov-main-cabin']},
 {id:'rov-depth-temperature',node:'rov_depth_temperature',label:'Depth / temperature',title:'Depth and temperature sensor',purpose:'Measure the surrounding water',view:[-.4,-.3,-1],trainingSection:'structure',page:5,rows:[['Measurements','Depth · water temperature'],['Location','Near the cabin interface'],['Live telemetry','Not connected']],note:'Sensor placement is approximate; no readings are fabricated.',related:['rov-ports','rov-main-cabin']},
 {id:'rov-battery-locks',node:'rov_battery_locks',label:'Twist locks',title:'Battery twist-lock knobs',purpose:'Secure the removable battery',view:[-.2,.6,1],trainingSection:'battery',page:29,rows:[['Quantity','Two side knobs'],['Release','90° to shallow position'],['Assembly','90° to deep position']],note:'Follow the complete battery sequence in the training guide.',related:['rov-battery','rov-fastener']},
 {id:'rov-pull-ring',node:'rov_battery_pull_ring',label:'Pull ring',title:'Battery pull ring',purpose:'Withdraw the released battery',view:[-1,.5,.3],trainingSection:'battery',page:5,rows:[['Use','Battery removal only'],['Vehicle lifting','Not a lifting point']],note:'Release the battery retention first.',related:['rov-battery','rov-fastener']},
 {id:'rov-fastener',node:'rov_battery_fastener',label:'Battery fastener',title:'Battery fastener',purpose:'Retain the battery assembly',view:[-1,-.3,.5],trainingSection:'battery',page:30,rows:[['Assembly','Pressure plate and buckle'],['Confirmation','Lock the chute until it clicks']],note:'Simplified external detail; service clearances are unmeasured.',related:['rov-battery','rov-battery-locks']},
 {id:'rov-tether-socket',node:'rov_tether',label:'Tether / charge',title:'Tether and charging socket',purpose:'Rear connection to the surface system',view:[-1,.5,.4],trainingSection:'setup',page:15,rows:[['Operation','ROV tether → E-Reel → RC'],['Connection','Tighten the connector nut'],['Protection','Keep pins dry and sealed']],note:'Do not lift or pull the ROV by the tether.',related:['rov-buckle','rov-ereel','rov-controller']},
 {id:'rov-buckle',node:'rov_buckle_bracket',label:'Tether buckle',title:'Tether buckle bracket',purpose:'Relieve load at the tether connector',view:[-1,1,.4],trainingSection:'setup',page:15,rows:[['Connection','Attach the tether buckle'],['Role','Connector strain relief']],note:'Reference geometry; not a rated lifting point.',related:['rov-tether-socket','rov-frame']}
];

export function createRovAnatomy(model,existingPorts){
 const owners={},components=[];
 const definitions=[...specs,{...existingPorts,node:'rov_accessory_interfaces',showMarker:true,pickGeometry:true,view:[.2,-1,1],note:'Five capped interfaces under the main cabin. Positions are approximate; identify each port on the real ROV.',extra:existingPorts.extra,related:['rov-depth-temperature','rov-tether-socket','rov-ereel']}];
 for(const d of definitions){
  const node=model.getObjectByName(d.node);if(!node)continue;
  const bounds=getHardwareBounds(node);if(bounds.isEmpty())continue;
  node.updateWorldMatrix(true,true);bounds.applyMatrix4(node.matrixWorld.clone().invert());
  const owner='anatomy-'+d.id;owners[owner]=node;
  node.userData.inspect={at:bounds.getCenter(new T.Vector3()).toArray(),size:bounds.getSize(new T.Vector3()).toArray().map(v=>Math.max(.012,v))};
  const {node:unused,...entry}=d;
  components.push({...common,...entry,...node.userData.inspect,owner,useOwnerBounds:true,pickable:true,pickGeometry:true,autoFrame:true,sources:d.sources||[['Supplied CHASING training · slide '+d.page,pdf+'#page='+d.page]],distance:.5});
 }
 return{owners,components};
}
