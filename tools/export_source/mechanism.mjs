/**
 * Static extraction of the site's buildMagnetic geometry.
 * Source: dist/scene.js, Site commit 9edda0a7a7fcd8700558ed00c44bbd2e29d9196e.
 * Units: metres. Scene axes: +X forward, +Y up, +Z lateral (source scene convention).
 * Geometry is a conceptual display, not validated hardware or load-bearing CAD.
 * This preserves the source's positive-lock pin / latch / keeper concepts.
 * Those details are present in the source despite the "magnetic" scene name.
 * They are NOT confirmation of the latest selected recovery hardware.
 * No OEM model copies, flow markers, timeline, collision or force model included.
 * head and yoke share the source carrier-local construction datum. Their final
 * world placement is left to the importing assembly; tetherGuide is boat-local.
 */
import * as T from 'three';
export function makeMechanism() {
 const P=(x,y,z)=>new T.Vector3(x,z,y);
 const C={metal:0xa4b7c6,black:0x15232c,orange:0xff812e,cyan:0x65d9e7,gold:0xe9c987,pad:0x263640,light:0xd5f4f5};
 const mats={};for(const [k,v]of Object.entries(C))mats[k]=new T.MeshStandardMaterial({color:v,metalness:['metal','cyan','gold'].includes(k)?.65:.15,roughness:k==='pad'?.9:.37});
 const carrier=new T.Group();
 let mountPlate,spider,magnetGroup,epmFace,shoe,tetherHardware;
 function mesh(geo,mat,parent,name){const m=new T.Mesh(geo,mats[mat]||mat);m.castShadow=true;m.receiveShadow=true;if(name)m.name=name;parent.add(m);return m;}
 function box(g,x,y,z,dx,dy,dz,mat='metal',name=''){const m=mesh(new T.BoxGeometry(dx,dz,dy),mat,g,name);m.position.copy(P(x,y,z));return m;}
 function cyl(g,x,y,z,r,h,mat='metal',axis='z',name=''){const m=mesh(new T.CylinderGeometry(r,r,h,32),mat,g,name);m.position.copy(P(x,y,z));if(axis==='x')m.rotation.z=Math.PI/2;if(axis==='y')m.rotation.x=Math.PI/2;return m;}
 function beam(g,a,b,r=.018,mat='metal',name=''){const m=mesh(new T.CylinderGeometry(r,r,1,12),mat,g,name);setBeam(m,a,b);return m;}
 function setBeam(m,a,b){const p=P(...a),q=P(...b),d=q.clone().sub(p);m.position.copy(p.add(q).multiplyScalar(.5));m.scale.set(1,d.length(),1);m.quaternion.setFromUnitVectors(new T.Vector3(0,1,0),d.normalize());}
 function annulus(parent,r,R,h,z,name){const shape=new T.Shape();shape.absarc(0,0,R,0,Math.PI*2,false);const hole=new T.Path();hole.absarc(0,0,r,0,Math.PI*2,true);shape.holes.push(hole);const m=mesh(new T.ExtrudeGeometry(shape,{depth:h,bevelEnabled:false,curveSegments:64}),'gold',parent,name);m.rotation.x=-Math.PI/2;m.position.y=z-h/2;return m;}
  carrier.name='compact_EPM_head_on_OEM_round_mount';
  mountPlate=new T.Group();mountPlate.name='blank_adapter';carrier.add(mountPlate);annulus(mountPlate,.037,.085,.008,.454,'blank_adapter_NO_MOUNT_HOLES');
  spider=new T.Group();spider.name='head_structural_spine';carrier.add(spider);
  box(spider,0,0,.425,.27,.05,.035,'metal','rigid_head_spine');box(spider,0,0,.430,.050,.32,.022,'metal','crossarm_to_lock_housings');
  for(const y of[-.145,.145]){beam(spider,[0,y,.337],[0,y,.430],.015,'metal','pin_housing_support');box(spider,0,y,.337,.065,.09,.052,'black','sealed_latch_housing_CONCEPT');box(spider,.039,y,.345,.014,.025,.022,'light','latch_sensor');}
  for(const y of[-.050,.050])beam(spider,[0,y,.410],[0,y,.452],.013,'metal');
  magnetGroup=new T.Group();magnetGroup.name='sealed_pulse_switched_EPM_CONCEPT';carrier.add(magnetGroup);
  cyl(magnetGroup,0,0,.381,.060,.046,'black','z','EPM_housing_UNSPECIFIED');
  epmFace=cyl(magnetGroup,0,0,.362,.062,.008,'gold','z','EPM_pole_face');epmFace.material=epmFace.material.clone();
  box(magnetGroup,.12,0,.418,.060,.080,.038,'black','pulse_driver_enclosure_CONCEPT');
  for(const x of[-.100,.100]){cyl(spider,x,0,.3615,.010,.113,'metal','z','guide_pin_shear_key');const cone=mesh(new T.ConeGeometry(.010,.022,24),'gold',spider,'tapered_guide_tip');cone.rotation.z=Math.PI;cone.position.copy(P(x,0,.300));}
  for(const sign of[-1,1]){const g=new T.Group();g.name='spring_engaged_pin_'+sign;g.position.copy(P(0,sign*.103,.337));carrier.add(g);cyl(g,0,0,0,.006,.085,'gold','y','positive_lock_pin');}
  shoe=new T.Group();shoe.name='ROV_load_spreading_yoke_stays_with_vehicle';
  for(const y of[-.105,.105])box(shoe,0,y,.058,.51,.025,.02,'cyan','underside_sled_runner');
  for(const x of[-.215,.215]){box(shoe,x,0,.072,.042,.365,.020,'cyan','load_spreader');for(const y of[-.105,.105])box(shoe,x,y,.085,.062,.05,.016,'pad','body_pad_CONTACT_UNVERIFIED');}
  for(const x of[-.215,.215])for(const y of[-.172,.172])beam(shoe,[x,y,.065],[x,y,.320],.010,'cyan','yoke_post');
  for(const y of[-.172,.172]){beam(shoe,[-.215,y,.320],[.215,y,.320],.010,'cyan');beam(shoe,[0,y,.320],[0,0,.335],.012,'cyan','armature_load_spreader');}
  for(const [x,w]of[[-.124,.022],[0,.174],[.124,.022]])box(shoe,x,0,.325,w,.050,.018,'metal','guide_receiver_crossbar');
  for(const x of[-.100,.100])for(const y of[-.019,.019])box(shoe,x,y,.325,.026,.012,.018,'metal','bore_side_bridge');
  cyl(shoe,0,0,.342,.040,.018,'metal','z','armature_load_boss');
  cyl(shoe,0,0,.352,.075,.012,'metal','z','ferromagnetic_strike_plate');
  for(const x of[-.100,.100]){const g=mesh(new T.TorusGeometry(.016,.003,10,32),'gold',shoe,'guide_socket');g.rotation.x=Math.PI/2;g.position.copy(P(x,0,.332));}
  for(const sign of[-1,1]){const lug=mesh(new T.TorusGeometry(.012,.005,12,32),'gold',shoe,'cross_pin_keeper_lug');lug.position.copy(P(0,sign*.103,.337));}
  tetherHardware=new T.Group();tetherHardware.name='separate_stern_tether_guide';
  for(const y of[-.14,.14]){box(tetherHardware,-.88,y,.38,.12,.07,.03,'metal','deck_bracket_LOCATION_UNVERIFIED');beam(tetherHardware,[-.88,y,.40],[-1.46,y*.65,.38],.015,'cyan','aft_guide_boom');}
  box(tetherHardware,-1.47,0,.37,.05,.25,.04,'metal','fairlead_crosshead');
  const wheel=mesh(new T.TorusGeometry(.080,.010,12,48),'gold',tetherHardware,'stern_captive_sheave_RADIUS_UNVERIFIED');wheel.position.copy(P(-1.48,0,.34));
  for(const y of[-.035,.035]){const guard=mesh(new T.TorusGeometry(.095,.008,10,48),'metal',tetherHardware,'captive_sheave_cheek');guard.position.copy(P(-1.48,y,.34));}
  cyl(tetherHardware,-1.48,0,.34,.009,.11,'metal','y','fairlead_axle');
  box(tetherHardware,-1.19,0,.50,.09,.07,.05,'gold','tether_tension_sensor');
  beam(tetherHardware,[-1.19,0,.41],[-1.19,0,.50],.010,'metal','load_sensor_bracket');
  for(const x of[-.98,-1.10])cyl(tetherHardware,x,0,.52,.030,.065,'pad','y','dancer_roller');

 const metadata={
   units:'metres', axes:'+X forward, +Y up, +Z lateral (source scene convention)',
   source:'dist/scene.js buildMagnetic',
   accuracy:'Conceptual site geometry; dimensions and load capacities unverified',
   retainedLegacyDetails:'Positive-lock pins, latch housings, keeper lugs and guide pieces are preserved from source; hardware choice is not established by this export',
   placement:'head and yoke in carrier-local construction datum; tetherGuide in boat-local scene datum',
   omitted:['OEM context copies','ROV duplicate','E-Reel duplicate','flow markers','timeline animation'],
 };
 for(const root of [carrier,shoe,tetherHardware]) {
   root.userData.transfer=metadata;
   root.traverse(node=>{
     if(/lock|latch|keeper|spring_engaged/i.test(node.name)) node.userData.unverifiedLegacyMechanicalConcept=true;
   });
   root.updateMatrixWorld(true);
 }
 return {head:carrier,yoke:shoe,tetherGuide:tetherHardware,metadata};
}
