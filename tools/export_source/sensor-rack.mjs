import * as T from 'three';

/**
 * Photo-referenced sensor stack, 159583–159586 (7 October 2026).
 * Attach `root` directly to the ME120 `antenna` node at identity transform.
 * Coordinates are that node's metres: X across the boat, Y up, +Z forward.
 * All mounting dimensions are visual estimates, not an OEM mounting drawing.
 */
export function createSensorRack() {
  const root = new T.Group();
  root.name = 'ME120_photo_sensor_stack';
  root.userData = {
    photoReferences: ['159583.heic', '159584.heic', '159585.heic', '159586.heic'],
    dimensionStatus: 'Photo-estimated geometry; exact sensor variants and mounting dimensions unverified',
    units: 'metres',
  };
  const owners = {};
  const mat = {
    anodized: new T.MeshStandardMaterial({color:0x171a1e, metalness:.64, roughness:.36}),
    edge: new T.MeshStandardMaterial({color:0x272c31, metalness:.7, roughness:.3}),
    camera: new T.MeshPhysicalMaterial({color:0x0d1117, metalness:.25, roughness:.17, clearcoat:.75, clearcoatRoughness:.08}),
    black: new T.MeshStandardMaterial({color:0x080c10, metalness:.12, roughness:.42}),
    rubber: new T.MeshStandardMaterial({color:0x131619, metalness:0, roughness:.81}),
    silver: new T.MeshStandardMaterial({color:0xb4bac0, metalness:.82, roughness:.29}),
    brushed: new T.MeshStandardMaterial({color:0x858e98, metalness:.76, roughness:.37}),
    optical: new T.MeshPhysicalMaterial({color:0x101720, metalness:.28, roughness:.095, clearcoat:1, clearcoatRoughness:.045}),
    lens: new T.MeshPhysicalMaterial({color:0x20392f, metalness:.3, roughness:.075, clearcoat:1, clearcoatRoughness:.04}),
    pupil: new T.MeshStandardMaterial({color:0x030608, metalness:.15, roughness:.15}),
    white: new T.MeshStandardMaterial({color:0xe2e4de, metalness:.04, roughness:.4}),
  };
  function mesh(parent, geometry, material, name) {
    const m = new T.Mesh(geometry, typeof material === 'string' ? mat[material] : material);
    m.name=name; m.castShadow=true; m.receiveShadow=true; parent.add(m); return m;
  }
  function box(parent,w,h,d,x,y,z,material,name) {
    const m=mesh(parent,new T.BoxGeometry(w,h,d),material,name);m.position.set(x,y,z);return m;
  }
  // Rounded XY outline extruded along Z, centred on all three axes.
  function rounded(parent,w,h,d,r,x,y,z,material,name) {
    r=Math.min(r,w*.49,h*.49);const s=new T.Shape(),a=-w/2,b=-h/2;
    s.moveTo(a+r,b);s.lineTo(a+w-r,b);s.quadraticCurveTo(a+w,b,a+w,b+r);
    s.lineTo(a+w,b+h-r);s.quadraticCurveTo(a+w,b+h,a+w-r,b+h);
    s.lineTo(a+r,b+h);s.quadraticCurveTo(a,b+h,a,b+h-r);
    s.lineTo(a,b+r);s.quadraticCurveTo(a,b,a+r,b);
    const geo=new T.ExtrudeGeometry(s,{depth:d,bevelEnabled:false,curveSegments:8});geo.translate(0,0,-d/2);
    const m=mesh(parent,geo,material,name);m.position.set(x,y,z);return m;
  }
  function cyl(parent,r,h,x,y,z,material,name,segments=48) {
    const m=mesh(parent,new T.CylinderGeometry(r,r,h,segments),material,name);m.position.set(x,y,z);return m;
  }
  function ring(parent,r,t,x,y,z,material,name,horizontal=false) {
    const m=mesh(parent,new T.TorusGeometry(r,t,8,48),material,name);if(horizontal)m.rotation.x=Math.PI/2;m.position.set(x,y,z);return m;
  }
  function frontCylinder(parent,r,d,x,y,z,material,name) {
    const m=cyl(parent,r,d,x,y,z,material,name);m.rotation.x=Math.PI/2;return m;
  }
  function group(id,x,y,z) {
    const g=new T.Group();g.name=id;g.position.set(x,y,z);g.userData.provisional=true;
    g.userData.photoReferences=root.userData.photoReferences;root.add(g);owners[id]=g;return g;
  }
  function cable(points,r,name) {
    const curve=new T.CatmullRomCurve3(points.map(p=>new T.Vector3(...p)));
    return mesh(cables,new T.TubeGeometry(curve,28,r,8,false),'rubber',name);
  }
  function label(parent,text,w,h,x,y,z,color='#edf0ef',back=null,name='wordmark') {
    if(typeof document==='undefined'||typeof document.createElement!=='function')return;
    const canvas=document.createElement('canvas');canvas.width=768;canvas.height=160;
    const ctx=canvas.getContext?.('2d');if(!ctx)return;
    ctx.clearRect(0,0,768,160);if(back){ctx.fillStyle=back;ctx.fillRect(0,0,768,160);}
    ctx.fillStyle=color;ctx.font='600 104px Arial, sans-serif';ctx.textAlign='center';ctx.textBaseline='middle';ctx.fillText(text,384,83);
    const map=new T.CanvasTexture(canvas);map.colorSpace=T.SRGBColorSpace;
    const m=mesh(parent,new T.PlaneGeometry(w,h),new T.MeshBasicMaterial({map,transparent:!back,depthWrite:false,toneMapped:false}),name);m.position.set(x,y,z);
  }
  function inspection(g,distance,view) {
    g.updateWorldMatrix(true,true);
    const inv=g.matrixWorld.clone().invert(),bounds=new T.Box3();
    g.traverse(o=>{if(!o.isMesh)return;o.geometry.computeBoundingBox();bounds.union(o.geometry.boundingBox.clone().applyMatrix4(new T.Matrix4().multiplyMatrices(inv,o.matrixWorld)));});
    g.userData.inspect={at:bounds.getCenter(new T.Vector3()).toArray(),size:bounds.getSize(new T.Vector3()).toArray(),distance,view};
    g.userData.dimensionStatus=root.userData.dimensionStatus;
  }

  // Existing crossbar top is Y=1.861. This base rests on it; no new mast.
  const mount=group('sensor-mount',0,1.866,.077);
  // Offset the mounting foot rearwards onto the existing crossbar, then carry
  // the camera posts forwards on a rigid plate. The optical stack is forward
  // of the diagonal braces; its base still has a continuous structural path.
  cyl(mount,.055,.010,0,0,-.062,'anodized','sensor_mount_round_base');
  cyl(mount,.048,.003,0,.0065,-.062,'edge','sensor_mount_upper_flange');
  box(mount,.096,.007,.091,0,.0105,-.031,'anodized','sensor_mount_forward_cantilever_plate');
  for(const x of[-.035,.035])
    cyl(mount,.0035,.003,x,.015,-.065,'silver','sensor_mount_crossbar_attachment_bolt',6);
  for(const x of[-.035,.035]) {
    cyl(mount,.0105,.033,x,.0305,0,'anodized','camera_support_post');
    cyl(mount,.013,.004,x,.015,0,'black','post_lower_collar');
    cyl(mount,.012,.004,x,.045,0,'edge','post_upper_collar');
    cyl(mount,.0033,.002,x,.048,.005,'silver','post_mount_bolt');
  }
  // Hood is open in front: the camera is recessed behind a narrow rectangular lip.
  box(mount,.208,.007,.077,0,.0435,.026,'anodized','camera_cradle_lower_tray');
  for(const x of[-.060,.060])
    box(mount,.024,.015,.027,x,.0545,.032,'rubber','camera_elastomer_support_pad');
  box(mount,.208,.006,.077,0,.113,.026,'anodized','camera_hood_roof');
  for(const x of[-.1005,.1005]) {
    box(mount,.007,.070,.077,x,.078,.026,'anodized','camera_hood_side_wall');
    box(mount,.009,.075,.005,x,.0775,.066,'edge','camera_hood_front_side_lip');
    // Silver pivot bolts are on the bracket's two outer side faces.
    const bolt=cyl(mount,.0048,.007,x+(x>0?.006:-.006),.077,.026,'silver','camera_side_hex_bolt',6);bolt.rotation.z=Math.PI/2;
    const washer=cyl(mount,.006,.0018,x+(x>0?.0038:-.0038),.077,.026,'brushed','camera_pivot_washer');washer.rotation.z=Math.PI/2;
  }
  box(mount,.208,.005,.005,0,.1145,.066,'edge','camera_hood_front_top_lip');
  box(mount,.208,.005,.005,0,.0415,.066,'edge','camera_hood_front_lower_lip');
  // Back plate and four real-looking fasteners support the upper LiDAR plate.
  box(mount,.188,.053,.004,0,.078,-.0105,'black','camera_hood_rear_wall');
  box(mount,.132,.006,.112,0,.119,.001,'anodized','lidar_mount_top_plate');
  for(const x of[-.054,.054])for(const z of[-.035,.037])
    cyl(mount,.003,.002,x,.123,z,'silver','lidar_plate_fastener',6);

  const camera=group('zed-camera',0,1.943,.109);
  rounded(camera,.175,.030,.033,.013,0,0,0,'camera','ZED_stereo_camera_body');
  rounded(camera,.171,.028,.0018,.012,0,0,.0173,'optical','ZED_glossy_front_window');
  rounded(camera,.050,.028,.002,.001,0,0,.0186,'black','ZED_central_nameplate');
  for(const [i,x] of[-.060,.060].entries()) {
    frontCylinder(camera,.012,.003,x,0,.018,'edge','ZED_lens_bezel_'+(i+1));
    ring(camera,.0095,.00085,x,0,.0198,'brushed','ZED_metal_lens_ring_'+(i+1));
    frontCylinder(camera,.0085,.0012,x,0,.0202,'lens','ZED_optical_lens_'+(i+1));
    frontCylinder(camera,.0033,.00035,x,0,.0209,'pupil','ZED_lens_aperture_'+(i+1));
    ring(camera,.0058,.00035,x,0,.0208,'edge','ZED_internal_lens_ring_'+(i+1));
  }
  label(camera,'ZED',.016,.0042,0,0,.0198,'#e5e8ed',null,'ZED_wordmark');
  // Visible rear connector and black strain-relief boot.
  box(camera,.018,.009,.007,.050,-.009,-.0185,'edge','ZED_rear_connector');
  const cameraPlug=frontCylinder(camera,.004,.012,.050,-.009,-.026,'rubber','ZED_cable_strain_relief');
  cameraPlug.rotation.x=Math.PI/2;

  const lidar=group('robosense-lidar',0,1.989,.072);
  cyl(lidar,.038,.012,0,.006,0,'black','RoboSense_lower_mount_foot');
  cyl(lidar,.0485,.004,0,.014,0,'anodized','RoboSense_lower_base');
  cyl(lidar,.049,.030,0,.027,0,'brushed','RoboSense_ribbed_base_band');
  for(let i=0;i<5;i++)ring(lidar,.0492,.0008,0,.017+i*.005,0,'silver','RoboSense_horizontal_cooling_rib_'+(i+1),true);
  cyl(lidar,.049,.063,0,.0735,0,'optical','RoboSense_black_optical_window');
  ring(lidar,.049,.00065,0,.042,0,'black','RoboSense_optical_lower_seal',true);
  cyl(lidar,.050,.007,0,.1085,0,'silver','RoboSense_silver_top_cap');
  cyl(lidar,.0478,.001,0,.1115,0,'brushed','RoboSense_top_cap_surface');
  ring(lidar,.0494,.00055,0,.111,0,'silver','RoboSense_top_cap_rim',true);
  // Small flat badge on the ribbed band reproduces the photographed wordmark.
  rounded(lidar,.045,.017,.002,.001,0,.027,.0492,'brushed','RoboSense_front_badge');
  label(lidar,'robosense',.041,.0095,0,.027,.0504,'#eceeeb',null,'RoboSense_wordmark');
  for(const x of[-.028,.028])for(const z of[-.021,.021])
    cyl(lidar,.0027,.002,x,.001,z,'silver','RoboSense_foot_fastener',6);
  const lidarPlug=frontCylinder(lidar,.0045,.011,.018,.011,-.043,'rubber','RoboSense_rear_connector');
  lidarPlug.rotation.x=Math.PI/2;

  // Central millimetre-wave radar: user correction, 8 October 2026.
  // Stable owner id front-cover retained; housing label BLD345 is not a verified model.
  const cover=group('front-cover',0,1.845,.181);
  rounded(cover,.120,.105,.032,.008,0,0,0,'white','mmwave_radar_housing');
  rounded(cover,.110,.095,.002,.006,0,0,.0165,'white','mmwave_radar_radome');
  box(cover,.081,.013,.021,0,-.047,-.019,'white','white_cover_lower_attachment');
  for(const x of[-.036,.036])frontCylinder(cover,.002,.002,x,-.041,.017,'brushed','white_cover_lower_fastener');

  const cables=new T.Group();cables.name='sensor_service_cable_loops';root.add(cables);
  cable([[.05,1.934,.078],[.061,1.914,.035],[.069,1.883,.029],[.10,1.843,.038],[.135,1.836,.019]],.0027,'ZED_rear_service_cable');
  cable([[.018,2.000,.024],[.060,1.990,.025],[.070,1.930,.018],[.071,1.872,.008],[.106,1.840,-.020]],.0031,'RoboSense_rear_service_cable');
  for(const g of[ mount,camera,lidar,cover ])inspection(g,g===mount?.60:.47,[1,.32,.55]);

  let exploded=0;
  const assembled={mount:mount.position.clone(),camera:camera.position.clone(),lidar:lidar.position.clone()};
  function updateExplode(fraction=0) {
    exploded=T.MathUtils.clamp(Number.isFinite(fraction)?fraction:0,0,1);
    mount.position.copy(assembled.mount).y+=.035*exploded;
    camera.position.copy(assembled.camera).y+=.135*exploded;
    lidar.position.copy(assembled.lidar).y+=.270*exploded;
    cables.visible=exploded<.001;
  }
  function info() {
    let meshes=0;root.traverse(o=>{if(o.isMesh)meshes++;});
    return {
      owners:Object.keys(owners),meshes,exploded,
      frame:'ME120 antenna local; X across, Y up, +Z forward; metres',
      status:root.userData.dimensionStatus,
      camera:{body:[.175,.030,.033],lensBaseline:.120,position:camera.position.toArray()},
      lidar:{diameter:.100,height:.112,position:lidar.position.toArray()},
      bracket:{hoodWidth:.208,envelope:mount.userData.inspect.size,position:mount.position.toArray()},
      photoReferences:root.userData.photoReferences,
    };
  }
  return {root,owners,updateExplode,info};
}
