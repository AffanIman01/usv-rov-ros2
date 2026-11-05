import * as T from 'three';

// Hardware framing must never grow to include the illustrative floodlight cones.
// Invisible meshes still contribute to THREE.Box3.setFromObject(), so visibility
// alone is not a suitable filter. Physical lamps and moving rotors remain included.
export function getHardwareBounds(root, bounds=new T.Box3()) {
 bounds.makeEmpty();
 if(!root)return bounds;
 root.updateWorldMatrix(true,true);
 root.traverse(node=>{
  if(!node.isMesh||!node.geometry)return;
  for(let parent=node;parent;parent=parent.parent){
   if(parent.name==='ROV_light_beam_visual'||parent.userData?.excludeFromFraming)return;
   if(parent===root)break;
  }
  if(!node.geometry.boundingBox)node.geometry.computeBoundingBox();
  if(node.geometry.boundingBox)bounds.union(node.geometry.boundingBox.clone().applyMatrix4(node.matrixWorld));
 });
 return bounds;
}

// Fit all eight hardware-bound corners into an unobscured pixel rectangle.
// Callers may measure their live title/tools and pass the corresponding insets.
export function fitModelFrame({bounds,direction,camera,width,height,safeArea={},margin=1.06}) {
 if(bounds.isEmpty()||!(width>0&&height>0))return null;
 const insets={left:24,right:24,top:96,bottom:width<=900?82:32,...safeArea};
 const region={
  x0:-1+2*Math.max(0,insets.left)/width,
  x1:1-2*Math.max(0,insets.right)/width,
  y0:-1+2*Math.max(0,insets.bottom)/height,
  y1:1-2*Math.max(0,insets.top)/height
 };
 if(region.x1<=region.x0||region.y1<=region.y0)return null;
 const d=direction.clone().normalize();
 if(d.lengthSq()<.5)return null;
 const referenceUp=Math.abs(d.y)>.999?new T.Vector3(0,0,1):new T.Vector3(0,1,0);
 const right=new T.Vector3().crossVectors(referenceUp,d).normalize();
 const up=new T.Vector3().crossVectors(d,right).normalize();
 const center=bounds.getCenter(new T.Vector3());
 const {x0,x1,y0,y1}=region,cx=(x0+x1)/2,cy=(y0+y1)/2;
 const ty=Math.tan(T.MathUtils.degToRad((camera.getEffectiveFOV?.()??camera.fov)/2)),tx=ty*camera.aspect;
 let distance=.2;
 for(const x of [bounds.min.x,bounds.max.x])for(const y of [bounds.min.y,bounds.max.y])for(const z of [bounds.min.z,bounds.max.z]){
  const delta=new T.Vector3(x,y,z).sub(center),px=delta.dot(right),py=delta.dot(up),pz=delta.dot(d);
  distance=Math.max(distance,pz+camera.near+.1,
   (px+x1*pz*tx)/((x1-cx)*tx),(-px-x0*pz*tx)/((cx-x0)*tx),
   (py+y1*pz*ty)/((y1-cy)*ty),(-py-y0*pz*ty)/((cy-y0)*ty));
 }
 distance*=Math.max(1,margin);
 const target=center.clone().addScaledVector(right,-cx*distance*tx).addScaledVector(up,-cy*distance*ty);
 return {target,position:target.clone().addScaledVector(d,distance),distance,region};
}
