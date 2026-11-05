"""Render the exported COLLADA geometry for inspection (not a Gazebo screenshot)."""
from pathlib import Path
import json
import xml.etree.ElementTree as ET
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

ROOT=Path(__file__).resolve().parents[1]
PKG=ROOT/'usv_rov_description'
manifest=json.loads((PKG/'config/model_manifest.json').read_text())
ns={'c':'http://www.collada.org/2005/11/COLLADASchema'}
fig=plt.figure(figsize=(15,10),facecolor='#edf1f4')
titles={'me120':'ME120 + sensor rack','m2_pro_max':'CHASING M2 Pro Max','e_reel':'CHASING E-Reel','starlink_mini':'Starlink Mini concept','compute_enclosure':'Compute enclosure concept','recovery_concept':'Recovery concept parts'}
for index,model in enumerate(manifest['models']):
 ax=fig.add_subplot(2,3,index+1,projection='3d',computed_zorder=False)
 ax.set_facecolor('#edf1f4')
 all_faces=[];all_colors=[]
 for part in model['parts']:
  tree=ET.parse(PKG/'meshes'/model['name']/(part['name']+'.dae'))
  for geom in tree.findall('.//c:library_geometries/c:geometry',ns):
   gid=geom.attrib['id'];mid='m'+gid[1:]
   pos=np.fromstring(geom.find(f'.//c:float_array[@id="{gid}_positions_array"]',ns).text,sep=' ').reshape(-1,3)
   pos+=np.asarray(part['origin'])
   packed=np.fromstring(geom.find('.//c:triangles/c:p',ns).text,sep=' ',dtype=int).reshape(-1,2)[:,0].reshape(-1,3)
   faces=pos[packed]
   col=np.fromstring(tree.find(f'.//c:effect[@id="{mid}_effect"]//c:diffuse/c:color',ns).text,sep=' ')
   normal=np.cross(faces[:,1]-faces[:,0],faces[:,2]-faces[:,0]);length=np.linalg.norm(normal,axis=1)
   normal/=np.maximum(length[:,None],1e-15)
   light=np.array([.5,-.6,.8]);light/=np.linalg.norm(light)
   shade=.48+.52*np.abs(normal@light)
   # Convert linear diffuse RGB to display sRGB for this preview.
   rgb=np.where(col[:3]<=.0031308,col[:3]*12.92,1.055*col[:3]**(1/2.4)-.055)
   colors=np.clip(shade[:,None]*rgb[None,:],0,1)
   all_faces.append(faces);all_colors.append(colors)
 polys=Poly3DCollection(np.concatenate(all_faces),facecolors=np.concatenate(all_colors),edgecolors='none',linewidths=0,zsort='average')
 ax.add_collection3d(polys)
 lo=np.array(model['bounds']['min']);hi=np.array(model['bounds']['max']);size=hi-lo;center=(lo+hi)/2
 radius=max(size)*.55
 ax.set_xlim(center[0]-radius,center[0]+radius)
 ax.set_ylim(center[1]-radius,center[1]+radius)
 ax.set_zlim(center[2]-radius,center[2]+radius)
 ax.set_box_aspect((1,1,1));ax.view_init(elev=22,azim=-50)
 ax.set_axis_off();ax.set_title(titles[model['name']],fontsize=13,fontweight='bold',color='#173448',pad=-5)
 ax.text2D(.5,.03,f"{model['triangles']:,} triangles · visual reference",transform=ax.transAxes,ha='center',fontsize=9,color='#516776')
fig.suptitle('USV–ROV 10k  /  Exported model collection',x=.055,ha='left',y=.97,fontsize=22,fontweight='bold',color='#173448')
fig.text(.055,.922,'Converted from your site to ROS 2 meshes • Geometry preview only; physics and sensor output are not implemented.',fontsize=10,color='#516776')
fig.subplots_adjust(left=.02,right=.98,top=.88,bottom=.035,hspace=.05,wspace=.01)
fig.savefig(ROOT/'MODEL_PREVIEW.png',dpi=140,facecolor=fig.get_facecolor())
print(ROOT/'MODEL_PREVIEW.png')
