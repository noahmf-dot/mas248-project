"""MAS248 student image-to-TCP target converter (no A4/paper model).

An IMAGE frame is located at the centre of the full raster image, with its
+X axis right, +Y axis up and +Z perpendicular to the drawing plane.
The image width in metres is an argument, and height follows its aspect ratio.
World pose coordinates are calculated using an adjustable centre and yaw.
Contour boundaries are teaching examples, NOT automatically pen centrelines.

Outputs: image-stage PNGs, browser slideshow, PNG preview, targets JSON.
Optional RViz marker publisher (separate process, Ctrl+C to stop).
"""
import argparse
import json
import math
import os
from pathlib import Path
import cv2
import numpy as np

VERSION = '0.4.0'
DEFAULT_IMAGE = 'pumpkin_outline.jpg'
STAGE_NAMES = (
    ('01_original.png', 'Original image'),
    ('02_grayscale.png', 'Grayscale'),
    ('03_binary_mask.png', 'Binary mask (dark pixels)'),
    ('04_contours.png', 'Detected contour boundaries'),
    ('05_sampled_points.png', 'Sampled contour targets'),
    ('06_tcp_target_map.png', 'Image-centred TCP targets'),
)


def resolve_image(image_file=None):
    """Resolve an absolute path, relative file or installed images/ basename."""
    name = str(image_file) if image_file else DEFAULT_IMAGE
    candidate = Path(name).expanduser()
    if candidate.is_file():
        return candidate.resolve()
    if candidate.is_absolute() or candidate.parent != Path('.'):
        raise FileNotFoundError(f'Cannot read image: {name}')
    options = []
    try:
        from ament_index_python.packages import get_package_share_directory
        options.append(Path(get_package_share_directory('mas248_ur5e_demos'))/'images'/name)
    except (ImportError, LookupError):
        pass
    options += [Path(__file__).resolve().parent.parent/'images'/name,
                Path.home()/'mas248_ws/src/mas248_ur5e_demos/images'/name]
    for p in options:
        if p.is_file():
            return p.resolve()
    raise FileNotFoundError(f'Image {name!r} not found. Use an absolute path or put it in the package images/ directory.')


def _sample(contour, spacing_px, max_points):
    p=np.asarray(contour,dtype=float).reshape(-1,2)
    if len(p)<2: return []
    if np.linalg.norm(p[-1]-p[0])>1.e-2: p=np.vstack([p,p[0]])
    length=np.linalg.norm(np.diff(p,axis=0),axis=1)
    arclength=np.concatenate(([0.],np.cumsum(length)))
    if arclength[-1]<1.e-9: return []
    n=max(3,min(max_points,int(math.ceil(arclength[-1]/spacing_px))+1))
    return [[float(np.interp(t,arclength,p[:,j])) for j in (0,1)]
            for t in np.linspace(0,arclength[-1],n)]


def _write_stages(images, directory):
    folder=Path(directory).expanduser()
    folder.mkdir(parents=True,exist_ok=True)
    result=[]
    for (filename, label), image in zip(STAGE_NAMES,images):
        src=cv2.cvtColor(image,cv2.COLOR_GRAY2BGR) if image.ndim==2 else image.copy()
        header=max(40,int(src.shape[0]*0.075))
        titled=cv2.copyMakeBorder(src,header,0,0,0,cv2.BORDER_CONSTANT,value=(255,255,255))
        cv2.putText(titled,label,(12,header-12),cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,(10,10,10),2,cv2.LINE_AA)
        path=folder/filename
        if not cv2.imwrite(str(path),titled): raise OSError(f'Cannot write {path}')
        result.append(str(path.resolve()))
    return result


def convert_image(image_file=None, *, output='/tmp/mas248_targets.json',
                  preview='/tmp/mas248_contours.png',
                  stages_dir=None, show_stages=False,
                  width_m=0.18, center_x=0.55, center_y=0., center_z=0.46,
                  yaw_deg=0., tcp_offset_mm=0.,
                  spacing_px=5., max_points=100, max_contours=8, min_area=15.):
    """Convert selected contour BOUNDARIES to IMAGE and WORLD pose coordinates.

    width_m is the full image width, not a bounding box around selected ink.
    center_* is the centre of the FULL image in the world frame (metres).
    Image-local (0,0,0) is the image centre, +X right, +Y up.
    TCP +Z points down toward drawing plane; optional positive offset raises it.
    No paper dimensions, page margins or A4 assumptions are used.
    """
    if not all(map(math.isfinite,(width_m,center_x,center_y,center_z,yaw_deg,tcp_offset_mm))):
        raise ValueError('Image width, centre and orientation must be finite')
    if width_m<=0: raise ValueError('width_m must be >0 metres')
    if spacing_px<=0 or max_points<3 or max_contours<1 or min_area<0:
        raise ValueError('Require spacing_px>0, max_points>=3, max_contours>=1, min_area>=0')
    stages_dir = stages_dir or str(Path.home()/'MAS248_Image_Stages')
    image_path=resolve_image(image_file)
    original=cv2.imread(str(image_path),cv2.IMREAD_COLOR)
    if original is None: raise FileNotFoundError('Cannot read image: '+str(image_path))
    gray=cv2.cvtColor(original,cv2.COLOR_BGR2GRAY)
    h_px,w_px=gray.shape[:2]
    if w_px<2 or h_px<2: raise ValueError('Image must contain at least 2x2 pixels')
    _,binary=cv2.threshold(gray,170,255,cv2.THRESH_BINARY_INV)
    contours,_=cv2.findContours(binary,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    contours=sorted((c for c in contours if cv2.contourArea(c)>=min_area),
                    key=cv2.contourArea,reverse=True)[:max_contours]
    if not contours: raise ValueError('No dark contours detected (black on white expected)')
    contour_preview=original.copy()
    cv2.drawContours(contour_preview,contours,-1,(0,155,0),2,cv2.LINE_AA)
    sample_preview=contour_preview.copy()
    sampled=[]
    for c in contours:
        points=_sample(c,spacing_px,max_points)
        if len(points)<3: continue
        sampled.append(points)
        for u,v in points:
            cv2.circle(sample_preview,(round(u),round(v)),2,(0,0,220),-1,cv2.LINE_AA)
    if not sampled: raise ValueError('Contours did not yield usable waypoints')

    # Image coordinates use pixel centres; thus the extreme pixel centres span width_m.
    scale=width_m/(w_px-1)
    image_height_m=scale*(h_px-1)
    u_mid,v_mid=(w_px-1)/2.,(h_px-1)/2.
    yaw=math.radians(yaw_deg)
    co,si=math.cos(yaw),math.sin(yaw)
    # Local 180-degree TCP X rotation turns +Z toward the surface.
    # Compose Rz(yaw)*Rx(pi) in XYZW quaternion order.
    tcp_q_world=[round(math.cos(yaw/2),9),round(math.sin(yaw/2),9),0.,0.]
    stroke_data=[]
    target_map=np.full_like(original,255)
    palette=[(40,100,230),(180,80,20),(45,150,45),(150,35,155),(80,150,190)]
    for index,px in enumerate(sampled):
        color=palette[index%len(palette)]
        pixels=np.rint(np.asarray(px)).astype(np.int32)
        cv2.polylines(target_map,[pixels.reshape(-1,1,2)],False,color,2,cv2.LINE_AA)
        poses_local=[]; poses_world=[]
        for u,v in px:
            local_x=(u-u_mid)*scale
            local_y=(v_mid-v)*scale
            local_z=tcp_offset_mm/1000.
            world_x=center_x+co*local_x-si*local_y
            world_y=center_y+si*local_x+co*local_y
            def r7(x): return round(float(x),7)
            poses_local.append({'x':r7(local_x),'y':r7(local_y),'z':r7(local_z),
                                'qx':1.,'qy':0.,'qz':0.,'qw':0.})
            poses_world.append({'x':r7(world_x),'y':r7(world_y),'z':r7(center_z+local_z),
                                'qx':tcp_q_world[0],'qy':tcp_q_world[1],
                                'qz':tcp_q_world[2],'qw':tcp_q_world[3]})
            cv2.circle(target_map,(round(u),round(v)),2,color,-1,cv2.LINE_AA)
        cv2.circle(target_map,tuple(pixels[0]),5,(0,190,0),-1,cv2.LINE_AA)
        stroke_data.append({'pixels':[[round(float(u),2),round(float(v),2)] for u,v in px],
                            'poses_image':poses_local,'poses':poses_world})
    cv2.putText(target_map,'image +X right, +Y up',(8,max(20,h_px-12)),
                cv2.FONT_HERSHEY_SIMPLEX,.55,(0,0,0),2,cv2.LINE_AA)
    stages=_write_stages((original,gray,binary,contour_preview,sample_preview,target_map),stages_dir)
    payload={
        'source_image':str(image_path),
        'image_size_px':[w_px,h_px],
        'drawing_frame':'image_center',
        'frame_to_world':{
            'parent':'world','child':'image_center',
            'xyz':[center_x,center_y,center_z],
            'quaternion_xyzw':[0.,0.,math.sin(yaw/2),math.cos(yaw/2)],
        },
        'image':{
            'width_m':width_m,'height_m':image_height_m,
            'center_world_xyz_m':[center_x,center_y,center_z],
            'yaw_deg':yaw_deg,'tcp_offset_m':tcp_offset_mm/1000.,
            'image_origin':'center',
        },
        'metres_per_pixel':scale,
        'stage_images':stages,
        'orientation_xyzw_world':tcp_q_world,
        'note':'Contours are image boundaries, not pen stroke centrelines. Targets are NOT validated for robot reachability or safety.',
        'strokes':stroke_data,
    }
    dst=Path(output).expanduser(); dst.parent.mkdir(parents=True,exist_ok=True)
    dst.write_text(json.dumps(payload,indent=2),encoding='utf-8')
    png=Path(preview).expanduser();png.parent.mkdir(parents=True,exist_ok=True)
    if not cv2.imwrite(str(png),target_map): raise OSError('Cannot write '+str(png))
    from .stage_viewer import write_viewer,open_viewer
    viewer=write_viewer(stages,[title for _,title in STAGE_NAMES],stages_dir)
    if show_stages:
        open_viewer(viewer)
        print('Browser stages: '+str(viewer))
    return len(stroke_data),sum(len(s['poses']) for s in stroke_data)


def get_parser():
    p=argparse.ArgumentParser(description='MAS248 student: image size + image centre -> TCP poses (no paper)')
    p.add_argument('--version',action='version',version='MAS248 student image_to_targets 0.4.0')
    p.add_argument('image',nargs='?',default=None,help=f'Full path or filename in images/ (default {DEFAULT_IMAGE})')
    p.add_argument('--width-m',type=float,default=0.18,help='Physical FULL image width in metres (default 0.18)')
    p.add_argument('--center-x',type=float,default=0.55,help='WORLD X of image centre in metres')
    p.add_argument('--center-y',type=float,default=0.,help='WORLD Y of image centre in metres')
    p.add_argument('--center-z',type=float,default=0.46,help='WORLD Z of image drawing plane in metres')
    p.add_argument('--yaw-deg',type=float,default=0.,help='Rotate image frame around world +Z')
    p.add_argument('--tcp-offset-mm',type=float,default=0.,help='TCP target height above image plane in millimetres')
    p.add_argument('--output',default='/tmp/mas248_targets.json')
    p.add_argument('--preview',default='/tmp/mas248_contours.png')
    p.add_argument('--stages-dir',default=str(Path.home()/'MAS248_Image_Stages'))
    p.add_argument('--open-stages',action='store_true',help='Open HTML viewer in browser, never an OpenCV popup')
    p.add_argument('--no-popup',action='store_true',help='Compatibility flag: do not open browser')
    p.add_argument('--rviz',action='store_true',help='Publish image bounds, paths, dots and TCP axes in RViz until Ctrl+C')
    p.add_argument('--axis-every',type=int,default=10)
    p.add_argument('--axis-length-mm',type=float,default=18.)
    p.add_argument('--axis-lift-mm',type=float,default=0.)
    p.add_argument('--no-orientations',action='store_true')
    p.add_argument('--spacing-px',type=float,default=5.)
    p.add_argument('--max-points',type=int,default=100)
    p.add_argument('--max-contours',type=int,default=8)
    p.add_argument('--min-area',type=float,default=15.)
    return p


def main():
    args=get_parser().parse_args()
    count,pts=convert_image(args.image,output=args.output,preview=args.preview,
        stages_dir=args.stages_dir,show_stages=args.open_stages and not args.no_popup,
        width_m=args.width_m,center_x=args.center_x,center_y=args.center_y,
        center_z=args.center_z,yaw_deg=args.yaw_deg,tcp_offset_mm=args.tcp_offset_mm,
        spacing_px=args.spacing_px,max_points=args.max_points,
        max_contours=args.max_contours,min_area=args.min_area)
    print(f'Created {count} contour boundaries / {pts} TCP poses: {args.output}',flush=True)
    print('Stage viewer: '+str(Path(args.stages_dir).expanduser()/'index.html'),flush=True)
    print('No A4 paper assumptions. Full input image width: '+str(args.width_m)+' m',flush=True)
    if args.rviz:
        import sys
        argv=[sys.executable,'-m','scripts.show_targets_rviz','--json',str(Path(args.output).expanduser().resolve()),
              '--axis-every',str(args.axis_every),'--axis-length-mm',str(args.axis_length_mm),
              '--axis-lift-mm',str(args.axis_lift_mm)]
        if args.no_orientations:argv.append('--no-orientations')
        print('Publishing image-frame markers; Ctrl+C to stop.',flush=True)
        os.execv(sys.executable,argv)

if __name__=='__main__':
    main()
