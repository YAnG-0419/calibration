#!/usr/bin/env python3
"""Observe a fixed ChArUco task board and map two separate workcells via it.

No robot control. A_T_B maps coordinates from B to A, using column vectors.
Board origin is the top-left outer board corner in board_reference.png;
X right, Y down the printed board, Z into the board (not gravity-up).
"""
import argparse
import base64
import hashlib
import json
import time
from pathlib import Path
from datetime import datetime, timezone
import cv2
import numpy as np

HERE=Path(__file__).resolve().parent
DEFAULT_BOARD=HERE/'config/board.json'


def read_json(path):
    return json.loads(Path(path).read_text())


def write_json(path, value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')


def fingerprint(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':')).encode()).hexdigest()


def validate_transform(value):
    t=np.asarray(value,dtype=float)
    if t.shape!=(4,4) or not np.isfinite(t).all() or not np.allclose(t[3],[0,0,0,1],atol=1e-8):
        raise ValueError('Expected a finite 4x4 homogeneous transform')
    r=t[:3,:3]
    if not np.allclose(r.T@r,np.eye(3),atol=1e-6) or not np.isclose(np.linalg.det(r),1,atol=1e-6):
        raise ValueError('Rotation is not a proper orthogonal matrix')
    return t


def inverse(t):
    t=validate_transform(t);out=np.eye(4);out[:3,:3]=t[:3,:3].T;out[:3,3]=-out[:3,:3]@t[:3,3]
    return out


def rotation_degrees(a,b):
    return float(np.degrees(np.arccos(np.clip((np.trace(a@b.T)-1)/2,-1,1))))


def mean_pose(poses):
    poses=np.asarray(poses);u,_,vh=np.linalg.svd(poses[:,:3,:3].sum(0));fix=np.eye(3);fix[2,2]=np.linalg.det(u@vh)
    t=np.eye(4);t[:3,:3]=u@fix@vh;t[:3,3]=np.median(poses[:,:3,3],axis=0)
    return t


def load_intrinsics(path):
    path=Path(path)
    if path.suffix.lower() in ['.yaml','.yml']:
        import yaml
        doc=yaml.safe_load(path.read_text())
    else:doc=read_json(path)
    d=doc.get('camera_intrinsics',doc)
    def array(value):
        if isinstance(value,dict):value=value['data']
        return np.asarray(value,dtype=float)
    k=array(d.get('K',d.get('k',d.get('camera_matrix')))).reshape(3,3)
    distortion=d.get('distortion_coefficients',d.get('D',d.get('d',[])))
    dist=array(distortion).reshape(-1)
    model=d.get('distortion_model',d.get('driver_distortion_model','plumb_bob'))
    if model not in ['plumb_bob','rational_polynomial','opencv','none']:
        raise ValueError(f'Unsupported distortion model {model}; do not treat a fisheye model as pinhole')
    w=int(d.get('image_width',d.get('width',0)));h=int(d.get('image_height',d.get('height',0)))
    if min(w,h)<=0 or not np.isfinite(k).all() or k[0,0]<=0 or k[1,1]<=0 or not np.allclose(k[2],[0,0,1]):
        raise ValueError('Intrinsics need width, height and a valid calibrated K matrix')
    if len(dist) not in [0,4,5,8,12,14] or not np.isfinite(dist).all():
        raise ValueError('Unsupported or nonfinite OpenCV distortion coefficients')
    return dict(width=w,height=h,K=k,D=dist,model=model,source=str(path.resolve()),frame_id=d.get('frame_id','camera_color_optical_frame'))


def intrinsics_json(c):
    return dict(width=c['width'],height=c['height'],K=c['K'].tolist(),D=c['D'].tolist(),distortion_model=c['model'],frame_id=c.get('frame_id','camera_color_optical_frame'))


class Observer:
    def __init__(self,board_config,intrinsics,max_error=1.5,min_corners=12):
        if not hasattr(cv2,'aruco'):raise RuntimeError('OpenCV aruco missing; install opencv-contrib-python')
        self.config=read_json(board_config) if isinstance(board_config,(str,Path)) else board_config
        b=self.config;self.nx,self.ny=b['squares'];self.square=float(b['square_m']);self.marker=float(b['marker_m'])
        if min(self.nx,self.ny)<3 or not 0<self.marker<self.square:raise ValueError('Invalid board geometry')
        self.dictionary=cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco,b['dictionary']))
        if hasattr(cv2.aruco,'CharucoBoard_create'):
            if self.ny%2==0 and not b.get('legacy_pattern',True):
                raise ValueError('This older OpenCV requires legacy_pattern=true for even-row boards; use a newer OpenCV for a modern even-row layout')
            self.board=cv2.aruco.CharucoBoard_create(self.nx,self.ny,self.square,self.marker,self.dictionary)
        else:
            self.board=cv2.aruco.CharucoBoard((self.nx,self.ny),self.square,self.marker,self.dictionary)
            self.board.setLegacyPattern(bool(b.get('legacy_pattern',True)))
        self.c=intrinsics;self.max_error=max_error;self.min_corners=min_corners
        self.detector=None
        if hasattr(cv2.aruco,'CharucoDetector'):
            params=cv2.aruco.CharucoParameters()
            if intrinsics is not None:params.cameraMatrix=intrinsics['K'];params.distCoeffs=intrinsics['D']
            self.detector=cv2.aruco.CharucoDetector(self.board,params)
        # Explicit IDs -> geometry keeps board axes consistent between OpenCV versions.
        ids=np.arange((self.nx-1)*(self.ny-1))
        self.object_points=np.c_[(ids%(self.nx-1)+1)*self.square,(ids//(self.nx-1)+1)*self.square,np.zeros(len(ids))].astype(np.float32)

    def board_image(self,pixels_per_square=100):
        size=(self.nx*pixels_per_square,self.ny*pixels_per_square)
        if hasattr(self.board,'generateImage'):return self.board.generateImage(size)
        return self.board.draw(size)

    def estimate(self,image):
        if image is None:raise ValueError('Could not read image')
        c=self.c
        if image.shape[1]!=c['width'] or image.shape[0]!=c['height']:
            raise ValueError(f"Image is {image.shape[1]}x{image.shape[0]}, intrinsics are {c['width']}x{c['height']}; use matching CameraInfo, do not silently resize")
        gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY) if image.ndim==3 else image
        if self.detector is not None:
            corners,ids,_,_=self.detector.detectBoard(gray)
        else:
            marker_corners,marker_ids,_=cv2.aruco.detectMarkers(gray,self.dictionary)
            if marker_ids is None:raise ValueError('No board markers detected')
            _,corners,ids=cv2.aruco.interpolateCornersCharuco(marker_corners,marker_ids,gray,self.board,cameraMatrix=c['K'],distCoeffs=c['D'])
        if ids is None or len(ids)<self.min_corners:raise ValueError(f'Need at least {self.min_corners} ChArUco corners; see more of the board')
        ids=ids.reshape(-1)
        if ids.min()<0 or ids.max()>=len(self.object_points) or len(np.unique(ids))!=len(ids):raise ValueError('Invalid corner IDs')
        obj=self.object_points[ids];img=corners.reshape(-1,2)
        ok,rv,tv,inliers=cv2.solvePnPRansac(obj,img,c['K'],c['D'],iterationsCount=200,reprojectionError=2.,confidence=.999,flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok or inliers is None:raise ValueError('Pose solve failed')
        keep=inliers.reshape(-1)
        if len(keep)<self.min_corners or len(keep)<.8*len(ids):raise ValueError('Too many inconsistent corners; check board layout and image quality')
        extent=np.ptp(obj[keep,:2],axis=0)
        if np.any(extent<np.array([self.nx-2,self.ny-2])*self.square*.3):raise ValueError('Corners cover too little of the board for a reliable pose')
        rv,tv=cv2.solvePnPRefineLM(obj[keep],img[keep],c['K'],c['D'],rv,tv)
        r=cv2.Rodrigues(rv)[0];t=np.eye(4);t[:3,:3]=r;t[:3,3]=tv.ravel()
        if np.min((obj@r.T+tv.ravel())[:,2])<=0:raise ValueError('Board lies behind camera in solved pose')
        projection=cv2.projectPoints(obj[keep],rv,tv,c['K'],c['D'])[0].reshape(-1,2)
        errors=np.linalg.norm(projection-img[keep],axis=1);rms=float(np.sqrt(np.mean(errors**2)))
        if rms>self.max_error:raise ValueError(f'Reprojection RMS {rms:.2f}px exceeds {self.max_error:.2f}px')
        return dict(T=t,rvec=rv,tvec=tv,corners=corners,ids=ids.reshape(-1,1),inlier_ids=ids[keep].tolist(),corner_count=len(ids),inlier_count=len(keep),reprojection_rms_px=rms,reprojection_max_px=float(errors.max()),image_points=img.tolist())

    def annotate(self,image,result,reference=None):
        image=image.copy();cv2.aruco.drawDetectedCornersCharuco(image,result['corners'])
        cv2.drawFrameAxes(image,self.c['K'],self.c['D'],result['rvec'],result['tvec'],self.square*3,2)
        t=result['T'];lines=[f"Corners {result['inlier_count']}/{result['corner_count']} | RMS {result['reprojection_rms_px']:.3f} px",f"Board in camera: X {t[0,3]*1000:.1f} Y {t[1,3]*1000:.1f} Z {t[2,3]*1000:.1f} mm",'Board axes: X red / Y green / Z blue (into printed board)']
        if reference is not None:
            if reference['board_fingerprint']!=fingerprint(self.config):raise ValueError('Reference board configuration differs')
            old=inverse(reference['camera_T_board']);new=inverse(t);d=(new[:3,3]-old[:3,3])*1000
            lines += [f"Camera position delta in board frame: {d[0]:+.1f} {d[1]:+.1f} {d[2]:+.1f} mm",f"Camera orientation difference: {rotation_degrees(new[:3,:3],old[:3,:3]):.2f} deg (matching is optional)"]
        for i,line in enumerate(lines):
            cv2.putText(image,line,(12,28+25*i),cv2.FONT_HERSHEY_SIMPLEX,.55,(0,0,0),3,cv2.LINE_AA)
            cv2.putText(image,line,(12,28+25*i),cv2.FONT_HERSHEY_SIMPLEX,.55,(0,255,255),1,cv2.LINE_AA)
        return image


class StableSamples:
    def __init__(self,count=30,translation_mm=3.,rotation_deg=1.):
        if count<2:raise ValueError('Live observation requires at least 2 samples')
        self.count=count;self.translation_mm=translation_mm;self.rotation_deg=rotation_deg;self.results=[]

    def add(self,result):
        self.results.append(result);self.results=self.results[-self.count:]
        if len(self.results)<self.count:return False
        poses=np.array([r['T'] for r in self.results]);mean=mean_pose(poses)
        pos=np.linalg.norm(poses[:,:3,3]-mean[:3,3],axis=1)*1000
        rot=[rotation_degrees(t[:3,:3],mean[:3,:3]) for t in poses]
        return bool(pos.max()<=self.translation_mm and max(rot)<=self.rotation_deg)

    def clear(self):self.results=[]


def save_observation(out_dir,observer,results,image,reference=None,camera_id='',source=''):
    out=Path(out_dir);out.mkdir(parents=True,exist_ok=True)
    poses=np.array([r['T'] for r in results]);mean=mean_pose(poses)
    pos=np.linalg.norm(poses[:,:3,3]-mean[:3,3],axis=1)*1000
    rot=[rotation_degrees(t[:3,:3],mean[:3,:3]) for t in poses]
    doc=dict(schema_version=1,created_at=datetime.now(timezone.utc).isoformat(),opencv_version=cv2.__version__,camera_id=camera_id,image_source=source,
        convention='A_T_B maps B to A; metres; column vectors; optical camera +X right +Y down +Z forward',
        board_axes='origin at top-left outer board corner in reference image; +X right, +Y down printed board, +Z into board',
        board_config=observer.config,board_fingerprint=fingerprint(observer.config),camera_intrinsics=intrinsics_json(observer.c),
        camera_T_board=mean.tolist(),board_T_camera=inverse(mean).tolist(),sample_count=len(results),
        quality=dict(reprojection_rms_px_mean=float(np.mean([r['reprojection_rms_px'] for r in results])),reprojection_rms_px_max=float(max(r['reprojection_rms_px'] for r in results)),translation_spread_max_mm=float(pos.max()),rotation_spread_max_deg=float(max(rot)),min_inlier_corners=min(r['inlier_count'] for r in results)),
        notes=['Stability and reprojection are internal checks, not absolute accuracy guarantees.','Between separate sites, corresponding board placement defines the intended task mapping.','This observation does not recalibrate robot hand-eye transforms.'])
    write_json(out/'observation.json',doc)
    write_json(out/'samples.json',[{k:(v.tolist() if isinstance(v,np.ndarray) else v) for k,v in r.items()} for r in results])
    cv2.imwrite(str(out/'image.png'),image);cv2.imwrite(str(out/'overlay.jpg'),observer.annotate(image,results[-1],reference))
    print(f"Saved {out/'observation.json'} | {len(results)} samples | RMS {doc['quality']['reprojection_rms_px_mean']:.3f}px",flush=True)
    return doc


def align_observations(source,target):
    if source['board_fingerprint']!=target['board_fingerprint']:raise ValueError('Source and target board configurations differ')
    if source.get('schema_version')!=1 or target.get('schema_version')!=1:raise ValueError('Unsupported observation schema')
    a=validate_transform(source['camera_T_board']);b=validate_transform(target['camera_T_board']);t=b@inverse(a)
    # In separate workcells these are corresponding task frames, not a measured
    # physical transform between the buildings or rooms.
    return dict(schema_version=1,created_at=datetime.now(timezone.utc).isoformat(),board_fingerprint=source['board_fingerprint'],
        target_camera_T_source_camera=t.tolist(),source_camera_T_target_camera=inverse(t).tolist(),
        formula='T_target_camera_wuji = target_camera_T_source_camera @ T_source_camera_wuji',
        assumption='The two observed board frames represent corresponding task origins and axes; identity mapping between those task frames is user-established by physical placement.',
        interpretation='Task-transfer mapping between separate workcells, not a surveyed inter-site physical camera extrinsic.',
        source_camera_id=source.get('camera_id',''),target_camera_id=target.get('camera_id',''),
        source_quality=source['quality'],target_quality=target['quality'],
        same_board_coordinate_closure_max_abs=float(abs(t@a-b).max()))


def main():
    ap=argparse.ArgumentParser(description=__doc__);sub=ap.add_subparsers(dest='command',required=True)
    g=sub.add_parser('board',help='Generate reference board and physical-size SVG');g.add_argument('--board',type=Path,default=DEFAULT_BOARD);g.add_argument('--out',type=Path,required=True)
    p=sub.add_parser('observe',help='Observe one or more photos with fixed board and camera');p.add_argument('--images',type=Path,nargs='+',required=True)
    l=sub.add_parser('live',help='Observe a USB/OpenCV camera; automatically save a stable sample window');l.add_argument('--device',default='0');l.add_argument('--samples',type=int,default=30);l.add_argument('--timeout',type=float,default=120);l.add_argument('--show',action='store_true');l.add_argument('--manual-save',action='store_true',help='Keep preview open; press S to save a stable window')
    for q in [p,l]:
        q.add_argument('--board',type=Path,default=DEFAULT_BOARD);q.add_argument('--intrinsics',type=Path,required=True);q.add_argument('--out',type=Path,required=True);q.add_argument('--reference',type=Path);q.add_argument('--camera-id',default='');q.add_argument('--max-error',type=float,default=1.5)
    a=sub.add_parser('align',help='Map source-camera task trajectories into target-camera coordinates');a.add_argument('--source',type=Path,required=True);a.add_argument('--target',type=Path,required=True);a.add_argument('--out',type=Path,required=True)
    args=ap.parse_args()
    if args.command=='live' and args.manual_save and not args.show:ap.error('--manual-save requires --show')
    if args.command=='board':
        o=Observer(args.board,None);args.out.mkdir(parents=True,exist_ok=True);im=o.board_image(200);cv2.imwrite(str(args.out/'board_reference.png'),im)
        payload=base64.b64encode(cv2.imencode('.png',im)[1]).decode();w=o.nx*o.square*1000;h=o.ny*o.square*1000
        svg=f'<svg xmlns="http://www.w3.org/2000/svg" width="{w+16}mm" height="{h+24}mm" viewBox="0 0 {w+16} {h+24}"><rect width="100%" height="100%" fill="white"/><text x="8" y="5" font-size="3">ORIGIN below-left | X right | Y down | Z into board</text><image x="8" y="8" width="{w}" height="{h}" href="data:image/png;base64,{payload}"/><text x="8" y="{h+14}" font-size="3">PRINT 100% / ACTUAL SIZE. Each square {o.square*1000:g} mm. Check with ruler.</text><text x="8" y="{h+20}" font-size="3">{o.config["dictionary"]} | {o.nx} x {o.ny} squares | board {w:g} x {h:g} mm</text></svg>'
        (args.out/'board_actual_size.svg').write_text(svg);write_json(args.out/'board.json',o.config);print(args.out);return
    if args.command=='align':
        result=align_observations(read_json(args.source),read_json(args.target));result['source_observation']=str(args.source);result['target_observation']=str(args.target);write_json(args.out,result);print(f'Saved {args.out}');return
    c=load_intrinsics(args.intrinsics);observer=Observer(args.board,c,args.max_error);reference=read_json(args.reference) if args.reference else None
    if args.command=='observe':
        results=[]
        for path in args.images:
            image=cv2.imread(str(path));results.append(observer.estimate(image))
        if len(results)>1:
            check=StableSamples(len(results))
            for r in results:stable=check.add(r)
            if not stable:raise ValueError('Photos do not describe one stable board/camera pose; do not move the board between these photos')
        save_observation(args.out,observer,results,image,reference,args.camera_id,','.join(map(str,args.images)));return
    device=int(args.device) if args.device.isdigit() else args.device;cap=cv2.VideoCapture(device);cap.set(cv2.CAP_PROP_FRAME_WIDTH,c['width']);cap.set(cv2.CAP_PROP_FRAME_HEIGHT,c['height'])
    if not cap.isOpened():raise RuntimeError('Could not open camera')
    samples=StableSamples(args.samples);start=time.monotonic();last_log=0
    try:
        while time.monotonic()-start<args.timeout:
            ok,image=cap.read()
            if not ok:raise RuntimeError('Camera read failed')
            try:
                r=observer.estimate(image);ready=samples.add(r);shown=observer.annotate(image,r,reference)
                message=f"Samples {len(samples.results)}/{args.samples}, RMS {r['reprojection_rms_px']:.3f}px"
            except ValueError as e:
                samples.clear();message=str(e);ready=False;shown=image
            if time.monotonic()-last_log>1:print(message,flush=True);last_log=time.monotonic()
            save_requested=not args.manual_save
            if args.show:
                cv2.putText(shown,'S: save when stable | Q: quit',(12,shown.shape[0]-20),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,255,255),2)
                cv2.imshow('Board alignment - Q to quit',shown)
                key=cv2.waitKey(1)&255
                if key in [ord('q'),27]:raise RuntimeError('Cancelled without saving a pose')
                save_requested=save_requested or key==ord('s')
            if ready and save_requested:save_observation(args.out,observer,samples.results,image,reference,args.camera_id,str(device));return
        raise RuntimeError('Timed out without a stable observation')
    finally:
        cap.release()
        if args.show:cv2.destroyAllWindows()


if __name__=='__main__':
    try:main()
    except (ValueError,RuntimeError) as error:raise SystemExit(str(error))
