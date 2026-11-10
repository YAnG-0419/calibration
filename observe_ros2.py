#!/usr/bin/env python3
"""Read-only ROS 2 Image + CameraInfo observer. Does not publish robot commands."""
import argparse
import time
from pathlib import Path
import cv2
import numpy as np
from board_align import Observer,StableSamples,DEFAULT_BOARD,read_json,save_observation


def camera_info_dict(info,rectified=False):
    if info.binning_x>1 or info.binning_y>1 or info.roi.x_offset or info.roi.y_offset or info.roi.width not in [0,info.width] or info.roi.height not in [0,info.height]:
        raise ValueError('Use full-frame unbinned CameraInfo; ROI/binning conversion is not implemented')
    if rectified:
        if not np.allclose(np.array(info.r).reshape(3,3),np.eye(3),atol=1e-8):
            raise ValueError('Rectification rotates the optical frame. Use raw color Image/CameraInfo to match hand-eye optical frame')
        p=np.array(info.p).reshape(3,4)
        if not np.allclose(p[:,3],0):raise ValueError('Stereo projection offset is unsupported; use raw color Image/CameraInfo')
        k=p[:,:3];dist=np.zeros(5);model='plumb_bob'
    else:k=np.array(info.k).reshape(3,3);dist=np.array(info.d);model=info.distortion_model
    if model not in ['plumb_bob','rational_polynomial'] or len(dist) not in [0,4,5,8,12,14]:
        raise ValueError(f'Unsupported CameraInfo distortion model: {model}')
    if min(info.width,info.height)<=0 or not np.isfinite(k).all() or not np.isfinite(dist).all() or min(k[0,0],k[1,1])<=0 or not np.allclose(k[2],[0,0,1]):
        raise ValueError('CameraInfo has invalid intrinsics')
    return dict(width=info.width,height=info.height,K=k,D=dist,model=model,frame_id=info.header.frame_id,source='ROS 2 CameraInfo')


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--image-topic',required=True);ap.add_argument('--info-topic',required=True)
    ap.add_argument('--out',type=Path,required=True);ap.add_argument('--board',type=Path,default=DEFAULT_BOARD)
    ap.add_argument('--reference',type=Path);ap.add_argument('--camera-id',default='')
    ap.add_argument('--samples',type=int,default=30);ap.add_argument('--timeout',type=float,default=120)
    ap.add_argument('--max-error',type=float,default=1.5);ap.add_argument('--show',action='store_true');ap.add_argument('--manual-save',action='store_true')
    ap.add_argument('--rectified',action='store_true',help='Use P and zero distortion; only identity optical rectification is supported')
    args=ap.parse_args()
    if args.manual_save and not args.show:ap.error('--manual-save requires --show')
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image,CameraInfo
    from cv_bridge import CvBridge
    rclpy.init();node=rclpy.create_node('task_board_observer');state={'image':None,'info':None,'generation':0}
    def image_cb(msg):state.update(image=msg,generation=state['generation']+1)
    def info_cb(msg):state['info']=msg
    node.create_subscription(Image,args.image_topic,image_cb,qos_profile_sensor_data)
    node.create_subscription(CameraInfo,args.info_topic,info_cb,qos_profile_sensor_data)
    bridge=CvBridge();reference=read_json(args.reference) if args.reference else None
    sample=StableSamples(args.samples);observer=None;old_info=None;seen=-1;last_stamp=None;start=time.monotonic();last_log=0
    try:
        while rclpy.ok() and time.monotonic()-start<args.timeout:
            rclpy.spin_once(node,timeout_sec=.05)
            msg,info=state['image'],state['info']
            if msg is None or info is None:
                if time.monotonic()-last_log>2:print('Waiting for Image and CameraInfo...',flush=True);last_log=time.monotonic()
                continue
            if seen==state['generation']:continue
            seen=state['generation'];stamp=(msg.header.stamp.sec,msg.header.stamp.nanosec)
            if stamp==last_stamp:continue
            last_stamp=stamp
            if msg.header.frame_id!=info.header.frame_id:
                raise ValueError('Image and CameraInfo frame_id differ; select the matching color optical topics')
            c=camera_info_dict(info,args.rectified);signature=(c['width'],c['height'],tuple(c['K'].ravel()),tuple(c['D']),c['frame_id'])
            if signature!=old_info:observer=Observer(args.board,c,args.max_error);sample.clear();old_info=signature
            image=bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
            try:
                result=observer.estimate(image);result['stamp_sec']=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
                ready=sample.add(result);shown=observer.annotate(image,result,reference)
                status=f"Stable samples {len(sample.results)}/{args.samples}, RMS {result['reprojection_rms_px']:.3f}px"
            except ValueError as e:sample.clear();ready=False;shown=image;status=str(e)
            if time.monotonic()-last_log>1:print(status,flush=True);last_log=time.monotonic()
            save_requested=not args.manual_save
            if args.show:
                cv2.putText(shown,'S: save when stable | Q: quit',(12,shown.shape[0]-20),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,255,255),2)
                cv2.imshow('ROS board observer - Q to quit',shown)
                key=cv2.waitKey(1)&255
                if key in [ord('q'),27]:raise RuntimeError('Cancelled without saving a pose')
                save_requested=save_requested or key==ord('s')
            if ready and save_requested:
                save_observation(args.out,observer,sample.results,image,reference,args.camera_id,args.image_topic)
                return
        raise RuntimeError('No stable observation saved; check topics, board visibility, dimensions and lighting')
    finally:
        node.destroy_node();rclpy.shutdown()
        if args.show:cv2.destroyAllWindows()


if __name__=='__main__':
    try:main()
    except (ValueError,RuntimeError) as error:raise SystemExit(str(error))
