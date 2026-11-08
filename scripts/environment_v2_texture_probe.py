#!/usr/bin/env python3
"""Attended desktop GL capacity/readback check for the three 8192 ground maps.

The checked host supervisor must wrap this diagnostic. It does not qualify the
renderer by itself; mapped Ogre provenance and RGB-D fixtures remain required.
"""
import argparse
import ctypes as c
import hashlib
import json
import os
from pathlib import Path
import time

import cv2
import numpy as np


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('scene_folder',type=Path);p.add_argument('output',type=Path)
    a=p.parse_args()
    if not os.environ.get('DISPLAY') or os.environ.get('LIBGL_ALWAYS_SOFTWARE')!='0':
        p.error('Attended desktop hardware GL is required')
    x=c.CDLL('libX11.so.6');gl=c.CDLL('libGL.so.1')
    def api(lib,name,result,args):
        f=getattr(lib,name);f.restype=result;f.argtypes=args;return f
    ptr=c.c_void_p;integer=c.c_int;uint=c.c_uint;ulong=c.c_ulong
    open_display=api(x,'XOpenDisplay',ptr,[c.c_char_p]);close_display=api(x,'XCloseDisplay',integer,[ptr])
    screen=api(x,'XDefaultScreen',integer,[ptr])
    choose=api(gl,'glXChooseFBConfig',c.POINTER(ptr),[ptr,integer,c.POINTER(integer),c.POINTER(integer)])
    new=api(gl,'glXCreateNewContext',ptr,[ptr,ptr,integer,ptr,integer])
    make=api(gl,'glXMakeContextCurrent',integer,[ptr,ulong,ulong,ptr])
    pbuffer=api(gl,'glXCreatePbuffer',ulong,[ptr,ptr,c.POINTER(integer)])
    destroy_context=api(gl,'glXDestroyContext',None,[ptr,ptr])
    destroy_buffer=api(gl,'glXDestroyPbuffer',None,[ptr,ulong])
    get_int=api(gl,'glGetIntegerv',None,[uint,c.POINTER(integer)])
    get_string=api(gl,'glGetString',c.c_char_p,[uint]);error=api(gl,'glGetError',uint,[])
    gen=api(gl,'glGenTextures',None,[integer,c.POINTER(uint)])
    bind=api(gl,'glBindTexture',None,[uint,uint])
    image=api(gl,'glTexImage2D',None,[uint,integer,integer,integer,integer,integer,uint,uint,ptr])
    read=api(gl,'glGetTexImage',None,[uint,integer,uint,uint,ptr])
    mipmap=api(gl,'glGenerateMipmap',None,[uint]);delete=api(gl,'glDeleteTextures',None,[integer,c.POINTER(uint)])
    display=open_display(None)
    if not display:raise RuntimeError('Desktop display authentication failed')
    context=None;buffer=0;textures=(uint*3)();report=dict(result='RUNNING',textures=[])
    try:
        attrs=(integer*7)(0x8010,4,0x8011,1,0x8012,1,0)
        count=integer();configs=choose(display,screen(display),attrs,c.byref(count))
        if not configs or count.value<1:raise RuntimeError('No hardware GLX pbuffer configuration')
        context=new(display,configs[0],0x8014,None,1)
        if not context:raise RuntimeError('GLX context failed')
        buffer=pbuffer(display,configs[0],(integer*5)(0x8041,32,0x8040,32,0))
        if not buffer or not make(display,buffer,buffer,context):raise RuntimeError('GLX context binding failed')
        renderer=get_string(0x1f01).decode();vendor=get_string(0x1f00).decode()
        if any(word in renderer.lower() for word in ('llvmpipe','softpipe','swrast')):raise RuntimeError('Software rendering is forbidden')
        maximum=integer();get_int(0x0d33,c.byref(maximum))
        if maximum.value<8192:raise RuntimeError('8192 textures unsupported')
        report.update(renderer=renderer,vendor=vendor,max_texture_size=maximum.value,
                      meminfo_before=Path('/proc/meminfo').read_text())
        gen(3,textures)
        total=0
        for index,name in enumerate(('ground_albedo.jpg','ground_normal.jpg','ground_roughness.jpg')):
            began=time.monotonic();source=a.scene_folder/name
            pixels=cv2.imread(str(source),cv2.IMREAD_GRAYSCALE if index==2 else cv2.IMREAD_COLOR)
            if pixels is None or pixels.shape[:2]!=(8192,8192):raise RuntimeError('Wrong or missing ground composite: '+name)
            if index!=2:pixels=cv2.cvtColor(pixels,cv2.COLOR_BGR2RGB)
            pixels=np.ascontiguousarray(pixels);fmt=0x1903 if index==2 else 0x1907
            internal=0x8229 if index==2 else 0x8051
            bind(0x0de1,textures[index]);image(0x0de1,0,internal,8192,8192,0,fmt,0x1401,pixels.ctypes.data)
            mipmap(0x0de1)
            if error():raise RuntimeError('Texture allocation/mipmap failed: '+name)
            actual=np.empty_like(pixels);read(0x0de1,0,fmt,0x1401,actual.ctypes.data)
            if error() or not np.array_equal(actual,pixels):raise RuntimeError('Texture readback mismatch: '+name)
            total+=pixels.nbytes
            report['textures'].append(dict(file=str(source),file_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                shape=list(pixels.shape),uncompressed_bytes=pixels.nbytes,exact_readback=True,seconds=time.monotonic()-began))
        report.update(result='8192_GL_ALLOCATION_MIPMAP_AND_READBACK_PASS',base_texture_bytes=total,
            estimated_bytes_with_full_mipmaps=round(total*4/3),meminfo_after=Path('/proc/meminfo').read_text(),
            limitation='Desktop GL texture capacity; Ogre provenance and rendered fixture checks are separate.')
    except Exception as e:
        report.update(result='FAIL',failure=str(e));raise
    finally:
        if context:
            delete(3,textures);make(display,0,0,None)
            if buffer:destroy_buffer(display,buffer)
            destroy_context(display,context)
        close_display(display)
        with a.output.open('x') as f:json.dump(report,f,indent=2)
    print(json.dumps({k:v for k,v in report.items() if not k.startswith('meminfo')}))


if __name__=='__main__':main()
