#!/usr/bin/env python3
"""Read actual fixture-server plugin/Mesa mappings before any probe changes logs."""
import hashlib
import json
import os
from pathlib import Path


def main():
    processes=[]; observations=[]
    for directory in Path('/proc').iterdir():
        if not directory.name.isdecimal(): continue
        try:
            command=(directory/'cmdline').read_bytes().replace(b'\0',b' ').decode().strip()
            comm=(directory/'comm').read_text().strip()
            observation=dict(pid=int(directory.name),comm=comm,command=command)
            observations.append(observation)
            try: maps=(directory/'maps').read_text()
            except OSError as error:
                observation['maps_error']=str(error); continue
            if 'libgz-rendering8-ogre.so' not in maps: continue
        except (OSError,UnicodeError): continue
        libraries=[]
        for line in maps.splitlines():
            fields=line.split(None,5)
            if len(fields)!=6 or 'x' not in fields[1]: continue
            name=fields[5]
            if not any(marker in name for marker in ('libgz-rendering8-ogre.so','libgallium','radeonsi_dri.so')): continue
            path=Path(name); info=path.stat()
            major,minor=map(lambda v:int(v,16),fields[3].split(':'))
            if (major,minor,int(fields[4]))!=(os.major(info.st_dev),os.minor(info.st_dev),info.st_ino):
                raise RuntimeError('Mapped library changed: '+name)
            with path.open('rb') as stream: sha=hashlib.file_digest(stream,'sha256').hexdigest()
            libraries.append(dict(path=name,sha256=sha,inode=info.st_ino,mapping=line))
        processes.append(dict(pid=int(directory.name),command=command,libraries=libraries,maps=maps))
    log=Path('/tmp/.gz/rendering/ogre.log').read_text()
    identities=[line for line in log.splitlines() if 'GL_RENDERER' in line]
    print(json.dumps(dict(processes=processes,renderer_identities=identities,observations=observations),indent=2),flush=True)
    if len(processes)!=1 or not identities or not all('Radeon 8060S' in s and 'radeonsi' in s for s in identities):
        raise RuntimeError('Expected one fixture server and observed Radeon/radeonsi identity')
    libraries=processes[0]['libraries']
    if not any('libgz-rendering8-ogre.so' in lib['path'] for lib in libraries):
        raise RuntimeError('Ogre backend mapping is missing')
    if not any('libgallium' in lib['path'] or 'radeonsi_dri.so' in lib['path'] for lib in libraries):
        raise RuntimeError('Executable Mesa driver mapping is missing')


if __name__=='__main__': main()
