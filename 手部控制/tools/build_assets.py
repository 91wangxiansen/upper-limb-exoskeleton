"""Extract the original assembled CAD meshes and embed offline web assets.
Requires Python 3 + numpy. No CAD or internet service is used at runtime.
"""
from pathlib import Path
import gzip, hashlib, json, shutil, xml.etree.ElementTree as ET, zipfile
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
WEB = ROOT / 'web'

def extract_model():
    source = ROOT.parent / 'mechanics/FDM_3DPrinting/_概念总装Assembly.3MF'
    with zipfile.ZipFile(source) as z:
        root = ET.fromstring(z.read('3D/3dmodel.model'))
    ns = {'m': root.tag.split('}')[0][1:]}
    objects = {o.attrib['id']: o for o in root.findall('m:resources/m:object', ns)}
    blob = bytearray(); parts = []
    finger_by_id = {'2':0,'20':0,'23':0,'29':1,'38':1,'41':1,
                    '26':2,'32':2,'35':2,'50':3,'53':3,'56':3}
    proximal = {'20','29','26','56'}
    distal = {'2','41','35','50'}
    omit = {'65','71','74','77'}  # thumb-only hardware
    fingers = [None]*4
    for instance,c in enumerate(objects['1'].find('m:components', ns)):
        key = c.attrib['objectid']
        if key in omit: continue
        obj = objects[key]
        leaf = objects[obj.find('m:components', ns)[0].attrib['objectid']]
        mesh = leaf.find('m:mesh', ns)
        v = np.array([[float(a.attrib[k]) for k in ('x','y','z')] for a in mesh.find('m:vertices',ns)])
        t = np.array(list(map(float,c.attrib.get('transform','1 0 0 0 1 0 0 0 1 0 0 0').split()))).reshape(4,3)
        v = v @ t[:3] + t[3]
        v = np.column_stack((v[:,0],-v[:,2],v[:,1])).astype('<f4')
        faces = np.array([[int(a.attrib[k]) for k in ('v1','v2','v3')] for a in mesh.find('m:triangles',ns)],dtype='<u2')
        finger = finger_by_id.get(key,-1)
        if key in {'5','11','14','17','44','47'}:
            finger = min(range(4),key=lambda f:abs(float(v[:,0].mean())-[-35,-8,19,43][f]))
        kind = 'proximal' if key in proximal else 'distal' if key in distal else 'link' if finger>=0 else 'base'
        while len(blob)%4: blob.append(0)
        vo = len(blob); blob.extend(v.tobytes())
        io = len(blob); blob.extend(faces.tobytes())
        parts.append(dict(name=obj.attrib.get('name',key),key=key,instance=instance,
                          finger=finger,kind=kind,vertexOffset=vo,vertexCount=len(v),
                          indexOffset=io,indexCount=faces.size))
        if key in proximal:
            lo,hi=v.min(0),v.max(0)
            fingers[finger] = dict(mcp=[float((lo[0]+hi[0])/2),float(lo[1]+6),0.0],
                                   pip=[float((lo[0]+hi[0])/2),float(hi[1]-8),0.0])
    manifest = dict(version=1,source=source.name,sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                    units='mm',kinematics='CAD mesh + approximate coupled hinge mapping; not closed-chain kinematics',
                    parts=parts,fingers=fingers)
    (WEB/'model.bin').write_bytes(blob)
    (WEB/'model.json').write_text(json.dumps(manifest,ensure_ascii=False,separators=(',',':')),encoding='utf-8')
    print(f'CAD: {len(parts)} parts, {len(blob):,} bytes; thumb omitted')

def embed():
    # Compile a sketch-local copy so Arduino IDE cannot silently pick another
    # installed SCServo version with different packet parsing behavior.
    library = ROOT / 'SCServo'
    embedded = ROOT / 'ServoDriver/src/SCServo'
    embedded.mkdir(parents=True, exist_ok=True)
    for source in library.iterdir():
        if source.suffix in {'.h', '.cpp'}:
            text = source.read_text(encoding='utf-8-sig').lstrip('\ufeff')
            source.write_text(text, encoding='utf-8')
            (embedded / source.name).write_text(text, encoding='utf-8')
    files=[('index.html','text/html; charset=utf-8'),('styles.css','text/css'),
           ('app.js','text/javascript'),('diagnostics.js','text/javascript'),('heartbeat.js','text/javascript'),('twin.js','text/javascript'),('transport.js','text/javascript'),
           ('model.json','application/json'),('model.bin','application/octet-stream'),
           ('vendor/three.module.js','text/javascript'),('vendor/three.core.js','text/javascript'),
           ('vendor/OrbitControls.js','text/javascript')]
    out=['#pragma once','#include <Arduino.h>',
         'struct WebAsset { const char* path; const char* mime; const uint8_t* data; size_t size; };']
    rows=[]; total=0
    for i,(name,mime) in enumerate(files):
        data=gzip.compress((WEB/name).read_bytes(),compresslevel=9,mtime=0); total+=len(data)
        out.append(f'static const uint8_t asset_{i}[] PROGMEM = {{')
        out.extend(','.join(f'0x{b:02x}' for b in data[j:j+32])+',' for j in range(0,len(data),32))
        out.append('};')
        rows.append(f'{{"/{name}","{mime}",asset_{i},sizeof(asset_{i})}}')
    out.append('static const WebAsset WEB_ASSETS[] = {'+','.join(rows)+'};')
    (ROOT/'ServoDriver/WebAssets.h').write_text('\n'.join(out)+'\n',encoding='utf-8')
    print(f'Offline gzip flash assets: {total:,} bytes')

if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser(); parser.add_argument('--embed-only',action='store_true'); args=parser.parse_args()
    if not args.embed_only:
        (WEB/'vendor').mkdir(parents=True,exist_ok=True)
        source=ROOT.parents[1]/'CyberGear控制系统/cybergear_upper_computer/web/vendor'
        for name in ['three.module.js','three.core.js','OrbitControls.js']: shutil.copy2(source/name,WEB/'vendor'/name)
        extract_model()
    embed()
