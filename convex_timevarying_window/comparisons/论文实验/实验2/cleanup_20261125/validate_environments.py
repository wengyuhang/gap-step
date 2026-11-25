import json,hashlib,os,collections
from pathlib import Path
R=Path('/home/jack/wyh/时变窗口');O=R/'convex_timevarying_window/comparisons/论文实验/实验2/cleanup_20261125';live=R/'convex_dynamic_seven_window_gazebo/.runtime/teleop-venv'
cache={}
def sha(p):
 s=p.stat();k=(s.st_dev,s.st_ino,s.st_size,s.st_mtime_ns)
 if k not in cache:cache[k]=hashlib.sha256(p.read_bytes()).hexdigest()
 return cache[k]
meta=[]
for p in live.glob('lib/python*/site-packages/*.dist-info/METADATA'):
 lines=p.read_text().splitlines();m={k:next((l[len(k)+2:] for l in lines if l.startswith(k+': ')),None) for k in ['Name','Version']};m['retained_metadata']=str(p);m['metadata_sha256']=sha(p);meta.append(m)
versions={x['Name'].lower().replace('-','_'):x for x in meta}
rows=json.loads((O/'DELETE_MANIFEST.json').read_text());inv=json.loads((O/'ENVIRONMENTS.json').read_text());counts=collections.Counter();bad=collections.defaultdict(list)
for row in rows:
 if row['category']!='snapshot_virtual_environment':continue
 p=Path(row['path']);i=p.parts.index('teleop-venv');env=Path(*p.parts[:i+1]);other=live/p.relative_to(env)
 try:ok=(other.is_symlink() and os.readlink(other)==row['link_target']) if row['symlink'] else other.is_file() and sha(other)==row['sha256']
 except OSError:ok=False
 counts[str(env)]+=1
 if not ok:bad[str(env)].append(str(p))
for e in inv:
 env=Path(e['environment']);e['snapshot_is_source_subset_not_complete_venv']=True;e['retained_live_environment']=str(live);e['compared_files']=counts[str(env)];e['mismatches']=bad[str(env)];e['cleanup_allowed']=not bad[str(env)]
 packages=[]
 for sp in env.glob('lib/python*/site-packages'):
  for p in sp.iterdir():
   name={'_distutils_hack':'setuptools','pkg_resources':'setuptools'}.get(p.name,p.name).lower().replace('-','_')
   if name in versions and versions[name] not in packages:packages.append(versions[name])
   elif name not in versions:e['cleanup_allowed']=False;e.setdefault('unidentified_packages',[]).append(name)
 e['packages']=packages
 e['version_evidence']='All copied files equal retained installed environment; package versions from retained METADATA. Not a complete old virtual environment archive.'
allowed={e['environment'] for e in inv if e['cleanup_allowed']};kept=[];deleted=[]
for a in rows:
 if a['category']=='snapshot_virtual_environment':
  p=Path(a['path']);env=str(Path(*p.parts[:p.parts.index('teleop-venv')+1]))
  if env not in allowed:kept.append(a);continue
 deleted.append(a)
with (O/'RETAINED_BEFORE.jsonl').open('a') as f:
 for a in kept:
  q={'path':a['path'],'symlink':a['symlink']}
  if a['symlink']:q['target']=a['link_target']
  else:q.update(size=a['size'],sha256=a['sha256'])
  f.write(json.dumps(q,ensure_ascii=False)+'\n')
for name,data in [('ENVIRONMENTS.json',inv),('DELETE_MANIFEST.json',deleted)]:
 (O/name).write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n')
s=json.loads((O/'SUMMARY.json').read_text());s['retained_files']+=len(kept);s['delete_files']=len(deleted);s['category_counts']=dict(collections.Counter(a['category'] for a in deleted));s['environment_copies_verified']=len(allowed);s['environment_copies_retained']=len(inv)-len(allowed)
un={};links=collections.Counter();est=collections.Counter()
for a in deleted:k=(a['dev'],a['ino']);un[k]=a;links[k]+=1
for k,a in un.items():
 if links[k]>=a['nlink']:est[a['category']]+=a['blocks']*512
s['estimated_released_bytes_by_category']=dict(est);(O/'SUMMARY.json').write_text(json.dumps(s,ensure_ascii=False,indent=2)+'\n')
(O/'validate_environments.py').write_text(Path(__file__).read_text())
print(json.dumps(s,ensure_ascii=False))
