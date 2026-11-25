import os,sys,json,hashlib,subprocess,time,stat,collections,re
from pathlib import Path
R=Path('/home/jack/wyh/时变窗口'); C=R/'convex_timevarying_window/comparisons'; E=C/'论文实验/实验2'; O=E/'cleanup_20261125'
PROTECTED_NAMES={'robust_ipopt_full_20261117','minco_position_ablation_20261122','dynamic_togt_audit_20261117','wang_audit_20261117','cpc_repaired_20261117','six_methods_cpc_repaired_20261117','six_methods_planning_20261117'}
PROTECTED=[E/n for n in PROTECTED_NAMES]
def within(p,d):return p==d or d in p.parents
def protected(p):return any(within(p,d) for d in PROTECTED) or within(p,O)
def dump(p,x):p.write_text(json.dumps(x,ensure_ascii=False,indent=2)+'\n')
def disk():s=os.statvfs(C);return {'available':s.f_bavail*s.f_frsize,'free':s.f_bfree*s.f_frsize}
def tracked():return {R/os.fsdecode(p) for p in subprocess.check_output(['git','-C',str(R),'ls-files','-z']).split(b'\0') if p}
def open_inodes():
 out=set();cwds=[]
 for proc in Path('/proc').glob('[0-9]*'):
  try:
   cwd=Path(os.readlink(proc/'cwd'));cwds.append(str(cwd))
   for f in (proc/'fd').iterdir():
    try:s=f.stat();out.add((s.st_dev,s.st_ino))
    except OSError:pass
   for line in (proc/'maps').read_text().splitlines():
    a=line.split(None,5)
    if len(a)==6 and a[5].startswith(str(C)):
     try:s=Path(a[5]).stat();out.add((s.st_dev,s.st_ino))
     except OSError:pass
  except (OSError,PermissionError):pass
 return out,cwds
hashcache={}
def sha(p):
 s=p.stat();key=(s.st_dev,s.st_ino,s.st_size,s.st_mtime_ns)
 if key not in hashcache:
  h=hashlib.sha256()
  with p.open('rb') as f:
   for b in iter(lambda:f.read(4*1024*1024),b''):h.update(b)
  hashcache[key]=h.hexdigest()
 return hashcache[key]
def exp(p):
 parts=p.relative_to(C).parts
 for k in ('results','outputs'):
  if k in parts:
   i=parts.index(k)
   if len(parts)>i+2:return str(C.joinpath(*parts[:i+2]))
 return str(p.parent)
def files():
 for d,ds,fs in os.walk(C,followlinks=False):
  ds[:]=[v for v in ds if v!='.git' and not within(Path(d)/v,O)]
  for n in fs:
   p=Path(d)/n
   if not within(p,O):yield p
  for n in list(ds):
   p=Path(d)/n
   if p.is_symlink():yield p;ds.remove(n)
def envroot(p):
 parts=p.parts
 if 'source_snapshot' in parts and 'teleop-venv' in parts:
  i=parts.index('teleop-venv')
  if parts[i-1]=='.runtime':return Path(*parts[:i+1])
 return None

def prepare():
 if (O/'SUMMARY.json').exists() or (O/'DELETED.jsonl').exists():raise RuntimeError('Already prepared/applied; do not overwrite')
 O.mkdir(exist_ok=True);(O/'cleanup_tool.py').write_text(Path(__file__).read_text())
 tr=tracked();opened,cwds=open_inodes();allfiles=list(files());envs=sorted({envroot(p) for p in allfiles if envroot(p)},key=str)
 banned_envs={e for e in envs if any(within(t,e) for t in tr) or any(within(Path(c),e) for c in cwds)}
 inventory=[]
 for env in envs:
  if env in banned_envs:continue
  packages=[]
  for meta in env.glob('lib/python*/site-packages/*.dist-info/METADATA'):
   lines=meta.read_text(errors='replace').splitlines(); vals={k:next((l[len(k)+2:] for l in lines if l.startswith(k+': ')),None) for k in ('Name','Version')}
   vals['metadata_sha256']=sha(meta);packages.append(vals)
  cfg=env/'pyvenv.cfg'
  inventory.append({'environment':str(env),'pyvenv_cfg':cfg.read_text(errors='replace') if cfg.exists() else None,'packages':packages})
 dump(O/'ENVIRONMENTS.json',inventory)
 # Exact byte duplicates can be restored from a retained copy; unique graphs stay.
 graphs=[p for p in allfiles if p.suffix=='.casadi' and not p.is_symlink()]
 graph_groups=collections.defaultdict(list)
 print('Hashing graphs',len(graphs),flush=True)
 for p in graphs:graph_groups[sha(p)].append(p)
 symlink_targets={p.resolve() for p in allfiles if p.is_symlink()}
 # Preserve explicit absolute graph references made by current formal bundles.
 refs=set()
 for p in allfiles:
  if not protected(p) or p.is_symlink() or p.suffix not in {'.json','.md','.sh','.cpp','.hpp','.txt'}:continue
  if p.stat().st_size>16*1024*1024:continue
  data=p.read_text(errors='replace')
  if '.casadi' not in data:continue
  for raw in re.findall(r'/home/[^\"\n\r]*?\.casadi',data):refs.add(Path(raw))
 deletes={};skips=[]
 for p in allfiles:
  if protected(p) or p in tr:continue
  s=p.lstat()
  if (s.st_dev,s.st_ino) in opened or p in symlink_targets:
   continue
  e=envroot(p)
  cat=None; evidence=None
  if e and e not in banned_envs:
   cat='snapshot_virtual_environment';evidence='ENVIRONMENTS.json; only copied source_snapshot/.runtime/teleop-venv'
  elif not p.is_symlink() and p.suffix=='.ulg':cat='old_flight_log'
  elif not p.is_symlink() and 'contact' in p.name.lower() and (p.name.endswith('.pbtxt') or p.name.endswith('.pbtxt.gz')):cat='old_contact_record'
  if cat:deletes[p]=(cat,evidence)
 for h,group in graph_groups.items():
  if len(group)<2:continue
  group.sort(key=lambda p:(not protected(p),p not in tr,p not in refs,str(p)))
  keeper=group[0]
  for p in group[1:]:
   if protected(p) or p in tr or p in refs or p in symlink_targets:continue
   s=p.stat()
   if (s.st_dev,s.st_ino) in opened:continue
   deletes[p]=('duplicate_casadi_graph',{'retained_copy':str(keeper),'sha256':h,'restore':'Copy retained_copy back to this exact path; byte-identical'})
  assert keeper not in deletes
 print('Candidate files',len(deletes),'hashing preservation inventory',flush=True)
 rows=[]
 for p,(cat,evidence) in sorted(deletes.items(),key=lambda z:str(z[0])):
  s=p.lstat();rows.append({'path':str(p),'experiment':exp(p),'category':cat,'evidence':evidence,'size':s.st_size,'blocks':s.st_blocks,'dev':s.st_dev,'ino':s.st_ino,'nlink':s.st_nlink,'mtime_ns':s.st_mtime_ns,'symlink':p.is_symlink(),'link_target':os.readlink(p) if p.is_symlink() else None,'sha256':None if p.is_symlink() else sha(p)})
 dump(O/'DELETE_MANIFEST.json',rows)
 with (O/'RETAINED_BEFORE.jsonl').open('w') as f:
  for i,p in enumerate(allfiles):
   if p in deletes:continue
   rec={'path':str(p),'symlink':p.is_symlink()}
   if p.is_symlink():rec['target']=os.readlink(p)
   else:rec.update(size=p.stat().st_size,sha256=sha(p))
   f.write(json.dumps(rec,ensure_ascii=False)+'\n')
 counts=collections.Counter();unique={};links=collections.Counter()
 for a in rows:
  counts[a['category']]+=1;k=(a['dev'],a['ino']);unique[k]=a;links[k]+=1
 estimate=collections.Counter()
 for k,a in unique.items():
  if links[k]>=a['nlink']:estimate[a['category']]+=a['blocks']*512
 summary={'state':'prepared','disk_before':disk(),'delete_files':len(rows),'retained_files':len(allfiles)-len(rows),'category_counts':counts,'estimated_released_bytes_by_category':estimate,'environment_copies':len(inventory),'protected_directories':[str(p) for p in PROTECTED],'unique_or_referenced_graphs_retained':len(graphs)-counts['duplicate_casadi_graph']}
 dump(O/'SUMMARY.json',summary)
 dump(O/'GIT_STATUS_BEFORE.json',subprocess.check_output(['git','-C',str(R),'status','--porcelain=v1','-z']).decode())
 print(json.dumps(summary,ensure_ascii=False),flush=True)

def apply():
 summary=json.loads((O/'SUMMARY.json').read_text());assert summary['state']=='prepared'
 assert summary.get('environment_copies_verified') is not None
 assert all(e.get('packages') for e in json.loads((O/'ENVIRONMENTS.json').read_text()) if e.get('cleanup_allowed'))
 rows=json.loads((O/'DELETE_MANIFEST.json').read_text());tr=tracked();opened,cwds=open_inodes();checked_envs=set()
 for a in rows:
  p=Path(a['path']);assert within(p,C) and not protected(p) and p not in tr
  s=p.lstat();assert (s.st_dev,s.st_ino,s.st_size,s.st_mtime_ns)==(a['dev'],a['ino'],a['size'],a['mtime_ns']),str(p)
  assert (s.st_dev,s.st_ino) not in opened,str(p)
  if a['category']=='duplicate_casadi_graph':assert sha(Path(a['evidence']['retained_copy']))==a['sha256']
  e=envroot(p)
  if e and e not in checked_envs:
   assert not any(within(Path(c),e) for c in cwds)
   checked_envs.add(e)
 summary['state']='deleting';dump(O/'SUMMARY.json',summary)
 with (O/'DELETED.jsonl').open('a') as f:
  for a in rows:
   p=Path(a['path']);p.unlink();f.write(json.dumps({'path':str(p),'category':a['category']},ensure_ascii=False)+'\n')
  f.flush();os.fsync(f.fileno())
 # Only remove empty directories inside the enumerated duplicate environments.
 for rec in json.loads((O/'ENVIRONMENTS.json').read_text()):
  e=Path(rec['environment'])
  if not e.exists():continue
  for d,ds,fs in os.walk(e,topdown=False):
   try:Path(d).rmdir()
   except OSError:pass
 print('Deletion finished. Verifying retained files.',flush=True)
 hashcache.clear();errors=[];checked=0
 for line in (O/'RETAINED_BEFORE.jsonl').open():
  a=json.loads(line);p=Path(a['path']);checked+=1
  try:
   if a['symlink']:ok=p.is_symlink() and os.readlink(p)==a['target']
   else:ok=not p.is_symlink() and p.stat().st_size==a['size'] and sha(p)==a['sha256']
   if not ok:errors.append(str(p))
  except OSError:errors.append(str(p))
 missing_fail=[a['path'] for a in rows if os.path.lexists(a['path'])]
 summary.update(state='complete' if not errors and not missing_fail else 'verification_failed',disk_after=disk(),retained_checked=checked,retained_changed=errors,delete_failed=missing_fail)
 summary['filesystem_free_increase_bytes']=summary['disk_after']['free']-summary['disk_before']['free'];dump(O/'SUMMARY.json',summary)
 experiments=collections.defaultdict(collections.Counter)
 for a in rows:experiments[a['experiment']][a['category']]+=1
 dump(O/'AFFECTED_EXPERIMENTS.json',experiments)
 text='# 实验产物清理记录（2026-11-25）\n\n按用户确认计划，仅清理 comparisons 内旧实飞日志、接触原始记录、源码快照内重复虚拟环境及有字节一致保留副本的旧 CasADi 图。\n\n最新完整 PVAJ、仅位置消融、最新 TOGT/Wang/修正版 CPC 与六方法汇总全部保留。源码、场景、配置、初值、最终轨迹、结果 JSON 和报告均保留。未知恢复来源的唯一计算图未删除。\n\n删除逐文件依据及 SHA-256：DELETE_MANIFEST.json；实际删除日志：DELETED.jsonl；被保留文件的事前 SHA-256：RETAINED_BEFORE.jsonl；版本清单：ENVIRONMENTS.json；核验结果与释放量：SUMMARY.json。\n\n## 历史原始证据可用性\n\n下列旧实验的 ULog 或接触记录已删除，保留的碰撞/失败/成功结论未改，但不能再从完整原始日志复核这些旧实飞。历史日志清单中对这些文件的哈希或路径不再代表文件仍可用。\n\n'
 for e,c in sorted(experiments.items()):
  if c['old_flight_log'] or c['old_contact_record']:text+=f'- `{e}`：ULog {c["old_flight_log"]}，接触记录 {c["old_contact_record"]}。\n'
 text+='\n## 计算图恢复\n\n每个被清理图的清单 evidence.retained_copy 指向未删除、SHA-256 完全一致的副本；复制回原路径即可恢复，不需要求解。恢复依赖应遵循各实验原配置。虚拟环境根据 ENVIRONMENTS.json 重建，不承诺二进制逐字节相同。\n'
 (O/'README.md').write_text(text)
 print(json.dumps(summary,ensure_ascii=False),flush=True)
if __name__=='__main__':{'prepare':prepare,'apply':apply}[sys.argv[1]]()
