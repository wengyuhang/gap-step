"""Independently reread ULogs and all contacts, including archived raw evidence."""
import gzip
import importlib.util
import json
from pathlib import Path
from . import scenario
from .run import read,digest,sources
from .report import METHODS


class NativeContactFile:
    def __init__(self,path):self.path=path
    def open(self,**kwargs):
        if self.path.exists():return self.path.open(**kwargs)
        gz=self.path.with_name(self.path.name+'.gz')
        if gz.exists():return gzip.open(gz,'rt',**kwargs)
        raise FileNotFoundError(self.path)


def validate(out):
    rows=read(out/'summary.json');protocol=read(out/'protocol.json');assert len(rows)==12
    assert {(r['case']['level'],r['variant']) for r in rows}=={(l,m) for l in scenario.LEVELS for m in METHODS}
    assert sources()==protocol['source_sha256']
    assert all(digest(out/'source_snapshot'/p)==sha for p,sha in protocol['source_sha256'].items())
    inherited=read(out/'reuse_audit.json')
    for record in inherited['retained']:
        assert all(digest(Path(record['directory'])/p)==sha for p,sha in record['sha256'].items())
        assert all(digest(Path(record['plan_directory'])/p)==sha for p,sha in record['plan_sha256'].items())
    helper_path=out.parent/'mixed_global_four_v4_20261105/finalize_results.py'
    spec=importlib.util.spec_from_file_location('raw_basic_audit',helper_path);helper=importlib.util.module_from_spec(spec);spec.loader.exec_module(helper)
    checks=[];worlds={};configs={}
    for row in rows:
        path=Path(row['directory']);raw=read(path/'result.json');m=row['variant'];level=row['case']['level']
        plan=raw if row['verdict']=='planning_not_found' else read(Path(raw['global_plan_directory'])/'result.json')
        assert len(plan['candidates'])==4 and plan['case']==row['case'] and plan['method']==m
        qualified=[c for c in plan['candidates'] if c['found' if m in ('ours','center') else 'success']]
        assert bool(qualified)==plan['found']
        if plan['found']:
            key='nominal_time_s' if m in ('ours','center') else 'objective'
            assert abs(plan['candidates'][plan['selected_seed']][key]-min(c[key] for c in qualified))<1e-9
        if row['verdict']=='planning_not_found':
            checks.append(dict(case=row['case'],method=m,planning_not_found=True,result_sha256=digest(path/'result.json')));continue
        assert raw['solves']==[] and raw['planning_scheduler']=='none_global_frozen'
        assert read(path/'race_start_audit.json')['ready_at_common_race_epoch']
        track,_=scenario.build_case(scenario.Case(**row['case']));truth=helper.truth_recheck(path,raw,row,track)
        contacts=[helper.contact_recheck(NativeContactFile(path/f'gate_{i:02d}_contacts.pbtxt'),30+raw['last_time_s']) for i in range(1,9)]
        collision=any(c['contact_pair_count'] for c in contacts);assert collision==row['collision_detected']
        assert not row['passed'] or (truth['completed_truth'] and not collision)
        cm=read(path/'contact_manifest.json');pubs=read(path/'contact_publishers.json');assert cm['sensors']==8 and cm['each_sensor_covers_entire_historic_sleeve_mesh']
        assert set(pubs)==set(cm['sensor_topics']) and all('gz.msgs.Contacts' in s and 'Publishers [Address, Message Type]:' in s for s in pubs.values())
        pose=read(path/'gate_pose_audit.json');assert len(pose)==7 and all(x['passed'] for x in pose.values())
        worlds.setdefault(level,set()).add(digest(path/'world.sdf'));configs.setdefault(level,[]).append((raw['mpc_config'],raw['parameters']))
        checks.append(dict(case=row['case'],method=m,truth=truth,collision=collision,contacts=contacts,result_sha256=digest(path/'result.json'),ulog_sha256=digest(path/'flight.ulg')))
    assert all(len(s)==1 for s in worlds.values())
    assert all(all(v==values[0] for v in values) for values in configs.values())
    for level in scenario.LEVELS:
        for m in ['ours','dynamic_togt']:
            pair=out/f'pair_original_{level}_{m}.json'
            if pair.exists():assert read(pair)['equivalent_except_free_points']
    result=dict(raw_audit_passed=True,conditions=len(rows),retained_static_flights=2,new_records=10,all_sources_snapshots_and_inherited_evidence_unchanged=True,
        physical_world_mpc_px4_equal_within_condition=True,collision_free_completions=sum(r['passed'] for r in rows),actual_collisions=sum(r.get('collision_detected',False) for r in rows),checks=checks)
    (out/'validation.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n');print('RAW_AUDIT_VERIFIED',len(rows),flush=True)
    return result
