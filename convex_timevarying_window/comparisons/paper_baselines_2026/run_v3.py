"""Run ONLY V3 on the existing 16 conditions; retain all integration failures."""
import argparse
import contextlib
import hashlib
import io
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys
import time
import numpy as np
from .run import summarize


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--outdir",type=Path,required=True)
    args = parser.parse_args(); args.outdir.mkdir(parents=True,exist_ok=False)
    root = Path("convex_timevarying_window/dynamic_gate_mppi_v3")
    sources = [p for p in root.rglob("*") if p.is_file() and p.suffix in (".py",".cpp",".msg",".xml",".txt")
               and not any(part in p.relative_to(root).parts for part in ("results","build","install","log"))]
    sources += [Path(__file__).resolve().relative_to(Path.cwd()),Path("convex_dynamic_seven_window_gazebo/x500_togt/native/togt_x500.cpp"),
                Path("convex_timevarying_window/togt/native_objective.py"),
                Path("convex_timevarying_window/receding_gate_racing/scenario.py")]
    hashes = {str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    for p in sources:
        dest = args.outdir/"source_snapshot"/p
        dest.parent.mkdir(parents=True,exist_ok=True); shutil.copy2(p,dest)
    conditions = [(*b,phase) for b in [(r,s,s) for r in (1,10) for s in (1,2,4)]+[(10,4,1),(10,1,4)]
                  for phase in (0.,2*math.pi/3)]
    protocol = {"conditions":conditions,"source_sha256":hashes,"algorithm":"user PDF V3 independent implementation",
                "baseline_rerun":False,"flight_limit_s":60.,"preparation_s":30.,
                "execution":"PX4 direct ActuatorMotors; preparatory mode handover starts at 29 s",
                "development_status":"Direct motor transport validated; successful racing NOT established",
                "evidence":"Frozen integration evaluation, not author-method ranking or safety certificate"}
    (args.outdir/"protocol.json").write_text(json.dumps(protocol,indent=2)+"\n")
    rows = []; start = time.time()
    def status(state, index=None):
        (args.outdir/"status.json").write_text(json.dumps({"state":state,"case":index,"finished":len(rows),
            "total":16,"started_unix":start,"updated_unix":time.time()},indent=2)+"\n")
    status("running",0)
    for index,(rate,translation,rotation,phase) in enumerate(conditions):
        for p in sources:
            if hashlib.sha256(p.read_bytes()).hexdigest()!=hashes[str(p)]:
                status("frozen_source_changed",index); raise RuntimeError(f"Frozen source changed: {p}")
        attempts = []
        for attempt in range(2):
            name = f"case_{index:02d}_r{rate}_t{translation}_a{rotation}"+("_arm_retry" if attempt else "")
            path = args.outdir/name
            cmd = [sys.executable,"-u","-m","convex_timevarying_window.dynamic_gate_mppi_v3.px4",
                "--outdir",str(path),"--motion-rate",str(rate),"--translation-scale",str(translation),
                "--rotation-scale",str(rotation),"--phase",str(phase),"--max-flight-time","60"]
            print("START",index,attempt,flush=True); tick=time.perf_counter(); status("running",index)
            with (args.outdir/(name+".log")).open("w") as console:
                try:
                    process = subprocess.run(cmd,stdout=console,stderr=subprocess.STDOUT,timeout=200)
                    returncode = process.returncode
                except subprocess.TimeoutExpired: returncode = None
            if (path/"result.json").exists():
                try: row=summarize(path)
                except Exception as error:
                    result=json.loads((path/"result.json").read_text())
                    row={"completed":result["completed"],"failure":result["failure"],"gates":len(result["crossings"]),
                         "physical_success":False,"realtime_success":False,"audit_error":repr(error)}
                result=json.loads((path/"result.json").read_text())
                ms=np.array([s["cycle_ms"] for s in result["solves"] if s.get("status")=="direct_motor_mppi"])
                row["mppi_over_20ms"]=int(np.sum(ms>20));row["mppi_cycles"]=len(ms)
                row["replan_returns"]=len(result["reference_updates"])
            else:
                row={"completed":False,"failure":"missing_result","physical_success":False,"realtime_success":False}
            row.update(case=index,attempt=attempt,run=str(path),returncode=returncode,wall_s=time.perf_counter()-tick,
                       motion_rate=rate,translation_scale=translation,rotation_scale=rotation,phase=phase)
            attempts.append(row)
            print("RESULT",json.dumps(row),flush=True)
            if "arm failed" not in str(row.get("failure")): break
        rows.append({**attempts[-1],"attempts":attempts})
        (args.outdir/"summary.json").write_text(json.dumps(rows,indent=2)+"\n")
    status("finished")
    print("DONE",str(args.outdir),flush=True)


if __name__=="__main__": main()
