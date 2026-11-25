"""First actual contact and an independent finer run; no safety inference from plots."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import numpy as np
from .pendulum.motion import build,Case

HERE=Path(__file__).resolve().parent

def main():
    primary=HERE/'results/pendulum_final_v2_20261105/angle60_high_mpc_author';fine=HERE/'results/pendulum_first_contact_resolution_v1_20261105';w=build(Case(60)).windows[0]
    fig,axes=plt.subplots(1,2,figsize=(9,5.2));records=[]
    for ax,path,label in zip(axes,(primary,fine),('1 ms: first frame contact','0.5 ms: centre-plane crossing')):
        row=json.loads((path/'comparison_audit.json').read_text());raw=json.loads((path/'result.json').read_text());tr=np.load(path/'mechanism_trace.npz')
        contact=row['contacts'][0]['first_contact_sim_time_s'];at=contact-30 if contact is not None else row['crossings'][0]['time_s']
        p=np.array([np.interp(at,tr['times'],tr['positions'][:,k]) for k in range(3)]);q,z=w.world_to_local(p,at);section=np.sqrt(max(0.,raw['config']['radius']**2-z*z))
        ax.add_patch(Circle((0,0),w.aperture.radius,fill=False,color='black',linewidth=2,label='Opening'))
        ax.add_patch(Circle(q,section,facecolor='#167d9a',alpha=.2,label='Sphere proxy section'))
        ax.scatter(*q,marker='x',color='#b82f2f',label='Measured body centre');ax.scatter(0,0,marker='+',color='black')
        ax.set(xlim=(-1.2,1.2),ylim=(-1.2,1.2),aspect='equal',xlabel='Instantaneous gate axis 1 (m)',ylabel='Instantaneous gate axis 2 (m)',title=label);ax.grid(alpha=.2)
        records.append(dict(directory=str(path),event_task_time_s=at,event='first_native_frame_contact' if contact is not None else 'actual_centre_crossing',point=q.tolist(),normal_offset_m=z,body_sphere_plane_section_radius_m=float(section),frame_contact_pairs=row['contacts'][0]['contact_pair_count']))
    for x,record in zip((.27,.76),records):fig.text(x,.025,f't={record["event_task_time_s"]:.3f}s; normal offset={record["normal_offset_m"]:.3f}m\nWhole-run frame contact pairs: {record["frame_contact_pairs"]}',ha='center',fontsize=9)
    axes[0].legend(fontsize=8,loc='upper left');fig.suptitle('60-degree High-MPC adaptation: two independent closed-loop runs\nSphere sections illustrate geometry; physical contacts determine outcomes',fontsize=11);fig.tight_layout(rect=(0,.20,1,.94));fig.savefig(primary.parent/'first_contact_and_independent_check.png',dpi=190);plt.close(fig)
    (primary.parent/'first_contact_mechanism.json').write_text(json.dumps(records,indent=2)+'\n')

if __name__=='__main__':main()
