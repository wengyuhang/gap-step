"""Initialize SIP only over the actual body/plane overlap of its own seed."""
import numpy as np
from safe_model import Planner as Base
from convex_timevarying_window.dynamic_passage_minco.safety import overlap_intervals
class Planner(Base):
    def witnesses(self):
        points=super().witnesses()
        if not hasattr(self,'initial_for_witnesses'):return points
        x=self.initial_for_witnesses;T=np.exp(x[:self.n]);C=np.asarray(self.curve_values(x)).ravel().reshape(self.n,3,8).transpose(0,2,1);radius=self.cfg.radius+self.cfg.frame_thickness+self.cfg.passage_margin+self.cfg.witness_slack
        for gate,w in enumerate(self.track.windows):
            for piece in [gate,gate+1]:
                coeff=C[piece]@w.R;coeff[0]-=w.center0@w.R
                ranges=overlap_intervals(coeff[:,2],radius)
                primary=[(a,b) for a,b in ranges if (b>1-1e-7 if piece==gate else a<1e-7)]
                kind='before' if piece==gate else 'after';values=[0.]
                for a,b in primary:
                    for u in np.linspace(a,b,7):values.append(float((1-u)*T[piece] if kind=='before' else u*T[piece]))
                points[(gate,piece)]=dict(kind=kind,values=sorted(set(values)))
        return points
