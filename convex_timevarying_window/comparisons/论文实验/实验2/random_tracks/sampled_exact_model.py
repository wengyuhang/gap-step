from sampled_safe_model import Planner as Base
class Planner(Base):
    def constrained_minimize(self,fun,initial,**kwargs):
        import finite_direct_exact
        return finite_direct_exact.minimize(self,fun,initial,**kwargs)
