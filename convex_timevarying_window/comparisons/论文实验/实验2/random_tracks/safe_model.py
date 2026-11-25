from repair import Planner as Base
class Planner(Base):
    def constrained_minimize(self,fun,initial,**kwargs):
        import finite_recovery
        return finite_recovery.minimize(self,fun,initial,**kwargs)
