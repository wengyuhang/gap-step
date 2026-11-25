import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

MODULE=Path(__file__).resolve().parents[1]/'finite_recovery.py'
spec=importlib.util.spec_from_file_location('finite_recovery_under_test',MODULE)
recovery=importlib.util.module_from_spec(spec);spec.loader.exec_module(recovery)


class TestFiniteRecovery(unittest.TestCase):
    def planner(self):
        return SimpleNamespace(optimizer_calls=[],_fast_warm={'bad_duals':True},
            _fast_problem='old_graph',_fast_cold_problem='old_cold_graph')

    def solver(self,success,seen):
        def minimize(planner,fun,initial,**kwargs):
            seen.append((initial,dict(planner.__dict__)))
            planner.optimizer_calls.append(dict(status='Solve_Succeeded' if success else 'Restoration_Failed'))
            return SimpleNamespace(success=success,x=initial+1,verification_ready=True)
        return SimpleNamespace(minimize=minimize)

    def test_success_does_not_retry(self):
        planner=self.planner();seen=[]
        with patch.object(recovery,'import_module',return_value=self.solver(True,seen)):
            result=recovery.minimize(planner,None,10)
        self.assertTrue(result.success);self.assertEqual(len(seen),1)

    def test_failed_duals_discarded_and_primal_retained(self):
        planner=self.planner();first=[];second=[]
        with patch.object(recovery,'import_module',side_effect=[self.solver(False,first),self.solver(True,second)]) as imports:
            result=recovery.minimize(planner,None,10)
        self.assertTrue(result.success);self.assertEqual(second[0][0],11)
        self.assertNotIn('_fast_warm',second[0][1]);self.assertNotIn('_fast_problem',second[0][1])
        self.assertTrue(planner._prefer_exact_hessian)
        self.assertEqual(imports.call_args_list[-1].args,('finite_direct_exact',))
        self.assertEqual(len(planner.optimizer_calls),2)

    def test_both_failures_end_attempt_instead_of_repeating_outer_loop(self):
        planner=self.planner();seen=[]
        with patch.object(recovery,'import_module',return_value=self.solver(False,seen)):
            result=recovery.minimize(planner,None,10)
        self.assertFalse(result.success);self.assertFalse(result.verification_ready)
        self.assertEqual(len(seen),2);self.assertNotIn('_fast_warm',planner.__dict__)


if __name__=='__main__':
    unittest.main()
