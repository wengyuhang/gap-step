# Third-window time estimate: hardest scene only

1 Hz / 2 m / initial phase. One trial each. Normal convergence is required, not best-iterate admission.
Seven-gate success and strict return are separate. Sampled safety is not a certificate.

|Variant|Race passed|Race time/s|Body exits|Accepted/converged/total solves|Max planner/ms|Failure|
|---|---|---|---|---|---|---|
|ours|False|None|7|7/10/10|154.92204809561372|None|
|center|False|None|7|7/13/13|202.57148705422878|None|
|no_value|False|None|7|7/15/15|208.42296816408634|None|

Historical references, NOT matched ablations: old forced-center 24.428 s; full-course anytime 22.760 s.
An online failure is not evidence that the moving course or the reach-time idea is impossible.
