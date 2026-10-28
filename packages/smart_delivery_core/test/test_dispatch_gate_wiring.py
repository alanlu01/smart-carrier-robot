"""Exercise the actual nested predicate instead of copying its formula."""
import ast
from pathlib import Path
from types import SimpleNamespace
from smart_delivery_core.delivery_state import localization_ready_for_resume


def test_persistent_stationary_mismatch_blocks_goal_even_while_ready_is_true():
    path=Path(__file__).parents[1]/'smart_delivery_core/smart_delivery.py'
    main=next(n for n in ast.parse(path.read_text(encoding='utf-8')).body if isinstance(n,ast.FunctionDef) and n.name=='_run_delivery')
    predicate=next(n for n in main.body if isinstance(n,ast.FunctionDef) and n.name=='localization_can_resume')
    unit=ast.Module(body=[predicate],type_ignores=[])
    context={'localization_dispatch_allowed':False,'localization_ready':True,
        'localization_ready_since':90.,'localization_resume_delay':1.,
        'time':SimpleNamespace(monotonic=lambda:100.),
        'localization_ready_for_resume':localization_ready_for_resume}
    exec(compile(ast.fix_missing_locations(unit),str(path),'exec'),context)
    assert not context['localization_can_resume']()
    context['localization_dispatch_allowed']=True
    assert context['localization_can_resume']()
    context['localization_ready']=False
    assert not context['localization_can_resume']()
