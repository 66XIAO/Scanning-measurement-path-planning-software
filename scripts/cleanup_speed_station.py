"""User-authorized cleanup of exact retired experiment namespaces after backup.

No source CAD, robot, frame, tool, original A targets, or shared live targets
are selected. A full station backup and an incremental deletion ledger persist.
"""
import hashlib
import json
import sys
from collections import Counter
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import robodk_bridge as b

RETIRED = {
    'SpeedV1_Import_20260917_204433',
    'SpeedV1_Continuous_01',
    'V1_B_smooth_fixed_20260917_235523',
    'V1_C_smooth_variable_20260917_235523',
    'V1_Joint_20260918_000543',
    'Probe_stop_20260918_075718',
    'Probe_once_20260918_075718',
    'Probe_each_rounding_20260918_075718',
    'Probe_each_speed_rounding_20260918_075718',
}


def main():
    folder=Path(sys.argv[1]).resolve()
    backup=folder/'before_cleanup.rdk'
    if not backup.is_file() or backup.stat().st_size<1000:
        raise ValueError('Verified nonempty full station backup required')
    inventory=json.loads((folder/'inventory.json').read_text(encoding='utf-8'))
    api=b._import_api(); r=api['Robolink']()
    if r.ActiveStation().Name()!=inventory['station']:
        raise ValueError('Active station changed')
    programs=r.ItemList(8,True); targets=r.ItemList(6,True)
    retired_targets=[name for name in inventory['targets'] if name.rsplit('_P',1)[0] in RETIRED]
    if len(targets)!=len(set(targets)) or len(programs)!=len(set(programs)):
        raise ValueError('Duplicate names prevent unambiguous cleanup')
    missing_programs=set(inventory['programs'])-set(programs)
    missing_targets=set(inventory['targets'])-set(targets)
    if (set(programs)-set(inventory['programs']) or set(targets)-set(inventory['targets'])
            or not missing_programs <= RETIRED or not missing_targets <= set(retired_targets)):
        raise ValueError('Unexpected station change since backup')
    for name in programs:
        if r.Item(name,8).Busy(): raise RuntimeError('Program still running: '+name)
    delete_targets=[name for name in targets if name in set(retired_targets)]
    keep_programs=[name for name in programs if name not in RETIRED]
    protected_types={kind:r.ItemList(kind,True) for kind in [2,3,4,5,10,13,15]}
    log={'backup':str(backup),'backup_sha256':hashlib.sha256(backup.read_bytes()).hexdigest(),
         'programs_before':len(inventory['programs']),'targets_before':len(inventory['targets']),
         'retired_programs':sorted(RETIRED),'retired_targets':retired_targets,
         'kept_programs':keep_programs,'deleted_programs':sorted(missing_programs),'deleted_targets':sorted(missing_targets),
         'shared_target_namespaces_preserved':['A','V1_A_stop_20260917_235523','V1_Joint_20260918_080248','V1_Joint_20260918_182439']}
    ledger=folder/'cleanup_report.json'
    def save(): ledger.write_text(json.dumps(log,ensure_ascii=False,indent=2),encoding='utf-8')
    save()
    # Snapshot handles once. Names were proven unique; verify exact names again
    # before mutation, without a fresh full-station scan for each deletion.
    target_handles={item.Name():item for item in r.ItemList(6)}
    program_handles={item.Name():item for item in r.ItemList(8)}
    try:
        r.Render(False)
        for name in sorted(RETIRED-set(missing_programs)):
            item=program_handles[name]
            if item.Name()!=name: raise RuntimeError('Program identity changed')
            item.Delete()
            log['deleted_programs'].append(name); save()
        for i,name in enumerate(delete_targets):
            item=target_handles[name]
            if item.Name()!=name: raise RuntimeError('Target identity changed')
            item.Delete()
            log['deleted_targets'].append(name)
            if (i+1)%100==0:
                save(); print('Deleted targets',i+1,'/',len(delete_targets),flush=True)
        save()
    finally:
        r.Render(True)
    remaining_programs=r.ItemList(8,True); remaining_targets=r.ItemList(6,True)
    if set(remaining_programs)!=set(keep_programs): raise RuntimeError('Unexpected program inventory')
    if set(remaining_targets)!=(set(targets)-set(delete_targets)): raise RuntimeError('Unexpected target inventory')
    expected=Counter(inventory['items'])-Counter(RETIRED)-Counter(retired_targets)
    if Counter(r.ItemList(list_names=True))!=expected:
        raise RuntimeError('Full item inventory differs from the authorized deletion set')
    for kind,names in protected_types.items():
        if r.ItemList(kind,True)!=names: raise RuntimeError('Protected station objects changed')
    log.update(programs_after=len(remaining_programs),targets_after=len(remaining_targets),
               protected_objects_unchanged=True,remaining_programs=remaining_programs)
    save()
    r.Save(str(folder/'after_cleanup.rdk'))
    print(json.dumps({'programs_after':len(remaining_programs),'targets_after':len(remaining_targets),
                      'report':str(ledger)}),flush=True)


if __name__=='__main__': main()
