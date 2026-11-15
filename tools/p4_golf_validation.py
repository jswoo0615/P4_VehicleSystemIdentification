#!/usr/bin/env python3
"""Create a traceable P4 Golf validation report from GOLF-B0-B1-KFOLD-001 JSON.
No refitting. No blending full-data coefficients into held-out metrics.
"""
import argparse, hashlib, json, math
from pathlib import Path

def rmse_weighted(folds, model, key):
    # Per-fold sample counts are not present in the given schema; avoid pooling fold RMSEs.
    return None

def build_report(src):
    d=json.loads(src.read_text(encoding='utf-8'))
    assert d['experiment']=='GOLF-B0-B1-KFOLD-001', 'Unexpected experiment'
    folds=d['fold_results']
    assert len(folds)==d['folds']==5
    names=[n for f in folds for n in f['test_files']]
    assert len(names)==d['unique_files']==20 and len(set(names))==20, 'Invalid grouped split'
    for f in folds:
        assert len(f['file_results'])==len(f['test_files'])
        assert {q['file'] for q in f['file_results']}==set(f['test_files'])
        assert all(q['b0_yaw_rmse_radps']>=0 for q in f['file_results'])
    pooled=d['pooled']; a=pooled['a_speed_refined_yaw_rmse_radps']; b0=pooled['b0']; b1=pooled['b1']
    files=[]
    for f in folds:
        for q in f['file_results']:
            files.append({'fold':f['fold'], 'file':q['file'],
                          'b0_yaw_rmse_radps':q['b0_yaw_rmse_radps'],
                          'b0_vy_rmse_mps':q['b0_vy_rmse_mps'],
                          'b1_yaw_rmse_radps':q['b1_yaw_rmse_radps']})
    files.sort(key=lambda z:z['b0_yaw_rmse_radps'],reverse=True)
    wins=sum(f['b0']['yaw_rmse_radps']<f['b1']['yaw_rmse_radps'] for f in folds)
    full=d['full_fit_reference']['b0']; L=d['wheelbase_m']
    kv=((L-full['lf_m'])/full['cf_over_m']-full['lf_m']/full['cr_over_m'])/L
    report={
       'schema':'p4.validation_summary.v1', 'experiment':d['experiment'],
       'input_sha256':hashlib.sha256(src.read_bytes()).hexdigest(),
       'data':{'unique_files':len(names),'folds':len(folds),'cv_samples_reported':d.get('cv_samples'),
               'sample_period_s':d['sample_period_s'],'whole_file_holdout':True,
               'file_name_uniqueness_checked':True,
               'note':'File names checked once per held-out fold; source-file byte-identity not reaudited here.'},
       'metrics':{'b0_yaw_rmse_radps':b0['yaw_rmse_radps'],'b0_vy_rmse_mps':b0['vy_rmse_mps'],
                  'b1_yaw_rmse_radps':b1['yaw_rmse_radps'],'b1_vy_rmse_mps':b1['vy_rmse_mps'],
                  'a_speed_refined_yaw_rmse_radps':a,
                  'b0_vs_refined_a_speed_improvement_percent':100*(a-b0['yaw_rmse_radps'])/a,
                  'b1_vs_b0_yaw_degradation_percent':100*(b1['yaw_rmse_radps']/b0['yaw_rmse_radps']-1),
                  'b0_wins_vs_b1_folds':wins},
       'worst_b0_files':files[:5],
       'full_data_reference_NOT_cv':{'parameters_normalized':full,'implied_kv_s2_per_m':kv,
              'warning':'Full-data fit is not a held-out estimate; the pooled CV metrics do not validate this exact fitted parameter vector.'},
       'assessment':{'reference_model':'B0','b1_adopted':False,
                     'physical_parameter_identifiability':'limited for lf and Iz/m',
                     'coordinate_convention':'bank sign unverified against original dataset documentation',
                     'validity_range':'Not inferred from this CV JSON; use golf_baselines.json observed domain',
                     'export_ready':'requires baseline source file + controller interface verification'},
    }
    return report

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('cv_json', type=Path)
    ap.add_argument('out_json', type=Path)
    args=ap.parse_args()
    rep=build_report(args.cv_json)
    args.out_json.parent.mkdir(parents=True,exist_ok=True)
    args.out_json.write_text(json.dumps(rep,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    print('Saved',args.out_json)
    print('B0 vs refined A-Speed: %.3f%% improvement'%rep['metrics']['b0_vs_refined_a_speed_improvement_percent'])
    print('Worst held-out file:',rep['worst_b0_files'][0]['file'])
if __name__=='__main__': main()
