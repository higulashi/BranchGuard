import argparse
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path
import statistics as stats
from PIL import Image, ImageDraw
from make_demo import font

ROOT=Path(__file__).resolve().parent
METHOD_ORDER=('nearest','greedy_route','spatial','temporal','branchguard','no_motion','no_prefix','spatial_slow','branchguard_slow')
PALETTE={'complete':'#57c4a9','wrong_branch':'#e57986','off_track':'#eeae5b','stopped':'#8f91a8','timeout':'#526d83'}


def read_rows(folder):
    with (folder/'runs.csv').open(encoding='utf-8',newline='') as f:
        rows=list(csv.DictReader(f))
    for r in rows:
        for key in ('time_s','progress_m','rms_error_m','max_error_m','steering_tv_rad'):
            r[key]=float(r[key])
    return rows


def key(row):
    return row['scenario'],row['condition'],row['seed']


def pair(rows,a,b):
    aa={key(r):r for r in rows if r['method']==a}
    bb={key(r):r for r in rows if r['method']==b}
    keys=sorted(aa.keys() & bb.keys())
    both=[k for k in keys if aa[k]['status']==bb[k]['status']=='complete']
    return {'pairs':len(keys),
            'a_only':sum(aa[k]['status']=='complete' and bb[k]['status']!='complete' for k in keys),
            'b_only':sum(bb[k]['status']=='complete' and aa[k]['status']!='complete' for k in keys),
            'both_complete':len(both),
            'median_time_a':stats.median(aa[k]['time_s'] for k in both) if both else None,
            'median_time_b':stats.median(bb[k]['time_s'] for k in both) if both else None,
            'median_time_delta':stats.median(aa[k]['time_s']-bb[k]['time_s'] for k in both) if both else None,
            'median_rms_a':stats.median(aa[k]['rms_error_m'] for k in both) if both else None,
            'median_rms_b':stats.median(bb[k]['rms_error_m'] for k in both) if both else None,
            'median_rms_delta':stats.median(aa[k]['rms_error_m']-bb[k]['rms_error_m'] for k in both) if both else None,
            'median_tv_delta':stats.median(aa[k]['steering_tv_rad']-bb[k]['steering_tv_rad'] for k in both) if both else None}


def summarize(rows):
    methods=[m for m in METHOD_ORDER if any(r['method']==m for r in rows)]
    result={}
    for method in methods:
        subset=[r for r in rows if r['method']==method]
        statuses=dict(Counter(r['status'] for r in subset))
        result[method]={'total':len(subset),'statuses':statuses,
                        'success_rate':statuses.get('complete',0)/len(subset)}
    return result


def chart(rows,dest):
    groups=summarize(rows)
    conditions=list(dict.fromkeys(r['condition'] for r in rows))
    image=Image.new('RGB',(1180,180+len(groups)*46+len(conditions)*80),'#111b26')
    d=ImageDraw.Draw(image)
    d.text((30,23),'BranchGuard | frozen-seed evaluation',font=font(27),fill='#f5f7fa')
    d.text((30,65),'Synthetic guide-line decisions. Stops and route errors count as failures.',font=font(17),fill='#a6b7c9')
    top=115
    for m,g in groups.items():
        d.text((30,top),m,font=font(18),fill='#f5f7fa')
        left=225
        for status,color in PALETTE.items():
            count=g['statuses'].get(status,0)
            width=650*count/g['total']
            if width: d.rectangle((left,top,left+width,top+25),fill=color)
            left+=width
        d.text((905,top),f'{g["statuses"].get("complete",0)}/{g["total"]}',font=font(18),fill='#f5f7fa')
        top+=46
    left=30
    for status,color in PALETTE.items():
        d.rectangle((left,top+4,left+12,top+16),fill=color)
        d.text((left+18,top),status,font=font(14),fill='#a6b7c9')
        left+=190
    top+=50
    d.text((30,top),'Completion by condition (same seeds and routes)',font=font(20),fill='#f5f7fa')
    top+=37
    for condition in conditions:
        d.text((30,top),condition,font=font(16),fill='#a6b7c9')
        ss=summarize([r for r in rows if r['condition']==condition])
        for i,(m,g) in enumerate(ss.items()):
            left=185+i*108
            d.text((left,top),f'{g["success_rate"]:.0%}',font=font(18),fill='#57c4a9')
            d.text((left,top+25),m[:12],font=font(11),fill='#a6b7c9')
        top+=64
    image.save(dest)


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--input',default='results/confirmation')
    a=p.parse_args()
    folder=ROOT/a.input
    rows=read_rows(folder)
    summary=summarize(rows)
    comparisons={f'{a} vs {b}':pair(rows,a,b) for a,b in (
        ('branchguard','spatial'),('branchguard','temporal'),('branchguard','no_motion'),
        ('branchguard','no_prefix'),
        ('branchguard','spatial_slow'),('branchguard_slow','spatial_slow'))}
    report={'methods':summary,'paired_comparisons':comparisons,
            'by_condition':{c:summarize([r for r in rows if r['condition']==c]) for c in dict.fromkeys(r['condition'] for r in rows)},
            'flicker_pair':pair([r for r in rows if r['condition']=='flicker'],'branchguard','spatial'),
            'runs_sha256':hashlib.sha256((folder/'runs.csv').read_bytes()).hexdigest()}
    (folder/'summary.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    lines=['# 仿真实验','',f'本报告由 `{a.input}/runs.csv` 生成。逐次结果包括全部失败运行。','',
           '## 总体结果','', '| 方法 | 完成指定路线 | 错误分支 | 出界 | 停止／超时 |', '|---|---:|---:|---:|---:|']
    for m,g in summary.items():
        s=g['statuses']
        lines.append(f'| `{m}` | {s.get("complete",0)}/{g["total"]}（{g["success_rate"]:.1%}） | {s.get("wrong_branch",0)} | {s.get("off_track",0)} | {s.get("stopped",0)+s.get("timeout",0)} |')
    lines+=['','## 配对比较','',
            '每一对使用同一赛道、干扰和种子。下表的耗时与误差仅比较双方都完成的运行；提前失败不会被当成低耗时或低误差。差值为前者减后者。','',
            '| 对比 | 仅前者完成 | 仅后者完成 | 共同完成 | 耗时差中位数（s） | RMS 差中位数（m） | 转向总变化差（rad） |',
            '|---|---:|---:|---:|---:|---:|---:|']
    for label,c in comparisons.items():
        def fmt(x): return '—' if x is None else f'{x:+.4f}'
        lines.append(f'| {label} | {c["a_only"]} | {c["b_only"]} | {c["both_complete"]} | {fmt(c["median_time_delta"])} | {fmt(c["median_rms_delta"])} | {fmt(c["median_tv_delta"])} |')
    lines+=['','## 按干扰划分','', '| 条件 | '+' | '.join(summary)+' |', '|---|'+'---:|'*len(summary)]
    for condition,groups in report['by_condition'].items():
        lines.append('| '+condition+' | '+' | '.join(f'{groups[m]["statuses"].get("complete",0)}/{groups[m]["total"]}' for m in summary)+' |')
    lines+=['','## 实验协议','',
            '- 四类场景：左岔路、右岔路、三分支直行和斜线交叉。每条路线从 x≈0.35 m 行驶到 x≥4.4 m。',
            '- 五类条件：正常、静态干扰、缺失线段、短时错误线曝光、混合干扰。错误曝光持续 80 ms，整幅可见引导线横向错位 0.15 m，是人为设定的感知扰动。',
            '- 单帧灰度图为 188×120。C 核心扫描 12 行，每行最多 6 个候选点，束宽 24，最终保留最多 8 条路径。',
            '- 车辆采用运动学自行车模型，轴距 0.26 m，包含 70–130 ms 舵机一阶滞后和速度一阶响应。编码器比例偏差、陀螺仪偏差及增量噪声按种子固定。',
            '- 控制器得到灰度图、路线指令和带噪声的相对运动；不读取世界中心线、真实分支编号或真实车身位姿。世界几何只用于渲染和评分。',
            '- 正常速度建议上限为 0.95 m/s；待确认时为 0.55 m/s，短丢线保持时为 0.30 m/s。所有方法共享同一曲率限速。两个 `*_slow` 方法另设统一 0.55 m/s 上限。',
            '- 横向偏差超过 0.28 m 并持续 0.20 s 判为出界；在分岔点后 0.8 m 处，错误分支距离比目标分支小至少 0.12 m 判为错误分支；20 s 内未到终点判为超时。停止不算成功。',
            '- `rms_error_m` 与 `steering_tv_rad` 的运行级原始值可能来自不同长度的失败轨迹，不能直接用它们的整体平均判断优劣。',
            '', '## 适用范围','',
            '这是合成黑色引导线仿真。它不代表传统竞赛白底黑边赛道的识别成绩，也没有测试真实摄像头、环岛、T 形急转、多路口导航、侧滑或 MCU 的运行时间。左右指令由上层给出，本模块不规划全局路线。',
            '', '场景共享几何模板，同一种子下运行之间相关。这里报告配对计数和描述性指标，不把几百次模拟当成几百个独立的实车样本。短时错误曝光用于检验换线确认规则；该规则阈值若与实际错误持续时间不匹配，仍会失效。','',
            f'原始 CSV SHA-256：`{report["runs_sha256"]}`。代码哈希、版本和种子见同目录的 `protocol.json`。','']
    docs=ROOT/'docs';docs.mkdir(exist_ok=True)
    (docs/'EXPERIMENTS.zh-CN.md').write_text('\n'.join(lines),encoding='utf-8')
    assets=ROOT/'assets';assets.mkdir(exist_ok=True)
    chart(rows,assets/'evaluation.png')
    print(json.dumps(report,ensure_ascii=False,indent=2))


if __name__=='__main__':
    main()
