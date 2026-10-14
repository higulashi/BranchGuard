import argparse
import csv
from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
import os
from pathlib import Path
import platform
import sys
from branchguard.native import build, METHODS
from branchguard.simulator import run, SCENARIOS, CONDITIONS, DT

ROOT = Path(__file__).resolve().parent


def job(args):
    return run(*args)[0]


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--out', default='results/development')
    p.add_argument('--seed-start', type=int, default=11)
    p.add_argument('--seeds', type=int, default=4)
    p.add_argument('--workers', type=int, default=1)
    p.add_argument('--quick', action='store_true')
    p.add_argument('--methods', nargs='+', choices=METHODS, default=list(METHODS))
    args = p.parse_args()
    build()  # Compile before workers open the shared library.
    n = 1 if args.quick else args.seeds
    scenarios = SCENARIOS[:2] if args.quick else SCENARIOS
    conditions = ('clean', 'mixed') if args.quick else CONDITIONS
    jobs = [(s, c, seed, m) for s in scenarios for c in conditions
            for seed in range(args.seed_start, args.seed_start + n) for m in args.methods]
    dst = ROOT / args.out
    dst.mkdir(parents=True, exist_ok=True)
    protocol = {'scenarios': scenarios, 'conditions': conditions, 'seeds': list(range(args.seed_start,args.seed_start+n)),
                'methods': args.methods, 'dt': DT, 'jobs': len(jobs),
                'python': sys.version, 'platform': platform.platform(),
                'source_sha256': {str(f.relative_to(ROOT)).replace(os.sep,'/'): hashlib.sha256(f.read_bytes()).hexdigest()
                                  for f in sorted(ROOT.rglob('*')) if f.suffix in ('.py','.c','.h') and 'build' not in f.parts},
                'sensor_input': '188x120 uint8 image, route -1/0/+1, noisy encoder distance and gyro yaw increment',
                'success': 'reach world x >= 4.4 m on requested route; stops are failures',
                'note': 'All conditions and failures retained. This is a synthetic black-guide-line testbed.'}
    (dst / 'protocol.json').write_text(json.dumps(protocol, indent=2), encoding='utf-8')
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        results = []
        for i, result in enumerate(pool.map(job, jobs), 1):
            results.append(result)
            if i % 24 == 0 or i == len(jobs):
                print(f'{i}/{len(jobs)}', flush=True)
    with (dst / 'runs.csv').open('w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=results[0])
        writer.writeheader()
        writer.writerows(results)
    for method in args.methods:
        rows = [r for r in results if r['method'] == method]
        print(method, sum(r['status'] == 'complete' for r in rows), '/', len(rows), flush=True)


if __name__ == '__main__':
    main()
