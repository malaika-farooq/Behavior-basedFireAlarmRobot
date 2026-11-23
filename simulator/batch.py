"""Many simulated runs -> how often / how fast the fire is found.

    python simulator/batch.py                              lab map, 30 random pad+start layouts
    python simulator/batch.py --map lab --named            lab map, its named pads x starts x 2 seeds
    python simulator/batch.py --map houses:20              20 random houses, 3 layouts each
    python simulator/batch.py --map lab,rooms4,corridor --random 20
    python simulator/batch.py --set "EXPLORE='map'"        compare a setting
"""
import argparse, os, sys, time
from multiprocessing import Pool
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)


def one(job):
    import simcore, arena
    mp, pad, start, seed, limit, ov, src = job
    arena.use(mp)
    if pad == 'random':                      # seed = layout number
        p, st, _ = simcore.random_spots(seed)
        r = simcore.run([p], st, seed, limit, src=src, overrides=ov, logfile=None)
    else:
        r = simcore.run(pad, start, seed, limit, src=src, overrides=ov, logfile=None)
    return mp, pad, start, seed, r['found_s'], r['dist_m'], r['candle']


def jobs_for(maps, named, n_random, seeds, limit, ov, src):
    import arena
    jobs = []
    for mp in maps:
        if mp.startswith('houses:'):
            for h in range(1, int(mp.split(':')[1]) + 1):
                jobs += [('house:%d' % h, 'random', 'random', k, limit, ov, src) for k in range(1, 4)]
            continue
        m = arena.use(mp)
        if named and m['pads'] and m['starts']:
            jobs += [(mp, p, s, k, limit, ov, src) for p in m['pads'] for s in m['starts']
                     for k in range(1, seeds + 1)]
        else:
            jobs += [(mp, 'random', 'random', k, limit, ov, src) for k in range(1, n_random + 1)]
    return jobs


def evaluate(jobs, label, limit, quiet=True):
    t0 = time.time()
    with Pool(max(1, (os.cpu_count() or 2) - 1)) as pool:
        res = pool.map(one, jobs)
    times = [r[4] for r in res]
    ok = [t for t in times if t is not None]
    u5 = sum(1 for t in ok if t <= 300)
    mean = sum(t if t is not None else limit for t in times) / len(times)
    med = sorted(t if t is not None else 1e9 for t in times)[len(times) // 2]
    hit = sum(1 for r in res if r[4] is not None and r[6] is not None)
    print('%-40s under 5 min %3d%%  found %3d/%3d  mean %4.0f s  median %s  candle hit %d/%d   [%.0f s]' % (
        label[:40], 100 * u5 // len(times), len(ok), len(times), mean,
        '%.0f s' % med if med < 1e8 else '-', hit, len(ok), time.time() - t0), flush=True)
    if not quiet:
        for r in res:
            print('    %-10s %-10s %-8s #%-3d %s%s' % (r[0], r[1] if isinstance(r[1], str) else 'pad',
                  r[2] if isinstance(r[2], str) else '', r[3],
                  '%4.0f s' % r[4] if r[4] else 'not found',
                  '  CANDLE HIT (%s)' % r[6] if r[6] else ''))
    return res


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', default='lab', help='comma list: lab, lab_old, rooms4, corridor, house:N, houses:K')
    ap.add_argument('--named', action='store_true', help="use the map's named pads and starts")
    ap.add_argument('--random', type=int, default=30, help='random layouts per map')
    ap.add_argument('--seeds', type=int, default=2)
    ap.add_argument('--limit', type=int, default=600)
    ap.add_argument('--set', action='append', default=[])
    ap.add_argument('--src', default=None, help='another main.py file to test')
    ap.add_argument('-v', action='store_true', help='list every run')
    a = ap.parse_args()
    ov = dict(s.split('=', 1) for s in a.set)
    src = open(a.src).read() if a.src else None
    jobs = jobs_for(a.map.split(','), a.named, a.random, a.seeds, a.limit, ov, src)
    label = '%s %s %s' % (os.path.basename(a.src) if a.src else 'main.py', a.map, ' '.join(a.set))
    evaluate(jobs, label, a.limit, quiet=not a.v)
