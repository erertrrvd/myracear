import glob, json, os, sys, datetime
for d in sorted(glob.glob(r'D:\ubuntu\slam\maps\run_2026110[13]_*')):
    f = os.path.join(d, '.run1-live.jsonl')
    if not os.path.exists(f):
        continue
    rows = []
    for line in open(f, encoding='utf-8', errors='replace'):
        try:
            rows.append(json.loads(line))
        except Exception:
            pass
    act = [r for r in rows if r.get('scans', 0) > 0]
    if not act:
        print(os.path.basename(d), 'no scans'); continue
    t = lambda r: datetime.datetime.fromisoformat(r['time_utc'])
    dur = (t(act[-1]) - t(act[0])).total_seconds()
    od = [r['odom'] for r in act if r.get('odom')]
    mv = [r for r in act if r.get('cmd_vel_messages')]
    pubs = max((r.get('link') or {}).get('odom_pubs', 0) or 0 for r in act)
    path = 0.0
    for a, b in zip(od, od[1:]):
        path += ((b[0]-a[0])**2 + (b[1]-a[1])**2) ** 0.5
    loc = t(act[0]).astimezone(datetime.timezone(datetime.timedelta(hours=8)))
    print(f"{os.path.basename(d):28s} {loc:%H:%M:%S} dur {dur:6.0f}s scans {act[-1]['scans']:6d} odom_msgs {act[-1].get('odometry_messages',0):6d} "
          f"path5s {path:6.1f} end ({od[-1][0]:.1f},{od[-1][1]:.1f}) pubs {pubs}" if od else f"{os.path.basename(d)} {dur:.0f}s no odom")
