import json, sys
for d in sys.argv[1:]:
    print('==', d)
    for l in open(d + '/run1/run.jsonl', encoding='utf-8'):
        r = json.loads(l)
        if r.get('corr'):
            print(r['time_utc'][11:19], 'odom', r['odom'], 'corr', r['corr'], 'scans', r.get('scans'))
